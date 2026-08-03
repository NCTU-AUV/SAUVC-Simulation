#!/usr/bin/env python3
"""Turn Gazebo's dry render into an underwater camera image.

Ignition Fortress cannot do this in the renderer. Its ogre2 backend implements
no fog, and gz-sim never reads `<scene><fog>` from SDF in the first place, so
there is no scene-level knob for water at all. What Gazebo *does* give us is an
aligned depth buffer alongside every colour frame, and with per-pixel range in
hand the underwater image formation model is directly computable:

    I_c = J_c * exp(-beta_c * z)  +  B_c * (1 - exp(-beta_c * z))
          \\__ direct transmission __/    \\__ backscattered veiling light __/

`J` is the dry render, `z` the range, `beta_c` the per-channel attenuation
coefficient and `B_c` the veiling light colour. Red attenuates roughly six
times faster than blue in pool water, which is why the competition photographs
show red drums as near-black lumps while blue ones stay obviously blue — a
distinction the detector depends on and that the dry render destroys.

On top of the physical model, `WaterColumn` also reproduces what the *camera*
does to the scene, which matters just as much for matching real footage:
scattering blur that grows with range, the RealSense's auto white balance
(which pulls a good part of the blue-green cast back out — skip it and the
output ends up far bluer than any real frame), auto exposure, sensor noise and
a slight vignette.

The model lives in `WaterColumn` rather than in the node so that offline tools
render water exactly the way the running simulation does; `capture_view.py`
uses it for its reference shots.
"""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# Attenuation coefficients (1/m) and veiling light for clear chlorinated pool
# water. These are not textbook values — they are fitted to the competition
# photographs (docs/sim-visual/reference/arena-2017.jpg, gate-2022.jpg) by
# inverting the model at two known ranges. Sampled there, the floor reads
# RGB (115, 209, 241) close up and (14, 153, 222) some 10 m out, so red must
# lose about eight times more energy over that span than blue does.
DEFAULT_BETA = (0.22, 0.050, 0.030)
DEFAULT_VEILING = (0.02, 0.30, 0.72)


class WaterColumn:
    """The water between the camera and the scene, plus the camera's response."""

    def __init__(self, **overrides):
        self.beta = np.asarray(DEFAULT_BETA, np.float32)
        self.veiling_colour = np.asarray(DEFAULT_VEILING, np.float32)
        self.veiling_gain = 1.0
        # Well beyond the 25 m pool: pixels with no geometry behind them are
        # pushed to this range, and they should converge all the way onto the
        # veiling colour rather than keeping a trace of the render's pale
        # background. Real depths are never clipped by it.
        self.max_range_m = 60.0
        self.blur_range_m = 14.0
        self.blur_max_sigma_px = 2.0
        # Deliberately weak: the real footage keeps most of its blue cast, so a
        # strong grey-world correction would grey out the very colour cue the
        # detector uses to separate red props from blue ones.
        self.awb_strength = 0.15
        self.exposure_target = 0.64
        self.exposure_strength = 0.7
        self.noise_sigma = 2.5
        self.vignette_strength = 0.18

        self._rng = np.random.default_rng()
        self._vignette = None
        self._vignette_shape = None
        self.update(**overrides)

    def update(self, **values) -> None:
        for key, value in values.items():
            if not hasattr(self, key):
                raise AttributeError(f'Unknown water parameter: {key}')
            current = getattr(self, key)
            setattr(self, key, np.asarray(value, np.float32)
                    if isinstance(current, np.ndarray) else type(current)(value))

    def seed(self, value: int) -> None:
        self._rng = np.random.default_rng(value or None)

    def randomize(self, beta_range, veiling_gain_range, exposure_target_range):
        """Draw a fresh water condition from the given ranges."""
        self.beta = np.array([self._rng.uniform(beta_range[0], beta_range[1]),
                              self._rng.uniform(beta_range[2], beta_range[3]),
                              self._rng.uniform(beta_range[4], beta_range[5])], np.float32)
        self.veiling_gain = float(self._rng.uniform(*veiling_gain_range))
        self.exposure_target = float(self._rng.uniform(*exposure_target_range))

    def clean_range(self, depth: np.ndarray) -> np.ndarray:
        """Per-pixel range in metres, with holes and sky pushed to max range.

        Pixels with no geometry behind them come back as 0, NaN or inf.
        Treating those as "infinitely far" rather than "touching the lens" is
        what turns the render's pale background into open water.
        """
        depth = np.nan_to_num(np.asarray(depth, np.float32),
                              nan=self.max_range_m, posinf=self.max_range_m)
        depth = np.where(depth <= 1e-3, self.max_range_m, depth)
        return np.clip(depth, 0.0, self.max_range_m)

    def _scatter_blur(self, image: np.ndarray, distance: np.ndarray) -> np.ndarray:
        """Blend a blurred copy in proportion to range.

        Scattering blur really varies per pixel; blending one globally blurred
        copy by a range-derived weight gets the visible effect for a fraction of
        the cost, which matters at 30 Hz.
        """
        if self.blur_max_sigma_px <= 0.0:
            return image
        blurred = cv2.GaussianBlur(image, (0, 0), self.blur_max_sigma_px)
        weight = np.clip(distance / max(self.blur_range_m, 1e-3), 0.0, 1.0)[:, :, None]
        return image * (1.0 - weight) + blurred * weight

    def _white_balance(self, image: np.ndarray) -> np.ndarray:
        """Partial grey-world correction, standing in for the sensor's AWB.

        A real RealSense does not hand you the raw cast; it corrects some of it
        away. Applying none of that leaves the output far bluer than any actual
        competition frame; applying all of it removes the colour cue the
        detector uses to tell red props from blue ones.
        """
        if self.awb_strength <= 0.0:
            return image
        means = image.reshape(-1, 3).mean(axis=0) + 1e-6
        gains = 1.0 + (means.mean() / means - 1.0) * self.awb_strength
        return image * gains

    def _auto_exposure(self, image: np.ndarray) -> np.ndarray:
        if self.exposure_strength <= 0.0:
            return image
        mean = float(image.mean()) + 1e-6
        gain = 1.0 + (self.exposure_target / mean - 1.0) * self.exposure_strength
        return image * np.clip(gain, 0.4, 2.5)

    def _apply_vignette(self, image: np.ndarray) -> np.ndarray:
        if self.vignette_strength <= 0.0:
            return image
        shape = image.shape[:2]
        if self._vignette_shape != shape:
            height, width = shape
            ys = np.linspace(-1.0, 1.0, height, dtype=np.float32)[:, None]
            xs = np.linspace(-1.0, 1.0, width, dtype=np.float32)[None, :]
            radius = np.sqrt(xs ** 2 + ys ** 2) / np.sqrt(2.0)
            self._vignette = (1.0 - self.vignette_strength * radius ** 2)[:, :, None]
            self._vignette_shape = shape
        return image * self._vignette

    def render(self, dry: np.ndarray, distance: np.ndarray) -> np.ndarray:
        """Apply the water column to a dry HxWx3 float image in [0, 1]."""
        transmission = np.exp(-distance[:, :, None] * self.beta[None, None, :])
        veiling = np.clip(self.veiling_colour * self.veiling_gain, 0.0, 1.0)
        wet = dry * transmission + veiling[None, None, :] * (1.0 - transmission)

        wet = self._scatter_blur(wet, distance)
        wet = self._white_balance(wet)
        wet = self._auto_exposure(wet)
        wet = self._apply_vignette(wet)
        if self.noise_sigma > 0.0:
            wet = wet + self._rng.normal(0.0, self.noise_sigma / 255.0, wet.shape)
        return np.clip(wet, 0.0, 1.0)


def decode_depth(msg: Image) -> np.ndarray:
    if msg.encoding in ('32FC1', '32FC'):
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
    if msg.encoding == '16UC1':
        raw = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
        return raw.astype(np.float32) * 1e-3
    raise ValueError(f'Unsupported depth encoding: {msg.encoding}')


class UnderwaterCamera(Node):
    def __init__(self) -> None:
        super().__init__('underwater_camera')

        self.declare_parameter('input_image_topic', 'color/image_raw_dry')
        self.declare_parameter('input_depth_topic', 'depth/image_raw')
        self.declare_parameter('output_image_topic', 'color/image_raw')

        self.declare_parameter('beta', list(DEFAULT_BETA))
        self.declare_parameter('veiling_colour', list(DEFAULT_VEILING))
        self.declare_parameter('veiling_gain', 1.0)
        self.declare_parameter('max_range_m', 60.0)
        self.declare_parameter('blur_range_m', 14.0)
        self.declare_parameter('blur_max_sigma_px', 2.0)
        self.declare_parameter('awb_strength', 0.15)
        self.declare_parameter('exposure_target', 0.64)
        self.declare_parameter('exposure_strength', 0.7)
        self.declare_parameter('noise_sigma', 2.5)
        self.declare_parameter('vignette_strength', 0.18)

        self.declare_parameter('randomize', False)
        self.declare_parameter('randomize_period_s', 20.0)
        self.declare_parameter('randomize_seed', 0)
        # [red_lo, red_hi, green_lo, green_hi, blue_lo, blue_hi]
        self.declare_parameter('beta_range', [0.18, 0.42, 0.035, 0.10, 0.02, 0.07])
        self.declare_parameter('veiling_gain_range', [0.75, 1.25])
        self.declare_parameter('exposure_target_range', [0.50, 0.70])

        self.water = WaterColumn(**{
            name: self.get_parameter(name).value
            for name in ('beta', 'veiling_colour', 'veiling_gain', 'max_range_m',
                         'blur_range_m', 'blur_max_sigma_px', 'awb_strength',
                         'exposure_target', 'exposure_strength', 'noise_sigma',
                         'vignette_strength')
        })
        self.water.seed(self.get_parameter('randomize_seed').value)

        self._lock = threading.Lock()
        self._depth = None

        if self.get_parameter('randomize').value:
            period = float(self.get_parameter('randomize_period_s').value)
            self.create_timer(period, self._roll_water)
            self.get_logger().info(f'Domain randomisation on, re-rolling every {period:.0f}s')

        self.publisher = self.create_publisher(
            Image, self.get_parameter('output_image_topic').value, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('input_depth_topic').value,
            self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('input_image_topic').value,
            self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f'Underwater camera: {self.get_parameter("input_image_topic").value} '
            f'-> {self.get_parameter("output_image_topic").value}  '
            f'beta={np.round(self.water.beta, 3).tolist()}')

    def _roll_water(self) -> None:
        self.water.randomize(self.get_parameter('beta_range').value,
                             self.get_parameter('veiling_gain_range').value,
                             self.get_parameter('exposure_target_range').value)
        self.get_logger().info(
            f'Water condition: beta={np.round(self.water.beta, 3).tolist()} '
            f'veiling_gain={self.water.veiling_gain:.2f} '
            f'exposure={self.water.exposure_target:.2f}')

    def _on_depth(self, msg: Image) -> None:
        with self._lock:
            self._depth = msg

    def _on_image(self, msg: Image) -> None:
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported colour encoding: {msg.encoding}', once=True)
            return

        dry = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, -1)[:, :, :3].astype(np.float32) / 255.0

        with self._lock:
            depth_msg = self._depth
        if depth_msg is None:
            distance = np.full((msg.height, msg.width), self.water.max_range_m, np.float32)
        else:
            try:
                depth = decode_depth(depth_msg)
            except ValueError as error:
                self.get_logger().warn(str(error), once=True)
                return
            if depth.shape != (msg.height, msg.width):
                depth = cv2.resize(depth, (msg.width, msg.height),
                                   interpolation=cv2.INTER_NEAREST)
            distance = self.water.clean_range(depth)

        wet = self.water.render(dry, distance)

        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.width * 3
        out.data = (wet * 255.0).astype(np.uint8).tobytes()
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
