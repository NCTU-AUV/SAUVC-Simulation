import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import Altimeter
from std_msgs.msg import Float64


class AltimeterToPressureSensorNode(Node):

    def __init__(self):
        super().__init__('altimeter_to_pressure_sensor_node', namespace='orca_auv')
        self._altimeter_subscription = self.create_subscription(
            Altimeter,
            '/orca_auv/altimeter',
            self._altimeter_subscription_callback,
            10)
        self._altimeter_subscription  # prevent unused variable warning

        self._pressure_sensor_depth_publisher = self.create_publisher(Float64, 'pressure_sensor_depth_m', 10)

    def _altimeter_subscription_callback(self, msg):
        altitude_m = msg.vertical_position
        pressure_sensor_depth_m = -altitude_m

        publish_msg = Float64()
        publish_msg.data = pressure_sensor_depth_m

        self._pressure_sensor_depth_publisher.publish(publish_msg)


def main(args=None):
    rclpy.init(args=args)

    altimeter_to_pressure_sensor_node = AltimeterToPressureSensorNode()

    rclpy.spin(altimeter_to_pressure_sensor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    altimeter_to_pressure_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
