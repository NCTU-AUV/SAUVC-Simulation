# SAUVC-Simulation —— Orca AUV 的 Gazebo 模擬

Gazebo Fortress 場景、水下成像、場地生成與 ROS 2 橋接。
依 [SAUVC 2026 rulebook](https://github.com/sauvc/rulebook) 建構，目標是讓實機訓練的
`finals.onnx` 不必重訓就能在模擬中穩定辨識。

**平常不用直接開這個 repo。** 從 super-repo 一個指令就會把模擬連同控制與感知堆疊一起拉起來：

```shell
cd ../          # SAUVC super-repo
make sim ARENA=finals SEED=2
```

底下是單獨開發本 repo 時用的流程。

---

## 這個 repo 提供什麼

```text
+-------------------------------------------------------------------------+
| Gazebo Fortress  (underwater world)                                     |
|                                                                         |
|   water_world.sdf --> entity_spawner --> arena props (randomised layout)|
|                                                                         |
|   orca_auv model: 8 thrusters + IMU + altimeter + front/bottom cameras  |
+-------------------------------------------------------------------------+
                |  ign topic
          +--------------------+
          | parameter_bridge   |  sensors, thruster forces
          +--------------------+
                |
      color/image_raw_dry --> underwater_camera --> color/image_raw
                                      (attenuation, scattering,
                                       white balance, noise)
```

| 目錄 | 內容 |
|---|---|
| `sim_ws/src/bringup/` | 世界檔、模型、launch、場地生成與資料集腳本 |
| `sim_ws/src/bridge/` | `underwater_camera_node`（水下成像）、altimeter → 壓力轉換 |

**水體是在影像層做的，不是 world 檔。** Ignition Fortress + ogre2 底下 `<scene><fog>`
會被靜默忽略（`gz-rendering` 沒有 fog 實作），所以改用 Gazebo 對齊的 depth buffer
逐像素套用水下成像模型。詳見 [../docs/SIM_VISUAL_FIDELITY.md](../docs/SIM_VISUAL_FIDELITY.md)。

---

## 單獨開發

### Ubuntu

```shell
make -f Makefile_ubuntu init      # 建容器 + colcon build
xhost +local:                     # 讓容器連得上 X server
make -f Makefile_ubuntu launch    # Gazebo + ROS bridge
```

前置：NVIDIA driver（`nvidia-smi`）、Docker + Compose plugin、以及能通過
`docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`。

### macOS

沒有 GPU passthrough，走 Gazebo 的 web visualization：

```shell
make init      # 產 TLS 憑證 + 建容器 + build（需要本機裝 mkcert）
make launch    # websocket server + Gazebo server + bridge
```

然後開 [app.gazebosim.org/visualization](https://app.gazebosim.org/visualization)
連到 `wss://localhost:9002`（實測 Safari 可用）。

### 常用 target

| 指令 | 用途 |
|---|---|
| `make init` | 建容器 + colcon build |
| `make launch` | 起 Gazebo 與 bridge |
| `make compose_shell` | 進容器 |
| `make compose_down` | 停掉容器 |

Ubuntu 一律加 `-f Makefile_ubuntu`。

---

## 啟動參數

`orca_ros_gz_bridge_launch.py` 的參數（super-repo 的 `make sim` 會幫你帶）：

| 參數 | 預設 | 說明 |
|---|---|---|
| `arena` | `qualification` | `finals` / `qualification`。**注意與 super-repo 的預設不同** |
| `seed` | 隨機 | 固定場地佈局。**必須是整數**，非整數會讓 spawner 在建構子裡崩潰 |
| `namespace` | `orca_auv` | 感測器與推進器話題的前綴 |
| `headless` | `false` | 不開 Gazebo GUI |
| `drum_style` | `random` | `drum`（圓桶）/ `tub`（方形塑膠箱）/ `random` |
| `randomize_water` | `false` | 每 20 秒重抽水質、能見度與曝光 |

```shell
ros2 launch bringup orca_ros_gz_bridge_launch.py arena:=finals seed:=2 headless:=true
```

相同的 `seed` 一定給出相同佈局，與 `drum_style` 無關。

---

## 場地

| Arena | 內容 |
|---|---|
| `finals` | 導航門、橘 flare、三個通訊 flare、目標桶區。道具位置每次隨機 |
| `qualification` | 懸掛式 q_gate，只有起始線隨機 |

道具模型在 `sim_ws/src/bringup/models/`：`gate`、`q_gate`、`orange_flare`、
`red/yellow/blue_flare`、`red/blue_drum`、`red/blue_tub`、`starting_zone`、
`qualification_start_line`、`pool_ground`。

> `water_surface` 是不透明盒（z ∈ [-0.03, -0.01]），GUI 從上方看會蓋住整個池子。
> 要檢視場地佈局時把這個 model 暫時關掉。

---

## 工具

全部在 `sim_ws/src/bringup/scripts/`，容器內以 `ros2 run bringup <script>` 執行。

**`capture_view.py`** —— 從任意視角截圖。截圖走與模擬完全相同的水下成像，
所以看到的就是感知端收到的畫面（`--dry` 看底下的乾畫面）。

```shell
ros2 run bringup capture_view.py --arena-views --out /root/captures --prefix after_
```

**`generate_dataset.py`** —— 自動標註的 YOLO 資料集產生器。bbox 由道具位姿解析投影而得。

```shell
ros2 run bringup generate_dataset.py --out /root/dataset --count 500 --profile finals
```

`--profile` 必須與啟動時的 `arena` 相符，否則找不到任何已知道具會直接退出。

**`make_pool_textures.py`** —— 重新烘焙池底與池壁貼圖（磁磚 + 水道線 + 端點 T 標記）。

**`entity_spawner.py`** —— 由 launch 自動執行，不用手動跑。它會寫一份
`/tmp/orca_arena_manifest.json` 記錄每個實體實際用了哪個模型 ——
實體名稱刻意與外形無關（tub 也叫 `blue_drum`），資料集產生器靠這份 manifest 分辨尺寸。

---

## 測試推進器

繞過控制堆疊直接推：

```shell
ign topic -t /orca_auv/thrusters/thruster_0/force_N -m ignition.msgs.Double -p 'data: 5'
```

座標慣例是**往下為正**（FRD 風格）：`+force.z` 下沉、`+torque.z` 右轉、
IMU 靜止時 `linear_acceleration.z` 讀 `-9.81`。不要套用 REP-103。
判斷轉向的實驗必須連續取樣並用 `np.unwrap` 解繞 —— 單點量測會被 ±π 繞回騙過。

---

## 新增模型

用 Blender 2.82 建模與貼圖，以
[sdf_exporter.py](https://github.com/gazebosim/gz-sim/blob/ign-gazebo5/examples/scripts/blender/sdf_exporter.py)
匯出成 SDF，流程見 [Blender SDF Exporter](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.17.0/tutorials/blender_sdf_exporter.md)。

改動道具幾何時要同步更新 `generate_dataset.py` 的 `PROP_EXTENTS` —— 那份是手動維護的，
不同步的後果是標註框系統性偏掉而沒有任何地方會發現。

---

## 相關文件

- [../docs/SIM_VISUAL_FIDELITY.md](../docs/SIM_VISUAL_FIDELITY.md) —— 場景改造、水下成像模型、量化驗證
- [../docs/HANDOFF.md](../docs/HANDOFF.md) —— 座標慣例、已知缺陷、踩過的坑
- [Gazebo web visualization](https://gazebosim.org/docs/fortress/web_visualization/)
