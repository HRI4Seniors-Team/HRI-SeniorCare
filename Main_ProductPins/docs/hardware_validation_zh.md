# MaixCam Pro + ESP32-S3 硬件实验与视觉联调方案

本文档是实际硬件到位后的执行规程。当前工作区没有 MaixCam Pro、ESP32-S3、模型文件或串口实测数据，因此不得把 `hardware_estimation` 文件夹中的数值写成实测结果。

## 1. 接线与安全

- MaixCam Pro A16/TX -> ESP32-S3 视觉 UART RX（当前工程默认 GPIO18）。
- MaixCam Pro A17/RX <- ESP32-S3 视觉 UART TX（当前工程默认 GPIO17）。
- 两块板共 GND；仅使用 3.3 V TTL 电平，禁止把 5 V 串口直接接入。
- MaixCam 调试建议另接 USB 串口。UART0 可能输出启动日志；若冲突，改用已确认引脚的 UART1，并同步修改 MaixPy 与 ESP32 配置。
- 第一次上电使用限流电源，确认没有反接、短路和异常发热。

## 2. 软件准备

### MaixCam Pro

1. 使用 MaixVision/SSH 将 `MaixCAM/resona_visual_node.py` 上传到设备。
2. 将官方模型放入 `/root/models/`：`yolov8n_face.mud` 与 `face_emotion.mud`。
3. 在设备上确认 MaixPy 版本支持官方 face-emotion API，并先运行模型加载/摄像头预览烟雾测试。
4. 确认 `UART_DEVICE`, A16/A17 引脚复用和 115200 波特率后运行脚本。

### ESP32-S3

工程目录为 `HRI-SeniorCare/Main`。在安装 ESP-IDF 的终端中执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

当前环境未安装 `idf.py`，所以本轮只能完成源码静态检查和主机端协议自测；烧录前需在 ESP-IDF 环境重新编译。

## 3. 分阶段联调

### Phase 0：不上电的准备

- 固化 Git 提交号、MaixPy 版本、两个 `.mud` 文件的 SHA-256、ESP-IDF 版本和板卡型号。
- 运行 `python -m unittest discover -s tools/hardware_validation/tests`。
- 生成实验清单：`generate_trial_manifest.py --actors 8 --repeats 5`。

### Phase 1：MaixCam 单机视觉

- 只接 MaixCam 和显示器，确认摄像头画面、单脸检测框、裁剪结果和四类概率。
- 无脸时必须发送 `face=false`，`quality<=0.05`，概率仍保持四维且和为 1。
- 有脸时记录 `lat_ms.capture/detect/align/fer/total`，确认 `other_mass` 表示被合并的 disgust/fear/surprise 概率质量。

### Phase 2：串口与 CRC

- 先做 USB-UART 回环，再接 ESP32-S3；TX/RX 交叉、共地。
- 观察 MaixCam 的 `PONG`、`STATE`、`ACK_TRIAL`，确认 ESP32 能解析 JSON。
- 人为修改一个 JSON 字节，ESP32 的 `crc` 字段应变为 0；恢复后连续 1000 包 CRC 全部有效。
- 检查序号无跳变、无缓冲区溢出、无重复包。

### Phase 3：ESP32 音频与融合

- 运行 60 s 空闲、60 s 讲话、60 s 视觉连续输入。
- 保存串口原始日志；`HWCSV,SER` 记录音频帧处理耗时、ready 和 heap，`HWCSV,FUSION` 记录融合耗时、冲突标志和 heap。
- 用 `analyze_logs.py` 输出 `records.csv` 与 `summary.json`。只引用 physical capture 文件中的统计量。

### Phase 4：控制变量视觉实验

- 固定相机距离、焦距、照明、背景和播放音量；每个 actor/condition/repetition 使用 `trial_manifest.csv` 中的唯一 `trial_id`。
- 先完成四类同情绪条件，再完成正负掩蔽和一般不一致条件；每段之间插入 3 s 空白帧。
- 每个 trial 开始前发送 `SET_TRIAL <trial_id>`，确认收到 `ACK_TRIAL` 后再呈现刺激。
- 可用 `python tools/hardware_validation/set_trial.py <trial_id> --port COMy` 发送标记；`COMy` 必须是 MaixCam 的控制串口，若视觉 UART 仍直接连接 ESP32，请使用串口切换器或在断开 ESP32 后设置，避免两个发送端同时驱动同一条 TX 线。
- 同步保存视频刺激标签、音频标签、MaixCam 包、ESP32 `HWCSV` 和异常备注。不得记录可识别的老年人身份信息。

## 4. 建议验收阈值

- JSON 概率四维、有限、归一化误差不超过 `1e-3`。
- 1000 个连续有效包 CRC 错误数为 0；序号缺口和重复数均为 0。
- 视觉 `total_ms`、SER 帧耗时和融合耗时报告 mean/p50/p95，而不是单次最好值。
- SER 处理时间必须小于音频 hop（当前实现为 15 ms）；融合耗时应小于 1 ms。
- 连续 10 min 运行中 heap 无单调下降趋势，且无 watchdog、UART overflow 或任务栈告警。
- 任何阈值不满足时，保留原始日志并记录固件、模型和接线版本，不得删除异常样本。

## 5. 日志格式

`HWCSV,VISION,host_us,seq,face,crc,quality,other_mass,capture_ms,detect_ms,align_ms,fer_ms,total_ms,trial`

`HWCSV,SER,host_us,seq,samples,elapsed_us,ready,free_heap`

`HWCSV,FUSION,host_us,seq,elapsed_us,high_conflict,free_heap,ser_ready`

`tools/hardware_validation` 中的分析器会计算计数、均值、p50、p95、脸检测率、CRC 错误数、序号缺口和 heap 相关字段。完成真实采集后，再把统计结果写回论文实验部分；在此之前只能作为联调记录。

## 6. 常见故障

- **无串口数据**：先确认 COM 口、波特率、TX/RX 交叉和共地；再检查是否被 UART0 启动日志干扰。
- **CRC 全错**：确认两端使用 ASCII 紧凑 JSON、CRC 多项式 `0x07`，且校验范围不包含 `,"crc"` 字段。
- **模型加载失败**：检查 `.mud` 路径、文件完整性和 MaixPy 版本；先单独运行官方 face-emotion 示例。
- **检测框为空**：增加光照和人脸尺寸，确认相机分辨率与模型输入一致；不要直接把无脸包当作中性情绪样本。
- **ESP32 无 HWCSV**：确认本次固件包含 `RESONA_HW_BENCHMARK_LOG=1`，并从 monitor 原始输出保存日志。
