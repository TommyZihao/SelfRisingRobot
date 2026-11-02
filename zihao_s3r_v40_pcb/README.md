# RoBoBo 4.0-PCB

作者：同济子豪兄 2026-10-26。PCB 接口适配更新：2026-11-02。
原始机器人项目：HomeMadeGarbage / SelfRisingRobot。

本版本在 V3.9 基础上增加背部 PCB 舵机信号输出，保留 Grove 输出。开机画面显示 4.0-PCB。

| 舵机 | Grove GPIO | PCB GPIO | 范围 |
|---|---|---|---|
| 下方靠近底板 | G1 | G39 | ±90° |
| 上方连接长杆 | G2 | G38 | ±120° |

每对引脚输出相同的目标脉宽。每个舵机信号线只接其中一个引脚，不要把两个 GPIO 短接。PCB 的 5V 引脚接稳压后的 5V，不能接 7.4V 电池正极；舵机独立供电并共地。

四种模式、模型权重和固定目标 200Hz 姿态回传沿用 V3.9。模型 ZIP、MuJoCo XML 和电脑查看器继续使用仓库中的 `../zihao_s3r_v39/`；本目录包含完整 Arduino 源码和全部推理头文件。

## 编译与烧录

依赖与 V3.9 相同：m5stack ESP32 core 3.3.8、M5Unified 0.2.23、M5GFX 0.2.30、ESP32Servo 3.2.1、Kalman Filter Library 1.0.2。

```sh
arduino-cli compile --fqbn m5stack:esp32:m5stack_atoms3 --build-path /tmp/robobo-pcb firmware/robo03_atom_s3r_4_0_PCB
arduino-cli upload --fqbn m5stack:esp32:m5stack_atoms3 --port YOUR_PORT --input-dir /tmp/robobo-pcb firmware/robo03_atom_s3r_4_0_PCB
```

或将 `build/RoBoBo_4.0-PCB.merged.bin` 用 ESP32-S3 烧录工具写到地址 `0x0`。

编译通过：Flash 1419999 字节，静态 RAM 53428 字节。实际设备烧录哈希验证通过，串口确认 4.0-PCB 启动及 IMU 正常。G38/G39 舵机实际动作尚待现场验证。
