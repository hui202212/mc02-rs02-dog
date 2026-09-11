# hui202212 | Embedded Robotics Projects

这是我的嵌入式与机器人项目资料库。仓库按“硬件、软件、文档、视频、图片”组织，
方便快速了解项目背景、工程实现和实机验证过程。

## 目录

| 目录 | 内容 |
| --- | --- |
| [1.Hardware](1.Hardware/) | 机械结构、原理图、PCB、BOM、装配和硬件版本记录。 |
| [2.Software](2.Software/) | 固件、上位机和其他软件工程。 |
| [3.Document](3.Document/) | 架构、标定、调试记录、设计说明和实验结论。 |
| [4.Video](4.Video/) | 实机演示、调试过程和功能验证视频。 |
| [5.Images](5.Images/) | 机器人照片、结构图、波形图和测试截图。 |
| [README.assets](README.assets/) | 根 README 使用的图片等资源。 |

## 重点项目

### DM02 四足机器狗运动控制器

基于 `STM32H723`、双 FDCAN 和 8 个 LZ 伺服电机的四足机器人控制固件。
项目覆盖机器人状态机、步态生成、五连杆 FK/IK、足端轨迹、BMI088 姿态解算、
电机健康监测和实机安全流程。

详细入口：[2.Software/DM02](2.Software/DM02/)

| 维度 | 已实现内容 |
| --- | --- |
| 控制周期 | `TIM7` 驱动的 `1 kHz` 机器人控制循环 |
| 运动控制 | 对角步态、起立、站立、趴下、行走、转向 |
| 运动学 | 平面五连杆 FK/IK 与工作空间保护 |
| 设备通信 | 双 FDCAN、LZ 电机协议、BMI088 SPI、PS2 遥控 |
| 工程验证 | Keil 全量构建、参数和轨迹离线检查 |

## 阅读顺序

1. 先看 [DM02 软件工程](2.Software/DM02/README.md)。
2. 再看 [软件架构](3.Document/ARCHITECTURE.md) 和 [实机标定流程](3.Document/ROBOT_BRINGUP.md)。
3. 最后查看 [硬件资料](1.Hardware/)、[视频](4.Video/) 和 [图片](5.Images/)。

## 资料补充约定

- 硬件资料放入 `1.Hardware/`，按版本或模块建立子目录。
- 演示视频放入 `4.Video/`，文件名包含日期、功能和验证结果。
- 图片放入 `5.Images/`；README 中使用的图片放入 `README.assets/`。
- 调试过程优先沉淀为 `3.Document/` 中的 Markdown，而不是只保留聊天记录。
