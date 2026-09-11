# 简单版并联五连杆机器狗

这版只实现一条容易追踪的控制链：

```text
手柄 -> 状态机 -> 四只脚的 (x, z_down) -> 五连杆逆解 -> 8 个电机角 -> CAN
```

`x` 是脚相对本腿中心向前的距离，`z_down` 是脚相对电机轴向下的距离，单位都是 mm。
每条腿只在自己的二维平面里运动，没有主动侧向自由度。

## 文件地图：每个文件到底干什么

### 第一层：你需要看的 5 个文件

| 文件                                             | 作用                                | 速通阶段要不要改                           |
| ---------------------------------------------- | --------------------------------- | ---------------------------------- |
| `Control/robot_config.h` | 所有机械、电机、零位、姿态、步态和增益参数             | **要改，而且优先只改它**                     |
| `Control/Kinematics/fivebar.cpp/.h`        | 按同轴五连杆公式把一只脚 `(x,z_down)` 换成两个电机角 | 不改，先读注释                            |
| `Control/Gait/simple_gait.cpp/.h`    | 画抬脚/落脚轨迹，给四腿分配对角相位                | 不改，步态只在配置文件调参                      |
| `Application/Robot/robot_application.cpp/.h`   | 手柄、状态机、起立插值、故障保护、写入 8 个电机目标       | 不改，先看主流程 `robot_application_step_1ms()` |
| `System/Runtime/task/control_task.cpp`              | 定时器入口：TIM7 跑 1 ms 上层，TIM8 发电机 CAN | 不改                                 |

建议阅读顺序：`config -> gait -> fivebar -> simple_robot -> control_task`。

### 第二层：硬件底座，能工作就不要重写

| 文件                                                      | 作用                            | 什么时候才需要改               |
| ------------------------------------------------------- | ----------------------------- | ---------------------- |
| `System/Runtime/task/motor_task.cpp/.h`                      | 初始化 8 电机、保存反馈/命令、掉线/过温保护、定时发送 | 单电机标定时只改 `.h` 的 4 个标定宏 |
| `Platform/Drivers/motor_lz/motor_LZ.cpp/.h`              | 灵足电机 CAN 协议打包和解包              | 电机型号或协议变化才改            |
| `Platform/Drivers/remote_control/remote_control.cpp/.h` | 读取 PS2 手柄并生成摇杆/按键数据           | 手柄型号或按键协议变化才改          |
| `Platform/Bsp/CAN/bsp_fdcan.cpp/.h`                    | STM32 FDCAN 收发回调和底层接口         | CAN 外设或过滤器变化才改         |

### 第三层：CubeMX/芯片生成文件，入门阶段不看

| 文件或目录                                     | 作用                                    |
| ----------------------------------------- | ------------------------------------- |
| `Core/Src/main.c`                         | 芯片启动、外设初始化，然后调用 `control_task_init()` |
| `Core/Src/fdcan.c`                        | CAN1/CAN2/CAN3 波特率和外设配置               |
| `Core/Src/tim.c`                          | TIM7/TIM8 等定时器配置                      |
| `Core/Src/gpio.c`, `spi.c`, `usart.c`     | GPIO、SPI、串口初始化                        |
| `Drivers/`, `Middlewares/`, `USB_DEVICE/` | STM32 HAL、CMSIS、USB 官方代码              |
| `dm02.ioc`                                | CubeMX 工程配置源文件                        |
| `MDK-ARM/dm02.uvprojx`                    | Keil 编译文件清单和头文件路径                     |

这些文件不是“机器狗算法”。当前外设能启动、CAN 能收到反馈，就不要为了简洁去重写它们。

### 辅助工具和旧代码

| 文件                           | 作用                                   |
| ---------------------------- | ------------------------------------ |
| `tools/verify_robot8.py`     | 直接读取固件配置，离线检查映射、工作空间和轨迹连续性           |
| `tools/calc_fivebar_zero.py` | 根据实测足端位置和电机反馈计算同轴五连杆软件零位             |
| `Control/Algorithms/robot/` | 旧运动学、旧步态和旧控制，保留作参考，**不参与当前 Keil 编译** |

## 真正让机器狗动起来，只需要改什么

绝大部分情况下只改两个文件：

```text
Control/robot_config.h  实机参数和最终动作开关
System/Runtime/task/motor_task.h                   单电机标定开关
```

必须按以下顺序，不能跳步骤：

1. **机械尺寸**：确认两电机同轴，并测量主动杆、从动杆，填写配置文件第 1 区。
2. **CAN 映射**：确认 RF/LF/RH/LH 分别对应哪个 BUS 和 ID，填写第 2 区。
3. **电机顺序**：确认 MOTOR1/MOTOR2 对应旧算法 theta1/theta2 的顺序。
4. **方向**：一次只点动一个电机，填写左右各自的 `SIGN`，只能是 `+1` 或 `-1`。
5. **软件零位**：夹具固定脚尖，记录两个反馈，用脚本计算并填写 8 个 `ZERO`。
6. **离线验证**：运行 `python .\tools\verify_robot8.py`，必须全部 PASS。
7. **动作授权**：以上都确认后，最后才把 `SIMPLE_ROBOT_CALIBRATED` 从 `0U` 改为 `1U`。
8. **架空验收**：先按 START，只观察四腿是否缓慢同向到站姿；任何一腿异常立即断电。
9. **落地验收**：吊带承重，从站立开始；最后才试 12 mm 小步长慢走。

不要为了“先动起来”直接把第 7 步提前。五连杆零位或映射错误时，高 Kp 会让闭链互相顶死。

## 目前唯一参数源

不要再到多个 `.cpp` 和 Python 脚本中分别修改参数。只修改：

```text
Control/robot_config.h
```

当前临时参数：

| 参数     | 值                   |
| ------ | -------------------:|
| 电机布置   | 面对面同轴，轴向安装距离不参与二维逆解 |
| 主动杆    | 110 mm，已按实物确认       |
| 被动杆    | 200 mm              |
| 站姿     | x=0，向下 180 mm       |
| 趴姿     | x=0，向下 130 mm       |
| 最大前后步长 | 12 mm               |
| 最大转向步长 | 8 mm                |
| 抬脚高度   | 8 mm                |
| 步态周期   | 1.50-2.20 s         |

`tools/verify_robot8.py` 会直接读取这份头文件，因此不会再验证另一套过期参数。

## 状态机

Keil Watch 查看 `simple_robot_state_watch`：

| 值   | 状态       | 行为              |
| ---:| -------- | --------------- |
| 0   | WAIT     | 上电等待，只给阻尼       |
| 1   | GET_UP   | 先到趴姿，再平滑起立      |
| 2   | STAND    | 回到并保持标准站姿       |
| 3   | WALK     | 对角步态前后走         |
| 4   | TURN     | 左右腿反向步进，原地转弯    |
| 5   | GET_DOWN | 平滑趴下            |
| 6   | FAULT    | 统一撤销 8 个电机的软件武装 |

遥控器：

- START：等待/趴下时请求起立；运动时回站姿。
- SELECT：请求趴下。
- 左摇杆上下：前后走。
- 右摇杆左右：原地转向。
- 换方向或在走/转之间切换前，两个摇杆必须先回中。
- 遥控失联时，走和转会退回站姿，不会继续使用旧命令。

## 为什么现在按键不会运动

`robot_config.h` 中：

```c
#define SIMPLE_ROBOT_CALIBRATED 0U
```

这是有意的安全锁。机械尺寸已经确认，但当前 8 个软件零位仍是 0，电机方向也尚未
逐台验证。在这些数据没有确认前，START/SELECT 会进入故障，不会输出姿态动作。

只有完成以下四项后，才能把它改成 `1U`：

1. 复核同轴结构、主动杆和从动杆长度。
2. 确认每条腿的 CAN BUS/ID 映射。
3. 确认两个电机的左右顺序和各自正方向。
4. 填好 8 个 `SIMPLE_ZERO_*` 软件零位。

## 单电机标定

### 独立 CAN 收发诊断

`motor_task.h` 中：

```c
#define ROBOT_CAN_DIAGNOSTIC_ENABLE 1
#define ROBOT_CAN_DIAGNOSTIC_TX_ENABLE 0
```

`TX_ENABLE=0` 是纯接收测试：三路 CAN 只启动过滤器、通知和中断，不主动发送帧。
PCAN 向 CAN2 发送标准帧 `0x123` 后，应观察 `can_diag_rx_frame_count[1]` 增长。

将 `TX_ENABLE` 改为 1 后，启动立即发送，并由 `main()` 主循环每 500 ms
发送一次普通标准测试帧：CAN1/CAN2/CAN3 分别使用 ID `0x123/0x124/0x125`，数据前
4 字节分别是 ASCII `CAN1/CAN2/CAN3`，后 4 字节是递增序号。它不依赖 TIM8，也不会
被灵足电机识别为扩展控制命令。

Keil Watch 添加：

```text
can_diag_filter_ok
can_diag_start_status
can_diag_notification_status
motor_can_diag_ping_count
can_diag_tx_attempt_count
can_diag_tx_ok_count
can_diag_tx_error_count
can_diag_tx_fifo_free
can_diag_rx_irq_count
can_diag_rx_frame_count
can_diag_last_rx_identifier
can_diag_last_rx_dlc
can_diag_last_error_code
can_diag_bus_off
can_diag_error_passive
can_diag_tx_error_counter
can_diag_rx_error_counter
```

下标 0/1 对应 CAN1/CAN2。正常时 filter=1，start/notification=0，ping/tx_ok/rx_frame
持续增长，tx_error=0，bus_off=0。tx_ok 只代表成功进入硬件 FIFO；如果 rx_frame=0
且 tx_error_counter 增长，通常是无 ACK、波特率、终端电阻、收发器供电或 CANH/CANL 接线问题。

诊断结束必须恢复：

```c
#define ROBOT_CAN_DIAGNOSTIC_ENABLE 0
```

并重新全量编译，才能进行八电机使能和单电机标定。

标定前必须把整机架空，并准备立即断电。配置在 `System/Runtime/task/motor_task.h`：

```c
#define ROBOT_MOTOR_CALIBRATION_ENABLE       0
#define ROBOT_MOTOR_CALIBRATION_BUS          0U
#define ROBOT_MOTOR_CALIBRATION_INDEX        0U
#define ROBOT_MOTOR_CALIBRATION_DELTA_RAD    0.0f
```

步骤：

1. 保持 `DELTA_RAD=0.0f`，只选择一个 BUS/INDEX，确认 Watch 中反馈来自预期电机。
2. 架空后最多先改为 `0.02f`，观察正方向；其余 7 个电机只有阻尼。
3. 测完立即恢复 `ENABLE=0` 和 `DELTA_RAD=0.0f`，再全量重建生产固件。
4. 将两根主动杆水平反向伸出；脚端应在共同电机轴正下方约 `167.03 mm`。
5. 记录该腿两个电机反馈，并运行零位计算：

```powershell
python .\tools\calc_fivebar_zero.py --leg RF --motor1 0.0 --motor2 0.0
```

脚本默认读取配置中的 `x=0、z=167.03293 mm`，并直接输出该腿两个
`SIMPLE_ZERO_*` 值。实物没有摆到默认夹具位置时，必须传入实测的 `--x-mm/--z-mm`。

当前映射下，标定宏和腿的对应关系为：

| 腿                | BUS | INDEX | CAN ID     |
| ---------------- | ---:| -----:| ----------:|
| RF motor1/motor2 | 0   | 0/1   | CAN1 ID1/2 |
| LF motor1/motor2 | 0   | 2/3   | CAN1 ID3/4 |
| RH motor1/motor2 | 1   | 0/1   | CAN2 ID1/2 |
| LH motor1/motor2 | 1   | 2/3   | CAN2 ID3/4 |

如果实物接线不是这样，只改 `robot_config.h` 第 2 区，不要去改逆运动学公式。

## 故障观察

Keil Watch 查看：

- `simple_robot_fault_watch`
- `simple_robot_fault_leg_watch`
- `simple_robot_fault_motor_watch`

故障值：

| 值   | 含义                    |
| ---:| --------------------- |
| 1   | 未完成标定，动作被锁止           |
| 2   | 电机未就绪、掉线或底层故障         |
| 3   | 电机位置反馈不是有限数           |
| 4   | 足端不可达、支链跳变或闭链姿态无效     |
| 5   | 目标角越界，或姿态变化超过 1.2 rad |

故障只保留第一个原因，并统一失能。生产状态下不自动清故障，应断电检查原因。

## 每次改参数后的检查

```powershell
python .\tools\verify_robot8.py
```

检查通过只能说明几何目标可达、轨迹连续，不能代替架空上电、机械干涉检查和零位标定。
