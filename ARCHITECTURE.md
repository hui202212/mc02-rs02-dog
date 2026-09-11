# DM02 Software Architecture

This firmware controls a four-legged robot with eight CAN-connected LZ servo
motors. The MCU is responsible for robot-level motion control. Current loops,
SVPWM, FOC, and internal position/velocity loops are implemented inside the
commercial motor controllers and are not claimed by this repository.

## Four-layer design

```text
Application
  Robot modes, remote-command policy, stand/walk/crawl/jump sequences
      |
Motion control
  Gait generation, foot trajectories, five-bar FK/IK, attitude compensation
      |
Real-time runtime
  TIM7 1 kHz control scheduling, TIM8 CAN transmission, IMU service,
  motor health checks, command/feedback ports
      |
Platform and drivers
  CubeMX HAL, FDCAN, SPI, GPIO, BMI088, PS2 protocol, LZ CAN protocol
```

The project currently uses a timer-driven bare-metal runtime, not FreeRTOS.
`TIM7` runs the 1 kHz robot application step, `TIM8` sends motor frames, and
the main loop performs the slower BMI088 SPI service. This separation is
deliberate: the SPI transaction is not permitted to block a motor CAN ISR.

## Ownership

| Layer | Directory | Responsibility |
| --- | --- | --- |
| Platform and drivers | `Core/`, `Platform/` | Hardware initialization and device protocols only. |
| Real-time runtime | `System/Runtime/` | Scheduling, timestamping, health supervision, and the `robot_io` port. |
| Motion control | `Control/` | Robot configuration, gait generation, kinematics, filters, and reusable algorithms. |
| Application | `Application/` | Robot behavior state machine and action sequencing. |

`System/Runtime/robot_io.*` is the boundary between the motion/application
code and LZ motor protocol. Callers use a leg and joint index to obtain a
feedback position or submit a joint target; they do not select a CAN bus,
motor ID, or manipulate `motor_lz_data_canX[]` directly.

## Runtime sequence

```text
TIM7, 1 kHz:  robot_app_step_1ms()
                -> robot state / gait / IK
                -> robot_io_set_joint_target()

TIM8:         motor_task()
                -> LZ command encoding
                -> FDCAN transmit

main loop:    IMU_Task_1ms() and CAN diagnostics
```

## Dependency rules

- `Platform` must not include headers from `System`, `Control`, or `Application`.
- `System` may use `Platform` and exposes ports such as `robot_io` upward.
- `Control` must use ports and data types, never HAL or FDCAN APIs.
- `Application` selects behavior and invokes control paths; it must not encode CAN frames.
- Bench and calibration paths are build-time guarded and remain disabled in the production configuration.

## Safety model

Motor feedback freshness, temperature/error checks, and software disarming
are centralized in `motor_task`. IK is solved for all four legs before a set
of eight targets is committed. The application retains the first fault reason
for debugging, and the runtime can disarm all motors through one interface.
