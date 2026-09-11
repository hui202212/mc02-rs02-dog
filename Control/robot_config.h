#ifndef SIMPLE_ROBOT_CONFIG_H
#define SIMPLE_ROBOT_CONFIG_H

/*
 * 简单版机器狗唯一参数文件
 *
 * 长度：mm，角度：rad，时间：s。
 * 固件、tools/verify_robot8.py 和 tools/calc_fivebar_zero.py 共用这里的参数。
 * 实机调试原则：先改几何和电机映射，再改零位，最后才调步态和 Kp/Kd。
 */

#define SIMPLE_LEG_COUNT                 4U
#define SIMPLE_MOTORS_PER_LEG            2U

/*============================== 1. 机械尺寸 ==============================*/
/* 已按实物确认：两个电机面对面同轴，侧视投影中共用一个旋转中心。
 * 115 mm 是轴向安装尺寸，不参与二维足端逆解。主动杆 110 mm、从动杆 200 mm。 */
#define SIMPLE_ACTIVE_LENGTH_MM          110.0f
#define SIMPLE_PASSIVE_LENGTH_MM         200.0f

/* 旧算法 theta=(A-90deg)，但当前灵足电机实测该方向会让腿收短。
 * 2026-07-12 前腿双电机协同验证后取 -1，使 +Z_DOWN 命令实际向下伸腿。 */
#define SIMPLE_COAXIAL_THETA_SIGN        -1.0f
/* 参考旧工程的 x 正方向在实机上朝狗尾；这里取 -1，统一为 +X 朝狗头。 */
#define SIMPLE_COAXIAL_X_SIGN            -1.0f

/* 工作空间安全余量。正常速通不需要改这两个值。 */
#define SIMPLE_WORKSPACE_MARGIN_MM       10.0f
#define SIMPLE_MIN_Z_DOWN_MM             20.0f

/*============================== 2. 电机映射 ==============================*/
/*
 * BUS: 0=CAN1，1=CAN2。
 * INDEX: motor_lz_data_canX[] 下标，0/1/2/3 对应电机 ID 1/2/3/4。
 * MOTOR1/MOTOR2 是该腿传给五连杆模块的顺序，不等同于几何左/右。
 */
#define SIMPLE_RF_BUS                    0U
#define SIMPLE_RF_MOTOR1_INDEX           0U
#define SIMPLE_RF_MOTOR2_INDEX           1U
#define SIMPLE_LF_BUS                    0U
#define SIMPLE_LF_MOTOR1_INDEX           2U
#define SIMPLE_LF_MOTOR2_INDEX           3U
#define SIMPLE_RH_BUS                    1U
#define SIMPLE_RH_MOTOR1_INDEX           0U
#define SIMPLE_RH_MOTOR2_INDEX           1U
#define SIMPLE_LH_BUS                    1U
#define SIMPLE_LH_MOTOR1_INDEX           2U
#define SIMPLE_LH_MOTOR2_INDEX           3U

/* 旧算法 theta1/theta2 到 CAN 对内顺序：1 表示 MOTOR1 对应 theta2。 */
#define SIMPLE_RF_RIGHT_MOTOR_FIRST      1U
#define SIMPLE_LF_RIGHT_MOTOR_FIRST      0U
#define SIMPLE_RH_RIGHT_MOTOR_FIRST      1U
#define SIMPLE_LH_RIGHT_MOTOR_FIRST      0U

/*=========================== 3. 方向和软件零位 ===========================*/
/*
 * motor = sign * (theta - 软件零位)
 * 面对面安装导致同一条腿的两根主动杆从外侧看旋向相反，但编码器数学符号
 * 相同。旧工程映射为右侧腿两台 -1，左侧腿两台 +1。
 */
/* 实测旋向保留为装配核对依据：RF ID1逆时针/ID2顺时针，
 * LF ID3逆时针/ID4顺时针（均从对应腿外侧朝机身观察）。 */
#define SIMPLE_SIGN_RF_LEFT             -1.0f
#define SIMPLE_SIGN_RF_RIGHT            -1.0f
#define SIMPLE_SIGN_LF_LEFT              1.0f
#define SIMPLE_SIGN_LF_RIGHT             1.0f
#define SIMPLE_SIGN_RH_LEFT             -1.0f
#define SIMPLE_SIGN_RH_RIGHT            -1.0f
#define SIMPLE_SIGN_LH_LEFT              1.0f
#define SIMPLE_SIGN_LH_RIGHT             1.0f

/* 电机反馈为 0 时，对应旧算法 theta1/theta2 的角度。
 * 恢复到“第一次四腿重标零 + L1提速一倍”实机验证可正常行走的标定值。
 * 左前、右后和左后使用与当前反馈相邻的等价圈数，防止跨2*pi追赶多圈。 */
#define SIMPLE_ZERO_RF_LEFT_RAD         -1.15946579f   // 右前左电机，CAN1 ID2
#define SIMPLE_ZERO_RF_RIGHT_RAD        -2.05596733f   // 右前右电机，CAN1 ID1
#define SIMPLE_ZERO_LF_LEFT_RAD         -3.00963432f   // 左前左电机，CAN1 ID3，等价于3.27355099
#define SIMPLE_ZERO_LF_RIGHT_RAD        -0.0569667816f // 左前右电机，CAN1 ID4
#define SIMPLE_ZERO_RH_LEFT_RAD          0.92662222f   // 右后左电机，CAN2 ID2，等价于-5.35656309
#define SIMPLE_ZERO_RH_RIGHT_RAD         1.06241322f   // 右后右电机，CAN2 ID1
#define SIMPLE_ZERO_LH_LEFT_RAD          1.13338089f   // 左后左电机，CAN2 ID3
#define SIMPLE_ZERO_LH_RIGHT_RAD         2.86386139f   // 左后右电机，CAN2 ID4，等价于-3.41932392

/* 软件零位夹具姿态：两根 110 mm 主动杆水平反向伸出。
 * 足端位于共同电机轴正下方，理论距离 sqrt(200^2-110^2)=167.03293 mm；
 * 此时参考算法 theta1=theta2=0。 */
#define SIMPLE_ZERO_CALIBRATION_X_MM      0.0f
#define SIMPLE_ZERO_CALIBRATION_Z_MM      167.03293f

/*============================== 4. 基本姿态 ==============================*/
/* 本腿局部坐标：+X 朝狗头，-X 朝狗尾，+Z_DOWN 朝地面。
 * 下面是最常改的姿态参数，单位都是 mm：
 * - STAND_X：四脚站立中心的前后位置；正常保持 0，不要用它修单腿偏差。
 * - STAND_Z：电机轴到脚尖的距离。数值越大腿越长、机身越高；数值越小机身越低。
 *   当前机构建议先在 190~220 mm 内调，过大接近伸直奇异位，过小容易贴地。
 * - DOWN_Z：趴下时腿长；减小会趴得更低，但要确认机身和连杆不碰地。
 * - CRAWL_Z：匍匐时腿长；最好与 DOWN_Z 接近，避免进入匍匐时突然跳动。 */
#define SIMPLE_STAND_X_MM                0.0f       // 四腿公共站立中心，通常不要改
#define SIMPLE_STAND_Z_DOWN_MM           200.0f     // 站立腿长：大=机身高，小=机身低
#define SIMPLE_DOWN_X_MM                 0.0f       // 趴姿公共前后中心，通常保持 0
#define SIMPLE_DOWN_Z_DOWN_MM            150.0f     // 趴下腿长
#define SIMPLE_CRAWL_Z_DOWN_MM           150.0f     // 匍匐腿长

/* 每条腿独立的前后安装修正，只用于“某一条腿静止时前后偏了”：
 * RF=右前，LF=左前，RH=右后，LH=左后。
 * +X 把该脚向狗头修正，-X 把该脚向狗尾修正。
 * 例：左后脚实测向后偏 6 mm，可先把 LH 填 +4~+5 mm，确认方向后再微调。
 * 不要凭感觉一次填很大；如果目标与电机反馈相差明显，应先排查摩擦/跟随而不是补偿。 */
#define SIMPLE_RF_X_TRIM_MM              9.0f       // 右前脚独立前后修正
#define SIMPLE_LF_X_TRIM_MM              6.0f       // 左前脚独立前后修正
#define SIMPLE_RH_X_TRIM_MM              9.0f       // 右后脚独立前后修正
#define SIMPLE_LH_X_TRIM_MM              15.0f       // 左后脚独立前后修正
#define SIMPLE_X_TRIM_MAX_ABS_MM         20.0f      // 单腿修正绝对值上限，不建议放大

/* 单腿上下补偿：+Z让腿伸长，-Z让腿缩短。
 * 左后腿卡住后若落地时左后机身角偏高，可只把LH设为负值；这不是重新标零。
 * 当前先让左后腿缩短3 mm。仍偏高时不要直接扩大，先确认高速轨迹速度余量。
 * 补偿只能修正小量几何偏差；如果连杆仍发卡，必须先处理机械问题。 */
#define SIMPLE_RF_Z_TRIM_MM               0.0f       // 右前上下补偿
#define SIMPLE_LF_Z_TRIM_MM              -3.0f       // 左前机身角偏高：左前腿缩短3 mm
#define SIMPLE_RH_Z_TRIM_MM               0.0f       // 右后上下补偿
#define SIMPLE_LH_Z_TRIM_MM              -3.0f       // 左后缩短3 mm，压低左后机身角
#define SIMPLE_Z_TRIM_MAX_ABS_MM          8.0f       // 单腿上下补偿绝对值上限

/* 仅静止STAND状态使用的左右带载调平：右侧伸长、左侧缩短。
 * 数值3表示左右各修正3 mm，总目标差高6 mm；不影响起立前段、L1和跳跃。 */
#define SIMPLE_STAND_SIDE_Z_COMP_MM       3.0f       // 左高右低时增大，建议每次只加1 mm
#define SIMPLE_STAND_SIDE_Z_COMP_MAX_MM   8.0f       // 静止站姿单侧补偿上限

/*============================== 5. 遥控步态 ==============================*/
/* 正常行走使用 RF+LH / LF+RH 两组对角腿交替；匍匐使用单腿四拍。
 * 调参建议一次只改一类：先周期，再步幅，再步高，最后才碰 Kp/Kd。 */
#define SIMPLE_ARTICLE_TRAJECTORY_ENABLE 1U
/* 1=支撑相匀速直线、摆动相复合摆线+余弦抬脚；0=回退到旧平台抬脚轨迹。
 * 新轨迹已经通过离线连续性检查，正常保持 1。 */

#define SIMPLE_WALK_SWING_RATIO          0.40f
/* 单腿“抬离地面”的时间占整个周期的比例，不是步幅：
 * 支撑占空比 = 1 - SWING_RATIO。当前 0.40 表示每腿 60% 时间接地，
 * 整个周期仍有约 20% 时间四脚同时接地。
 * 减小：接地更久、更稳但动作偏慢；增大：腾空更久、动态性更强。
 * 无 IMU 时建议约 0.32~0.45，不建议直接设成 0.50。 */

#define SIMPLE_WALK_PERIOD_MIN_S         0.32f
#define SIMPLE_WALK_PERIOD_MAX_S         0.46f
/* 一个完整对角步态周期的时间：数值越小换腿越快。
 * MIN 是摇杆推满时的最快周期，MAX 是刚过死区时的最慢周期。
 * 腿跟不上或频繁拖地：两个值一起增加；想提高频率：两个值一起小幅减小。
 * 每次建议只改 0.05~0.10 s，MIN 必须小于等于 MAX。 */

#define SIMPLE_CRAWL_SWING_RATIO         0.24f      // 木桥四拍每次只抬一腿，其余三腿保持接地
#define SIMPLE_CRAWL_PERIOD_MIN_S        0.45f      // 木桥满摇杆周期；保证能推动机身且不过快
#define SIMPLE_CRAWL_PERIOD_MAX_S        0.65f      // 木桥小摇杆周期

#define SIMPLE_FAST_CRAWL_ENABLE         1U
/* 1=蹲下后使用高频对角小碎步；0=恢复上面的慢速四拍匍匐。
 * 高频模式每次抬一组对角腿，速度明显更快，但对地面摩擦和机械跟随要求更高。 */
#define SIMPLE_WALK_USE_LOW_POSE         1U
/* 1=站立时推左摇杆，也先自动蹲下再用低姿快速步态；0=保留原来的高姿行走。
 * 当前目标是全部采用低姿行走，因此保持 1。 */
#define SIMPLE_TURN_USE_LOW_POSE         1U
/* 1=右摇杆转弯也先蹲下，支持低姿原地转弯和边走边转；0=恢复高姿原地转弯。 */
#define SIMPLE_FAST_CRAWL_SWING_RATIO    0.44f      // 56% 时间接地，仅保留 12% 双支撑
#define SIMPLE_FAST_CRAWL_PERIOD_MIN_S   0.18f      // 满摇杆约5.6Hz；避免右前+左后对角组跟随不及而抖动
#define SIMPLE_FAST_CRAWL_PERIOD_MAX_S   0.28f      // 小摇杆约3.6Hz；保留小碎步速度并增加落脚稳定时间

#define SIMPLE_GAIT_RAMP_S               0.20f
/* 水平步幅从 0 建立到目标值所需时间。增大更柔和但起步拖沓，减小响应更快但冲击更大。
 * 抬脚高度不再乘这个慢斜坡，因此第一步也能完整抬脚。 */

#define SIMPLE_COMMAND_SLEW_S            0.06f
/* 摇杆命令从 0 变化到满量程的最短时间，用于过滤手抖。
 * 增大更平滑，减小更跟手；过小会造成步幅和周期突然变化。 */

#define SIMPLE_WALK_STEP_MAX_MM          44.0f
/* 满摇杆时脚尖前后“总行程”，不是单边距离：44 mm 表示 -22~+22 mm。
 * 增大走得更远但更易打滑/扭身；减小更像快速小碎步。建议每次只增加 4~8 mm。 */

#define SIMPLE_TURN_STEP_MAX_MM          16.0f
/* 原地转弯时左右两侧反向步幅。太小转不动，太大容易侧滑；建议每次增加 2 mm。 */

#define SIMPLE_STEP_HEIGHT_MM            30.0f
/* 正常行走抬脚高度。当前四条腿统一为 30 mm；
 * 太大会导致机身上下跳、落脚冲击和不稳定。单腿拖地不要先改这个公共值。 */

/* 正常行走/原地转弯的单腿额外抬脚量，不影响站立高度和匍匐步态。
 * 某腿总抬脚高度 = SIMPLE_STEP_HEIGHT_MM + 对应 LIFT_EXTRA。
 * 例：公共步高 18、RF_EXTRA 3，右前实际目标步高就是 21 mm。
 * 仅在确认某一腿持续拖地时使用，建议从 2~3 mm 开始，不要超过下面的上限。 */
#define SIMPLE_RF_LIFT_EXTRA_MM          0.0f       // 右前额外抬高
#define SIMPLE_LF_LIFT_EXTRA_MM          0.0f       // 左前额外抬高
#define SIMPLE_RH_LIFT_EXTRA_MM          0.0f       // 右后额外抬高
#define SIMPLE_LH_LIFT_EXTRA_MM          0.0f       // 左后额外抬高
#define SIMPLE_LIFT_EXTRA_MAX_MM         8.0f       // 单腿额外抬脚上限

#define SIMPLE_CRAWL_STEP_MAX_MM         32.0f      // 高频步幅略收小，降低对角支撑切换时的机身扭动
#define SIMPLE_CRAWL_TURN_STEP_MAX_MM    36.0f      // 低姿转弯总步幅；左右侧自动反向
#define SIMPLE_CRAWL_STEP_HEIGHT_MM      22.0f      // 降低高频竖直加速度，四腿仍保持相同步高

/*======================= 6. 新规则障碍赛步态 ===========================*/
/* 新生演示版默认直接使用高速步态；木桥、斜坡、T台等比赛专用入口关闭。
 * 底层参数保留，方便以后切回比赛固件，不参与本次演示遥控。 */
#define SIMPLE_OBSTACLE_MODE_ENABLE      0U

/* L1高速转场专用周期：54mm/0.11s约491mm/s，是旧244mm/s的约2.01倍。
 * 同时负责限高杆；只影响L1，不会连带加速木桥和横坡模式。高速下把机身、步高和
 * 转向幅度略收低，并增加Kd，以减小换腿时的侧摆和落脚反弹。 */
#define SIMPLE_TRANSIT_SWING_RATIO       0.42f
#define SIMPLE_TRANSIT_PERIOD_MIN_S      0.11f
#define SIMPLE_TRANSIT_PERIOD_MAX_S      0.18f
#define SIMPLE_TRANSIT_Z_DOWN_MM         145.0f
#define SIMPLE_TRANSIT_STEP_MAX_MM       54.0f
#define SIMPLE_TRANSIT_TURN_STEP_MAX_MM  32.0f
#define SIMPLE_TRANSIT_STEP_HEIGHT_MM    24.0f
#define SIMPLE_TRANSIT_FORWARD_YAW_TRIM   0.10f     // L1满速前进自动左修正；正=左、负=右
#define SIMPLE_TRANSIT_REVERSE_YAW_TRIM   0.00f     // L1满速后退默认不修正，先保证拉后杆直退
#define SIMPLE_TRANSIT_COMBINED_TURN_SCALE 0.60f   // 只限转向差速；54mm前后步幅和0.11s周期仍保持满速
#define SIMPLE_TRANSIT_KP                40.0f
#define SIMPLE_TRANSIT_KD                2.0f

/* 砂砾碎木坑已放弃且没有遥控按键；参数只保留为历史参考。 */
#define SIMPLE_PIT_Z_DOWN_MM             155.0f
#define SIMPLE_PIT_STEP_MAX_MM           38.0f
#define SIMPLE_PIT_TURN_STEP_MAX_MM      24.0f
#define SIMPLE_PIT_STEP_HEIGHT_MM        34.0f
#define SIMPLE_PIT_KP                    40.0f
#define SIMPLE_PIT_KD                    1.7f

/* 旧限高专用参数已无遥控入口；当前限高杆统一使用L1。 */
#define SIMPLE_LIMIT_BAR_Z_DOWN_MM       135.0f
#define SIMPLE_LIMIT_BAR_STEP_MAX_MM     30.0f
#define SIMPLE_LIMIT_BAR_TURN_STEP_MAX_MM 18.0f
#define SIMPLE_LIMIT_BAR_STEP_HEIGHT_MM  16.0f
#define SIMPLE_LIMIT_BAR_KP              36.0f
#define SIMPLE_LIMIT_BAR_KD              1.8f

/* R2长斜坡：固定左右腿差高，不依赖IMU闭环；右腿在坡上侧并缩短，
 * 左腿在坡下侧并伸长。使用快速对角步态，右摇杆可直接边走边转向修正。
 * FIXED_COMP是单侧补偿，12mm表示左右脚目标总高度差24mm。 */
#define SIMPLE_CROSS_SLOPE_Z_DOWN_MM            160.0f
#define SIMPLE_CROSS_SLOPE_FIXED_COMP_MM         12.0f
#define SIMPLE_CROSS_SLOPE_COMP_MAX_MM           16.0f
#define SIMPLE_CROSS_SLOPE_COMP_RATE_MM_S        40.0f
#define SIMPLE_CROSS_SLOPE_STEP_MAX_MM           40.0f
#define SIMPLE_CROSS_SLOPE_TURN_STEP_MAX_MM      26.0f
#define SIMPLE_CROSS_SLOPE_STEP_HEIGHT_MM        48.0f // 接近工作区内边界的最大稳妥步高
#define SIMPLE_CROSS_SLOPE_KP                    46.0f
#define SIMPLE_CROSS_SLOPE_KD                     2.2f

/* L2木桥A：独立的加速四拍。每次只抬一腿，另外三腿支撑；较低机身、
 * 较小转向和较高阻尼抑制窄桥侧摆。36/0.34约106mm/s，比旧版快约40%。 */
#define SIMPLE_BRIDGE_A_SWING_RATIO      0.22f
#define SIMPLE_BRIDGE_A_PERIOD_MIN_S     0.34f
#define SIMPLE_BRIDGE_A_PERIOD_MAX_S     0.48f
#define SIMPLE_BRIDGE_A_Z_DOWN_MM        150.0f
#define SIMPLE_BRIDGE_A_STEP_MAX_MM      36.0f
#define SIMPLE_BRIDGE_A_TURN_STEP_MAX_MM 8.0f
#define SIMPLE_BRIDGE_A_STEP_HEIGHT_MM   26.0f
#define SIMPLE_BRIDGE_A_YAW_TRIM         0.08f      // L2正值左转，抵消直行向右偏
#define SIMPLE_BRIDGE_A_KP               40.0f
#define SIMPLE_BRIDGE_A_KD               2.6f

/* 比赛版木桥B参数，演示版不从遥控器进入该模式。 */
#define SIMPLE_BRIDGE_B_SWING_RATIO      0.22f
#define SIMPLE_BRIDGE_B_PERIOD_MIN_S     0.36f
#define SIMPLE_BRIDGE_B_PERIOD_MAX_S     0.52f
#define SIMPLE_BRIDGE_B_Z_DOWN_MM        150.0f
#define SIMPLE_BRIDGE_B_STEP_MAX_MM      32.0f
#define SIMPLE_BRIDGE_B_TURN_STEP_MAX_MM 8.0f
#define SIMPLE_BRIDGE_B_STEP_HEIGHT_MM   34.0f
#define SIMPLE_BRIDGE_B_YAW_TRIM         0.08f      // R1独立直行纠偏，后续可单独微调
#define SIMPLE_BRIDGE_B_KP               40.0f
#define SIMPLE_BRIDGE_B_KD               2.6f

/*=========================== 7. 跳跃参数 ================================*/
/* 演示版R1直接执行四腿同步向前小跳；三角键执行下方的单腿律动。
 * 第一轮必须架空或用吊绳减载测试，确认四腿同步和方向后才能落地跳。
 * Z_DOWN越大表示腿越长：145mm预压，255mm快速蹬伸，随后收腿并准备落地。
 * 参考开源项目只采用“先曲腿到位、再蹬伸、空中低刚度、落地高阻尼”的阶段思路；
 * 其150/300mm连杆和约100度关节动作不能直接套到本机110/200mm五连杆。 */
#define SIMPLE_JUMP_ENABLE               1U
#define SIMPLE_JUMP_COMPRESS_Z_MM        145.0f      // 蓄力腿长：小=蹲得更深
#define SIMPLE_JUMP_EXTEND_Z_MM          255.0f      // 蹬伸腿长：大=蹬得更猛，暂勿超过270
#define SIMPLE_JUMP_TUCK_Z_MM            145.0f      // 腾空收腿目标
#define SIMPLE_JUMP_LANDING_Z_MM         205.0f      // 落地前伸腿目标，接近正常站立
#define SIMPLE_JUMP_ABSORB_Z_MM          170.0f      // 触地后的压缩缓冲目标

/* 普通前跳的备用足端X，当前没有遥控按键触发。+X朝狗头、-X朝狗尾。
 * 蓄力/蹬伸时脚在身体后方，地面对机身产生向前分力；腾空后把脚收向前方接地。
 * 想增加前跳力度时，优先把THRUST_X再减小2~3mm，不要先缩短蹬伸时间。 */
#define SIMPLE_FORWARD_JUMP_COMPRESS_X_MM  -18.0f    // 预压时脚稍向后
#define SIMPLE_FORWARD_JUMP_THRUST_X_MM    -28.0f    // 蹬伸方向，负值绝对值越大前推越强
#define SIMPLE_FORWARD_JUMP_TUCK_X_MM       20.0f    // 腾空收腿时把脚摆向前方
#define SIMPLE_FORWARD_JUMP_LANDING_X_MM    28.0f    // 预计落脚点，防止机身直接向前扑
#define SIMPLE_FORWARD_JUMP_ABSORB_X_MM     12.0f    // 落地缓冲后逐渐回到脚下

/* 木桥B向前小跳参数。+X朝狗头、-X朝狗尾：预压和蹬伸时脚向后推地，
 * 机身因此获得向前速度；腾空后四脚收向+X，并在前方落地。 */
#define SIMPLE_BRIDGE_B_JUMP_COMPRESS_X_MM -60.0f    // 预压脚稍向后，提前建立前跳方向
#define SIMPLE_BRIDGE_B_JUMP_THRUST_X_MM  -100.0f    // 比上一版-88再加约14%，增强向前蹬地分量
#define SIMPLE_BRIDGE_B_JUMP_TUCK_X_MM      72.0f    // 腾空后进一步向前收脚
#define SIMPLE_BRIDGE_B_JUMP_LANDING_X_MM  100.0f    // 比上一版88再前移12mm，准备更远落脚
#define SIMPLE_BRIDGE_B_JUMP_ABSORB_X_MM    36.0f    // 落地缓冲时逐渐把脚收回机身下方
#define SIMPLE_BRIDGE_B_JUMP_COMPRESS_Z_MM 150.0f
#define SIMPLE_BRIDGE_B_JUMP_EXTEND_Z_MM   245.0f
#define SIMPLE_BRIDGE_B_JUMP_TUCK_Z_MM     150.0f
#define SIMPLE_BRIDGE_B_JUMP_LANDING_Z_MM  200.0f
#define SIMPLE_BRIDGE_B_JUMP_ABSORB_Z_MM   172.0f
#define SIMPLE_BRIDGE_B_JUMP_THRUST_S      0.15f

/* 演示跳舞：每次只抬一条腿约20mm，始终保持三条腿支撑。
 * 这不是比赛步态，只用于给新生展示四足机构的协调运动。 */
#define SIMPLE_DANCE_LIFT_Z_MM           180.0f     // 站立腿长200mm，抬脚约20mm
#define SIMPLE_DANCE_LIFT_S               0.32f     // 抬脚/换腿时间，越大越柔和
#define SIMPLE_DANCE_STAGE_COUNT         8U         // 前后四腿往返两轮
#define SIMPLE_DANCE_KP                  26.0f
#define SIMPLE_DANCE_KD                   2.2f

/* 比赛版分腿跨缝参数暂时保留作备用，演示版不会调用。 */
#define SIMPLE_BRIDGE_B_GAP_STEP_X_MM      180.0f    // 跨150mm缝，保留约30mm落脚余量
#define SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM      110.0f    // 相对150mm低姿竖直抬脚40mm
#define SIMPLE_BRIDGE_B_GAP_LIFT_S           0.35f
#define SIMPLE_BRIDGE_B_GAP_SWING_S          0.65f
#define SIMPLE_BRIDGE_B_GAP_LOWER_S          0.45f
#define SIMPLE_BRIDGE_B_GAP_SHIFT_S          0.80f
#define SIMPLE_BRIDGE_B_GAP_RECOVER_S        0.80f
#define SIMPLE_BRIDGE_B_GAP_KP              36.0f
#define SIMPLE_BRIDGE_B_GAP_KD               2.8f

/* 方向上旧跳跃参数仅保留为历史数据；当前方向上使用下方无腾空分腿步态。 */
#define SIMPLE_T_UP_COMPRESS_X_MM       -35.0f
#define SIMPLE_T_UP_THRUST_X_MM         -65.0f
#define SIMPLE_T_UP_TUCK_X_MM            50.0f
#define SIMPLE_T_UP_LANDING_X_MM         80.0f
#define SIMPLE_T_UP_ABSORB_X_MM          30.0f
#define SIMPLE_T_UP_COMPRESS_Z_MM       135.0f
#define SIMPLE_T_UP_EXTEND_Z_MM         270.0f
#define SIMPLE_T_UP_TUCK_Z_MM           135.0f
#define SIMPLE_T_UP_LANDING_Z_MM        205.0f
#define SIMPLE_T_UP_ABSORB_Z_MM         165.0f
#define SIMPLE_T_UP_THRUST_S             0.11f

/* T台每级高100mm、踏面深300mm。单次上一级时，每条腿先近似竖直抬起，
 * 再在高位摆到前方并落到上一级；前后腿之间平移机身，全程至少三脚支撑。 */
#define SIMPLE_T_UP_STEP_X_MM            100.0f     // 单次净前移约100mm，落点仍在300mm踏面内
#define SIMPLE_T_UP_PRELIFT_Z_MM         105.0f     // 原地先抬到接近台沿，避开同轴机构内边界
#define SIMPLE_T_UP_SWING_Z_MM            90.0f     // 越过100mm台沿时再抬10mm
#define SIMPLE_T_UP_SUPPORT_Z_MM         110.0f     // 上一级支撑，保留少量压紧量
#define SIMPLE_T_UP_SHIFT_FRONT_X_MM      40.0f
#define SIMPLE_T_UP_SHIFT_REAR_X_MM      -60.0f
#define SIMPLE_T_UP_LIFT_S                 0.55f
#define SIMPLE_T_UP_SWING_S                0.65f
#define SIMPLE_T_UP_LOWER_S                0.45f
#define SIMPLE_T_UP_SHIFT_S                0.85f
#define SIMPLE_T_UP_RECOVER_S              0.90f
#define SIMPLE_T_UP_STEP_KP                34.0f
#define SIMPLE_T_UP_STEP_KD                 2.8f

/* 方向下：不再跳下。四腿依次做“大步前探 + 向下找地”，再缓慢前移车身。
 * REACH_Z越大向下探得越深，接近300mm会靠近本机构伸直极限，不要继续放大。 */
#define SIMPLE_T_DOWN_STEP_X_MM           45.0f      // 靠近机身探地，给295mm腿长留工作区
#define SIMPLE_T_DOWN_LIFT_Z_MM          165.0f
#define SIMPLE_T_DOWN_REACH_Z_MM         295.0f      // 接近100mm真实落差的最大安全找地深度
#define SIMPLE_T_DOWN_SUPPORT_Z_MM       285.0f      // 找到下一级后的承重腿长
#define SIMPLE_T_DOWN_SHIFT_FRONT_X_MM    10.0f
#define SIMPLE_T_DOWN_SHIFT_REAR_X_MM    -55.0f
#define SIMPLE_T_DOWN_LIFT_S               0.40f
#define SIMPLE_T_DOWN_REACH_S              0.75f
#define SIMPLE_T_DOWN_SHIFT_S              0.85f
#define SIMPLE_T_DOWN_RECOVER_S            1.00f
/* 后腿不再从-55直接扫到+75：先原地小抬，再收敛地摆到+45，最后找地。 */
#define SIMPLE_T_DOWN_REAR_STEP_X_MM       45.0f
#define SIMPLE_T_DOWN_REAR_LIFT_Z_MM      185.0f      // 相对200mm站姿只抬15mm
#define SIMPLE_T_DOWN_REAR_LIFT_S           0.55f
#define SIMPLE_T_DOWN_REAR_SWING_S          0.65f
#define SIMPLE_T_DOWN_KP                   36.0f
#define SIMPLE_T_DOWN_KD                    3.0f

/* 高墙已放弃且没有遥控按键；参数只保留为历史参考。 */
#define SIMPLE_WALL_JUMP_COMPRESS_X_MM  -35.0f
#define SIMPLE_WALL_JUMP_THRUST_X_MM    -55.0f
#define SIMPLE_WALL_JUMP_TUCK_X_MM       55.0f
#define SIMPLE_WALL_JUMP_LANDING_X_MM    70.0f
#define SIMPLE_WALL_JUMP_ABSORB_X_MM     25.0f
#define SIMPLE_WALL_JUMP_COMPRESS_Z_MM  130.0f
#define SIMPLE_WALL_JUMP_EXTEND_Z_MM    280.0f
#define SIMPLE_WALL_JUMP_TUCK_Z_MM      125.0f
#define SIMPLE_WALL_JUMP_LANDING_Z_MM   220.0f
#define SIMPLE_WALL_JUMP_ABSORB_Z_MM    165.0f
#define SIMPLE_WALL_JUMP_THRUST_S        0.10f

#define SIMPLE_JUMP_COMPRESS_S           0.45f       // 站立到预压，先慢一些保证四腿同步
#define SIMPLE_JUMP_READY_HOLD_S         0.12f       // 预压到位后的最短稳定时间
#define SIMPLE_JUMP_READY_TIMEOUT_S      0.60f       // 超时仍不同步则取消，不进入蹬伸
#define SIMPLE_JUMP_THRUST_S             0.14f       // 蹬伸时间：减小会更猛，也更容易失稳
#define SIMPLE_JUMP_TUCK_S               0.14f
#define SIMPLE_JUMP_LANDING_S            0.18f
#define SIMPLE_JUMP_ABSORB_S             0.26f       // 落地后缓慢压腿，时间越长冲击越柔和
#define SIMPLE_JUMP_ABSORB_HOLD_S        0.12f       // 最低姿态短暂停留，避免尚未落稳就伸腿
#define SIMPLE_JUMP_RECOVER_S            0.50f       // 缓冲结束后平滑恢复站立
#define SIMPLE_JUMP_READY_TOL_RAD        0.15f       // 八电机进入该误差后才允许蹬伸

#define SIMPLE_JUMP_COMPRESS_KP          30.0f
#define SIMPLE_JUMP_COMPRESS_KD          2.0f
#define SIMPLE_JUMP_THRUST_KP            40.0f
#define SIMPLE_JUMP_THRUST_KD            0.8f        // 蹬伸低阻尼，避免动作发闷
#define SIMPLE_JUMP_FLIGHT_KP            18.0f
#define SIMPLE_JUMP_FLIGHT_KD            0.8f
#define SIMPLE_JUMP_LANDING_KP           22.0f
#define SIMPLE_JUMP_LANDING_KD           3.0f        // 落地高阻尼，抑制第一下反弹
#define SIMPLE_JUMP_ABSORB_KP            16.0f       // 缓冲阶段稍软，允许腿吸收冲击
#define SIMPLE_JUMP_ABSORB_KD            3.0f        // 高阻尼抑制缓冲后的二次弹跳

/*============================ 8. 过渡和电机增益 ==========================*/
#define SIMPLE_CONTROL_DT_S              0.001f     // 1 ms 控制周期，不要改
#define SIMPLE_GETUP_STAGE1_S            6.00f      // 当前姿态整理到统一趴姿的时间
#define SIMPLE_GETUP_STAGE2_S            3.00f      // 趴姿升到站姿的时间
#define SIMPLE_STAND_TRANSITION_S        0.70f      // 回到站姿的平滑时间
#define SIMPLE_GETDOWN_S                 1.20f      // 站立到趴下的时间：小=更快，大=更柔和
/* 0=从实时反馈平滑起步，不因初始姿态偏差锁存故障。 */
#define SIMPLE_MAX_POSE_DELTA_RAD        0.0f
#define SIMPLE_POSE_GEOMETRY_GUARD       0
#define SIMPLE_MAX_IK_STEP_RAD           0.35f

/* MIT 阻抗增益：Kp 主要决定位置刚度，Kd 主要决定阻尼。
 * Kp 太小会塌腿/跟不上，太大容易抖动和打滑；Kd 太小容易振，太大会动作发闷并阻碍快抬腿。
 * 当前电机协议允许 Kp 0~500、Kd 0~5，但实机速通阶段不要按协议上限调。 */
#define SIMPLE_SAFE_HOLD_KD              1.0f       // 等待状态只保留阻尼
#define SIMPLE_GETUP_KP                  12.0f      // 起立前段保持柔和
#define SIMPLE_GETUP_KD                  1.0f
#define SIMPLE_STAND_KP                  22.0f      // 左侧和公共站立刚度
#define SIMPLE_STAND_KD                  2.0f
/* 仅在STAND静止站立状态加强右侧，抵消右侧承重后下沉。
 * 不影响起立整理、低姿高速行走和跳跃，避免再次加重左后腿卡滞。 */
#define SIMPLE_RIGHT_STAND_KP            34.0f      // 右前+右后站立刚度，微调右侧承重下沉
#define SIMPLE_RIGHT_STAND_KD            2.5f       // 右侧增加阻尼，防止提高Kp后抖动
#define SIMPLE_WALK_KP                   30.0f      // 行走支撑刚度，建议先约 25~35
#define SIMPLE_WALK_KD                   2.0f       // 快速步态建议约 1.5~2.3
#define SIMPLE_CRAWL_KP                  40.0f      // 拉到当前软件上限，保证高频位置跟随
#define SIMPLE_CRAWL_KD                  1.5f       // 降低阻尼，避免妨碍快速换腿
#define SIMPLE_DOWN_KP                   12.0f
#define SIMPLE_DOWN_KD                   1.5f

/* 四腿重新标零后编码器包含多圈偏移，必须使用电机协议的完整位置范围。
 * 实际五连杆工作空间仍由足端逆解限定，不再用旧的±210度编码器绝对值拦截。 */
#define SIMPLE_POSITION_MIN_RAD          -12.57f
#define SIMPLE_POSITION_MAX_RAD           12.57f
#define SIMPLE_KP_MAX                    40.0f
#define SIMPLE_KD_MAX                    3.0f

/*============================== 9. 动作总开关 =============================*/
/*
 * 完成杆长、映射、方向和 8 个零位标定前必须保持 0。
 * 改成 1 代表你已经确认实物，并授权 START/SELECT 产生位置动作。
 */
#define SIMPLE_ROBOT_CALIBRATED          1U

/*============================= 10. 手柄参数 ==============================*/
#define SIMPLE_RC_DEADBAND               80         // 摇杆死区：大=不易误动，小=更灵敏
#define SIMPLE_RC_FULL_SCALE             660.0f     // 当前 PS2 满量程，通常不要改
/* 起立承重已经通过：解锁左摇杆前后运动和右摇杆原地转向。 */
#define SIMPLE_REMOTE_LOCOMOTION_ENABLE  1
/*============================= 11. BMI088 ================================*/
/* IMU初始化和读取只在main主循环执行，不进入电机/CAN定时器中断。 */
#define SIMPLE_IMU_BALANCE_ENABLE        1U
/* 手动抬高狗的右侧时，roll_relative_watch应为正；若为负改成-1.0。 */
#define SIMPLE_IMU_ROLL_SIGN             1.0f
#define SIMPLE_IMU_ROLL_DEADBAND_RAD     0.0174533f // 1度内视为平地抖动
#define SIMPLE_IMU_ROLL_MAX_RAD          0.349066f  // 最多按20度计算
#define SIMPLE_IMU_GAIN_MM_PER_RAD       45.0f      // 10度约增加7.9mm

#endif
