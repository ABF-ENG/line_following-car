# 智能车循迹（STM32F407）

车王争霸赛智能小车项目 —— 基于八路灰度传感器 + 编码器 + 陀螺仪的循迹方案。

[**▶ 演示视频**](docs/demo.mp4)（点开可下载观看）

---

## 硬件

| 部件 | 型号 / 参数 | 备注 |
|---|---|---|
| 主控 | STM32F407VET6，168 MHz | HSE 8 MHz，PLL 168/84/42 |
| 电机驱动 | TB6612 双路 | VM=12V，VCC=3.3V |
| 电机 | MG513P28，GMR 编码器 500 线 | 减速比 28，4 倍频 |
| 轮子 | 直径 65 mm | 周长 204.2 mm |
| 灰度传感器 | 八路，CD4051 复用，数字输出 | 最佳离地高度 18 mm |
| 陀螺仪 | MPU6050（I2C1） | 只取 Z 轴 |
| 电池 | 12V | 经 12V→5V 模块给逻辑供电 |

### 引脚分配

| 功能 | 引脚 | 说明 |
|---|---|---|
| 左电机 PWM / 方向 | PE9 (TIM1_CH1) / PE2、PE3 | |
| 右电机 PWM / 方向 | PE11 (TIM1_CH2) / PE4、PE5 | **右路方向脚与左路相反**（接线镜像） |
| 左编码器 A/B | PA0 / PB3 | TIM2 编码器模式，32 位 |
| 右编码器 A/B | PA6 / PA7 | TIM3 编码器模式，16 位 |
| 灰度地址 AD0/AD1/AD2 | PD0 / PD1 / PD2 | 推挽输出 |
| 灰度输出 OUT | PD3 | 数字输入，**压黑线 = 1** |
| MPU6050 SCL/SDA | PB6 / PB7 | I2C1，400 kHz |
| 急停按键 | PC0 | 上拉，按下 = 低电平 |
| 调试串口 | PA2 / PA3 | USART2，115200 8N1 |

---

## 软件结构

```
Broad/
├── Gray.c/h          八路灰度驱动（CD4051 轮询，DWT 微秒延时）
├── Motor_base.c/h    电机底层（PWM/方向/编码器）+ PC0 急停
├── Motor_advance.c/h 双轮速度环（前馈 + PI，含堵转保护）
├── Trace.c/h         循迹决策层（PD、丢线、盲走、环岛、终点）
├── Mpu6050.c/h       陀螺仪驱动（±500°/s，DLPF 20Hz，零偏校准）
└── PID.c/h           通用 PID 结构（备用）
```

**数据流**

```
Timer6 (20ms 中断)
   └─→ 编码器 Δ计数 → 累计里程 / 瞬时速度
          │
          ├─→ Trace_Tick()        读灰度 + 陀螺仪 → 算出左右轮目标速度
          │        └─→ MotorAdvance_SetTarget()
          │
          └─→ MotorAdvance_Tick() 速度环 PI → PWM 占空比
                   └─→ Motor_SetSpeed()
```

**分层原则**：`Trace` 只决定「该跑多快」，`Motor_advance` 只负责「把速度做到」，`Motor_base` 只管「怎么输出 PWM」。改循迹逻辑不会碰到电机代码。

---

## 核心算法

### 1. 循迹误差

八路探头加权重心，X1 最左 = −7，X8 最右 = +7：

```c
error = Σ(weight[i] × mask[i]) / Σ mask[i]     // −7 … +7
steer = KP × error + KD × Δerror
左轮 = BASE + steer      右轮 = BASE − steer
```

直线稳定速度 **400 mm/s**（`BASE_MM_S`）。

### 2. 丢线分两种处理

| 场景 | 判据 | 动作 |
|---|---|---|
| **断口（盲走）** | 丢线瞬间 `\|误差\| < 3`（线在正中消失） | 陀螺仪保持航向直行 |
| **直角弯** | 丢线瞬间 `\|误差\| ≥ 3`（线跑到最外侧） | 朝最后看到的方向打舵找线 |

### 3. 盲走（断线通过）

赛道有一段完全没有线，车必须直线冲过去。

```
进盲走：锁定目标航向 = 发车航向 + 180°（赛道绝对方向，而非车头方向）
每 20ms：yaw_err = 目标 − 当前
         steer = −KP_YAW × yaw_err        // 纯 P，逆时针为 yaw 正方向
出盲走：连续 2 拍重新看到线
```

不用「丢线瞬间的车头方向」当参考，是因为车进断口时可能正带着修正角，会把歪角锁成目标。

### 4. 环岛

环岛入口是 Y 型岔口，**不会产生全黑**，所以不能用「全黑」判据。

```
盲走出口 → 再走 60cm → 开始监视 X8（最右侧探头）上升沿
第 1 次亮 → 忽略
第 2 次亮 → 陀螺仪闭环强制转 30°（进环）
转完交回 PD，靠 PD 沿环跟线
```

### 5. 终点

起终点是同一条粗黑线，规则要求压线后停在停车区。

```
统计「8 路全黑」的上升沿次数
第 4 次压线 → 记录里程 → 直行 200mm → 停车
```

用**次数 + 里程**而不是时间，换速度不用重调。

### 6. 急停（安全）

PC0 按下 → 短路刹车（IN1=IN2=1）+ 锁存，松手不恢复，需显式 `Motor_ClearEStop()`。

检查放在两处：
- `Motor_EncoderUpdate()`（TIM6 中断里）—— 主循环卡死也能停
- `Motor_SetSpeed()` 入口 —— 防止主循环把输出写回去

---

## 关键参数

全部集中在 `Broad/Trace.c` 顶部：

```c
/* 循迹 */
#define BASE_MM_S          400.0f   // 直线速度
#define KP_TRACE            30.0f   // 比例增益
#define KD_TRACE            15.0f   // 微分增益（未除以周期，等效阻尼 = KD × 周期）
#define SPEED_MIN_MM_S      60.0f   // 内侧轮下限，防停死
#define SPEED_MAX_MM_S     900.0f
#define SEARCH_STEER_MM_S  340.0f   // 丢线/直角时的差速

/* 盲走 */
#define BLIND_SPEED_MM_S   500.0f
#define KP_YAW               3.0f   // 每 1° 航向误差给多少 mm/s 差速
#define BLIND_ENTER_E        3.0f   // 判定断口/直角的分界

/* 环岛 / 终点 */
#define ISLAND_WATCH_MM    600.0f
#define ISLAND_EDGE_N        2u
#define ISLAND_TURN_DEG     30.0f
#define FINISH_CROSS_N       4u
#define FINISH_RUN_MM      200.0f   // 压线后再走 200mm 停
```

> **调参注意**：`BASE` 改了，`KP` 要按同比例改；控制周期改了（20ms ↔ 10ms），`KD` 也要跟着变（等效阻尼 ∝ KD × 周期）。

---

## 编译与下载

1. 用 **Keil MDK 5** 打开 `MDK-ARM/f4.uvprojx`
2. 编译下载
3. 上电后：PWM 初始化 → 灰度初始化 → 速度环初始化 → **陀螺仪零偏校准（1 秒，车必须静止）** → 开始循迹

> 零偏校准期间车会静止约 1 秒，属正常现象。正式比赛时应把 `Mpu_Calibrate()` 挪到发车按键触发处。

外设配置在 `f4.ioc`（STM32CubeMX 6.18），可重新生成代码，用户代码都在 `USER CODE` 区内，不会被覆盖。

---

## 调试手段

Keil 的 Watch 窗口**不能调用函数**（会显示 `<cannot evaluate>`），所以关键状态都同步到了全局变量：

| 变量 | 含义 |
|---|---|
| `g_trace_mask` | 当前八路灰度（bit0 = X1） |
| `g_trace_err` | 当前误差 −7…+7 |
| `g_trace_steer` | 当前转向量 mm/s |
| `g_trace_tgtl` / `g_trace_tgtr` | 左右轮目标速度 |
| `g_trace_blind` | 1 = 正在盲走 |
| `g_trace_ye` | 盲走航向误差（度） |
| `g_trace_ab` / `g_trace_edge` / `g_trace_ft` | 环岛监视 / 边灯计数 / 强制转向 |
| `g_trace_cross` / `g_trace_fin` | 压线次数 / 终点冲刺 |
| `g_mpu_ok` / `g_mpu_who` | 陀螺仪是否就绪 / 芯片 ID |
| `g_mpu_yaw` / `g_mpu_rate` / `g_mpu_bias` | 航向 / 角速度 / 零偏 |

`docs/01-原理图复核与测试方案.md` 里有完整的板子测试步骤和固件自检程序说明。

---

## 已知限制

| 限制 | 说明 |
|---|---|
| 20ms 采样 | 400 mm/s 时每拍走 8mm，再提速建议把 TIM6 改成 10ms（PSC 167→83，`CTRL_PERIOD_MS` 同步改） |
| 16 位右编码器 | TIM3 每 239mm 溢出一圈，必须靠 int16 差分累加 |
| 陀螺仪无磁力计 | yaw 无法自校正，只能靠静止零偏校准 + 短时积分 |
| 弯道全速 | 尚未实现入弯减速，直角弯依赖「丢线后打舵」兜底 |

---

## 目录

```
├── Broad/          应用层模块
├── Core/           CubeMX 生成的 HAL 初始化 + main
├── Drivers/        STM32 HAL 库（.gitignore 忽略，CubeMX 可重建）
├── MDK-ARM/        Keil 工程
├── docs/           设计文档、板级测试程序、演示视频
└── f4.ioc          CubeMX 工程配置
```
