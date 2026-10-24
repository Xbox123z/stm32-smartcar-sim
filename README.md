# 🏎️ STM32 智能车仿真平台

> 不用焊板子、不用等场地，写完代码立刻知道小车能不能跑起来。

面向全国大学生智能车竞赛 / 各类循迹小车比赛的选手：把你的 **STM32 C 代码**原样丢进来，
平台会把它放进一个**带真实物理引擎的虚拟世界**里跑一遍，告诉你
**能不能跑完、跑了多久、哪里出界、代码有什么坑**，并给出评分和改进建议。

```
你的 main.c  ──►  C 子集解释器（带 HAL 仿真）  ──►  差速运动学 + 电机滞后物理世界
                          ▲                                    │
                          └────── tick yield：HAL_Delay ◄──────┘
                                  （延时即推进物理）
                                          │
                                          ▼
                              评分 + 遥测 CSV + JSON 报告
```

---

## 快速开始

```bash
# 1. 静态检查（秒出结果，不跑仿真）
car-sim check main.c

# 2. 跑仿真
car-sim run main.c --track track01 --out report.json

# 3. 在三套赛道上对比
car-sim bench main.c

# 4. 看内置示例的效果
car-sim demo
```

退出码：`0` 跑完 · `1` 有必须修的错误 · `2` 程序崩溃 · `3` 超时 · `4` 中途失败。
CI 里可以直接拿退出码卡门禁。

---

## 它能做什么

### 1. 静态检查（11 条规则）

在跑仿真之前先挑刺，每条规则有稳定 ID：

| ID | 级别 | 说明 |
|---|---|---|
| `CARSIM001` | ❌ | 没有 `main` 函数 |
| `CARSIM002` | ⚠️ | `main` 里没有主循环，车只会动一下 |
| `CARSIM003` | ❌ | 主循环里没有 `HAL_Delay`，物理无法推进（忙等） |
| `CARSIM004` | ❌ | 从来没有给电机输出赋值，车不会动 |
| `CARSIM005` | ⚠️ | 没读巡线传感器，等于闭眼开车 |
| `CARSIM006` | ⚠️ | PID 积分项没有限幅，会积分饱和 / 剧烈震荡 |
| `CARSIM007` | ⚠️ | PWM 值超出常见范围 |
| `CARSIM008` | ⚠️ | `printf` 用 `%d` 打印 float |
| `CARSIM009` | ⚠️ | 可能的整数除零 |
| `CARSIM010` | 💡 | `main` 返回类型不是 `int`/`void` |
| `CARSIM011` | 💡 | F103 无 FPU，大量 float 会变慢 |

### 2. 真实物理仿真

- **差速运动学** + 电机一阶滞后（PWM → 轮速不是瞬间到位）
- 加速度限制、空气阻力、偏离赛道的抓地力损失
- 5 路红外巡线阵列（引导线宽 3cm，与真实比赛一致）
- 超声波（HC-SR04）锥形探测、编码器计数、带噪声的陀螺仪
- 圆形 / 矩形障碍物碰撞，出界计数

### 3. 评分

| 维度 | 分值 | 依据 |
|---|---|---|
| 完成度 | 40 | 赛道推进百分比 |
| 速度 | 20 | 与赛道目标单圈时间比 |
| 稳定性 | 20 | 出界次数、碰撞次数扣分 |
| 代码风格 | 20 | 静态检查扣分 |

评级 **S / A / B / C / D**，附中文改进建议。

---

## 怎么让代码跑起来

平台支持两种写法，**你的真实固件代码基本可以原样跑**。

### 方式 A：直接写 HAL（推荐，最贴近真实工程）

```c
#include "stm32f1xx_hal.h"

int main(void) {
    HAL_Init();
    while (1) {
        float pos = Read_Line();              // 加权巡线位置 -2000~2000
        float turn = Kp * (pos / 2000.0);
        __HAL_TIM_SET_COMPARE(&htim2, "TIM2_CH1", 500 + (int)turn);
        __HAL_TIM_SET_COMPARE(&htim2, "TIM2_CH2", 500 - (int)turn);
        HAL_Delay(10);                        // ← 关键：延时即推进物理
    }
}
```

已实现的 HAL / 常用函数：

| 类别 | 函数 |
|---|---|
| 系统 | `HAL_Init` `HAL_GetTick` `HAL_Delay` `SystemInit` |
| 电机 | `__HAL_TIM_SET_COMPARE` `TIM_SetCompare` `HAL_TIM_PWM_Start` |
| 传感器 | `HAL_ADC_GetValue` `HAL_ADC_Start` `HAL_ADC_PollForConversion` |
| GPIO | `HAL_GPIO_ReadPin` `HAL_GPIO_WritePin` `HAL_GPIO_TogglePin` |
| 串口 | `HAL_UART_Transmit` `printf` |

> **不认识的函数不会让仿真崩溃**：会返回 0 并在报告里记一条提示。
> 所以你代码里那些 `HAL_RCC_OscConfig`、`MX_GPIO_Init` 之类直接留着就行。

### 方式 B：用我们提供的简洁 API

```c
Motor_Set(left, right);          // 直接设左右轮 PWM
Sensor_Read(i);                  // 读第 i 路红外 (0/1)
Read_Line();                     // 读加权位置 -2000~2000
Ultrasonic_Read();               // 超声波，单位 cm
Encoder_Read(0);                 // 编码器计数
IMU_Read();                      // 陀螺仪 度/秒
Delay_Ms(10);                    // 延时
Printf("v=%d\n", v);             // 输出（%d %f %s）
```

### ⚠️ 最关键的一条：`HAL_Delay` 必须放在主循环里

解释器执行到 `HAL_Delay(10)` 时会**把控制权交还给仿真器**，
仿真器推进 10ms 物理，然后回到断点继续执行 —— 这就是 **tick yield**。

```c
while (1) {
    // 控制逻辑
    HAL_Delay(10);   // ← 没有这一句，物理永远不推进，会判忙等
}
```

**不支持在函数内部调用延时**，会明确报错提示你挪到主循环。

---

## 支持的 C 子集

- 类型：`int` `float` `char` `void` + `uint8_t` `uint16_t` `uint32_t` `int16_t` `int32_t` 等
- 声明：全局 / 局部、一维数组、`{...}` 初始化列表、`volatile`（解析并忽略）
- 运算符：`+ - * / %`、比较、`&& \|\|`、`= += -= *= /=`、三目、强制转换、`sizeof`
- 语句：`if/else` `while` `for` `do-while` `break` `continue` `return` `switch/case`
- 函数：带参数、可递归（深度 64 上限）
- 预处理：`#include`（忽略）`#define`（对象宏）`#ifdef/#ifndef/#else/#endif`
- 保护：步数预算（防死循环）、除零检测、数组越界检测

暂不支持：结构体成员访问会被**跳过而不报错**，指针算术、`union`、位域。

---

## 赛道格式

```jsonc
{
  "name": "track01",
  "width_m": 0.55,          // 赛道宽度（出界判据）
  "line_width_m": 0.03,     // 引导线宽度（传感器判据），默认 0.03
  "centerline": [ {"x":0,"y":0}, {"x":1,"y":0} ],
  "obstacles": [
    {"type":"circle","x":2.0,"y":0.3,"r":0.15},
    {"type":"box","x":3.0,"y":0.0,"w":0.3,"h":0.2}
  ],
  "start": {"index": 0},
  "target_lap_time_s": 25.0
}
```

内置三条：`track01` 椭圆入门 · `track02` S 弯 · `track03` 直角弯 + 障碍。
也可以直接传自己的 JSON 文件路径给 `--track`。

---

## 报告结构（JSON）

```jsonc
{
  "run_id": "...", "submitted_at": "RFC3339",
  "code": { "path": "main.c", "bytes": 1234, "sha256": "..." },
  "static": {
    "issues": [ {"severity":"warn","line":12,"rule":"CARSIM006","message":"..."} ],
    "detected": { "mcu":"STM32F103C8T6","motor_bindings":[...],"has_main_loop":true,"uses_delay":true }
  },
  "sim": {
    "track":"track01","dt":0.01,"steps":6000,"time_limit_s":60.0,
    "result":"finished","failure_reason":"",
    "lap_time_s":29.7,"distance_m":23.57,"progress":1.0,
    "max_speed_mps":1.2,"avg_speed_mps":0.9,
    "off_track_events":0,"collisions":0,"track_target_s":25.0,
    "telemetry":[ {"t":0,"x":0,"y":0,"yaw":0,"v":0,"wheel_l":0,"wheel_r":0,
                   "pwm_l":0,"pwm_r":0,"sensors":[0,0,1,0,0],"ultrasonic_cm":120} ]
  },
  "score": { "total":88, "breakdown":{"completion":40,"speed":20,"stability":20,"style":20},
             "grade":"A", "comment":"..." },
  "logs": ["..."]
}
```

`result` 取值：`finished` / `timeout` / `failed` / `crashed`。

遥测也能导出 CSV：

```bash
car-sim run main.c --telemetry-csv telemetry.csv
# t,x,y,yaw,v,wheel_l,wheel_r,pwm_l,pwm_r,s0,s1,s2,s3,s4,ultrasonic_cm
```

---

## 确定性

同一份代码 + 同一个 `--seed` → **完全一致的结果**，可以放心做回归测试：

```bash
car-sim run main.c --seed 42 --out a.json
car-sim run main.c --seed 42 --out b.json   # 与 a.json 逐字节相同（除时间戳）
```

---

## 目录结构

```
stm32-smartcar-sim/
├── simcore/                  # 仿真内核
│   ├── src/physics.rs        #   车辆动力学、赛道、传感器
│   ├── src/csim.rs           #   词法 + 递归下降语法分析
│   ├── src/interp.rs         #   树遍历解释器 + HAL 仿真（tick yield）
│   ├── src/lint.rs           #   11 条静态规则
│   ├── src/sim.rs            #   驱动器 + 评分 + 报告
│   ├── tracks/*.json         #   三套内置赛道
│   └── examples/*.c          #   好 / 积分饱和 / 忙等 三个示例
├── cli/                      # car-sim 命令行
├── server/                   # Go Web 服务（上传代码 + 报告展示）
├── web/                      # 前端页面（轨迹可视化 + 详细报告）
└── tools/verify-algo.mjs     # 算法等价性验证脚本
```

## 编译

```bash
cargo build --release
# 产物：target/release/car-sim
```

内核只依赖 `serde` / `serde_json`，CLI 零额外依赖（参数解析手写）。

## 验证

```bash
cargo test                        # 内核单元测试（物理 / 解析器 / 解释器 / lint / 评分）
node tools/verify-algo.mjs        # 算法等价性：13 项断言
```

## License

MIT
