# BMA422 可移植倾角计算库（只算角度）

一个**纯 C、无 RTOS / 无 UI / 无平台依赖**的倾角计算库：从 BMA422 原始加速度
一路算到「离水平总倾斜角」。**只做角度计算**。

---

## 1. 目录结构

| 文件 | 职责 |
| --- | --- |
| `bma422_cfg.h` / `.c` | **传感器层**：寄存器定义、I2C 回调注入、初始化、数据读取、轴重映射、软件校零 |
| `tilt_calc.h` / `.c` | **算法层**：定点无死区 IIR 低通 + 总倾斜角 `atan2(√(Y²+Z²), \|X\|)` 计算 |
| `README.md` | 本说明文档 |

代码按层分离，**算法层完全不依赖传感器层**：`tilt_calc` 的入口是
`TiltCalc_Process(x, y, z)` —— 任意能输出"逻辑轴 LSB"的加速度计都能喂它。

---

## 2. 模块数据流

```
                 ┌──────────────────────┐        ┌──────────────────────┐
  物理轴原始数据 │    bma422_cfg.c       │ 逻辑轴 │     tilt_calc.c       │
 ──────────────▶ │ 12bit 符号扩展        │──LSB──▶│ 无死区 IIR 低通        │
  (I2C 回调注入)  │ 物理轴→逻辑轴重映射   │ 每样本  │ → 总倾斜角 atan2(L,|X|)│
                 │ 软件零偏扣除          │        │                      │
                 └──────────────────────┘        └──────────┬───────────┘
                                                             ▼
                                        TiltCalc_Result(角度 / 原始角度 /
                                                        水平分量 / |X| / 滤波三轴)
```

分层关键点：**轴约定**。本库约定逻辑轴为「水平放置时 X ≈ -1g、Y ≈ Z ≈ 0」，
`bma422_cfg` 负责把传感器的实际安装角度映射成这个约定，`tilt_calc` 只认约定。

---

## 3. 角度算法

```
水平分量 L = sqrt(Y² + Z²)        （Y、Z 是逻辑轴读数）
敏感轴分量 V = |X|
倾斜角    θ = atan2(L, V)         （0 = 水平，90° = 垂直）
```

- **方向无关**：倾斜角只看 Y/Z 的合幅值，不随"往哪个方向倒"变化。实测同一
  5° 倾斜在 +Y/−Y/+Z/−Z 四个方位读数完全相同（498 = 4.98°）。
- **水平附近线性好**：0°~10° 区间分辨率不打折。
- **为什么不用 `atan2(|X|, L)`（X 俯仰角）**：它在水平附近输出趋近 90° 并饱和，
  5° 附近分辨率坍缩；`atan2(L, |X|)` 在 0° 附近才是线性的。
- **滤波无静差**：定点无死区 IIR，不存在 `filter += (raw-filter)>>3` 那种
  1~7 LSB 追不到目标的截断死区，稳态读数精确落在真值上。

---

## 4. 快速移植

### 4.1 第一步：提供 I2C 回调

```c
typedef struct {
    uint8_t (*write_reg)(uint8_t dev_addr, uint8_t reg, uint8_t val);
    uint8_t (*read_reg)(uint8_t dev_addr, uint8_t reg);
    void    (*read_multi)(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);
    void    (*delay_us)(uint32_t us);
} BMA422_I2cOps;
```

- `dev_addr` 是 **8-bit 写地址**（7-bit 地址左移 1 位，0x18 → 0x30）；
- `write_reg` / `read_reg` **必须实现**；
- `read_multi` 可传 `NULL`，库内自动退化为单字节循环读（略慢但正确）；
- `delay_us` 可传 `NULL`（会失去上电/切换后的时序等待，不推荐）。

### 4.2 第二步：接线与轴映射

1. 接线：SDA/SCL 两根线（本库不关心引脚，由你的 I2C 实现决定），
   SDO 接 GND = 地址 0x18（库自动探测，0x18/0x19 都可）；
2. 轴重映射：在 `bma422_cfg.h` 修改 `BMA422_AXIS_REMAP`，让逻辑轴满足
   「水平时 X ≈ -1g、Y ≈ Z ≈ 0」：

   - 传感器平放、丝印方向与 PCB 一致：默认 `{1,2,3}` 直通即可；
   - 90° 旋转安装（huada 实物）：`{2,1,3}`（逻辑 X = 物理 Y 等，见头文件注释）；
   - 某轴装反：对应元素加负号，如 `{2,-1,3}`。

### 4.3 第三步：加入工程

把 4 个 .c/.h 拷进工程编译。`tilt_calc.c` 用到 `math.h` 的
`sqrtf/atan2f`，需要浮点库（`-lm`，MCU 侧有 FPU 或软浮点即可）。

### 4.4 第四步：调用

```c
#include "bma422_cfg.h"
#include "tilt_calc.h"

static const BMA422_I2cOps my_i2c = { my_write_reg, my_read_reg,
                                      my_read_multi, my_delay_us };

int main(void)
{
    MyI2cInit();                       /* 你自己的 I2C 初始化 */

    if (BMA422_Init(&my_i2c) != 0u) {
        /* 器件没找到：查接线/地址/供电 */
    }
    TiltCalc_Init();

    /* 可选：出厂校零。调用前把设备确实放水平！ */
    /* BMA422_CalibrateLevel();  */

    for (;;) {
        int16_t x, y, z;

        if (BMA422_DataReady()) {      /* 有新样本（~3.1Hz） */
            BMA422_ReadRaw(&x, &y, &z);
            TiltCalc_Process(x, y, z); /* 滤波 + 算角度 */

            /* 想报警就自己比：超过 5.00° 做点什么 */
            if (TiltCalc_GetAngleX100() > 500u) {
                /* ... */
            }
        }
        /* 该睡睡、该喂狗喂狗，库不占用你的调度 */
    }
}
```

### 4.5 用法提示：报警逻辑怎么写

本库不内置报警，典型写法（含滞回 + 去抖）在使用方：

```c
#define ALARM_X100    500u   /* 5.00° 报警 */
#define RELEASE_X100  455u   /* 4.55° 释放（0.45° 滞回）*/
#define CONFIRM       3u     /* 连续 3 个样本一致才切换 */

static uint8_t s_alarm = 0u;
static uint8_t s_cnt = 0u;

void MyAlarmTask_OnNewSample(void)
{
    uint16_t a = TiltCalc_GetAngleX100();

    if (s_alarm == 0u) {
        if (a > ALARM_X100) {
            if (++s_cnt >= CONFIRM) { s_alarm = 1u; s_cnt = 0u; }
        } else {
            s_cnt = 0u;
        }
    } else {
        if (a < RELEASE_X100) {
            if (++s_cnt >= CONFIRM) { s_alarm = 0u; s_cnt = 0u; }
        } else {
            s_cnt = 0u;
        }
    }
}
```

---

## 5. 校准（软件零偏方案）

- 传感器安装后 PCB 与水平面会有零点偏差，直接导致"水平也报角度"；
- 本库**不写硬件 OFFSET 寄存器（0x71~0x73）**（BMA422 该寄存器的权重/符号
  约定与常见器件不同，易踩坑），改存软件 offset 在读取时扣除：
  - `BMA422_SetOffset(ox, oy, oz)`：外部（如你自己的按键+Flash 流程）恢复
    上次保存的校零值；
  - `BMA422_CalibrateLevel()`：阻塞式一键校零，要求设备当前**确实放平**，
    内部采 8 个样本平均后自动算出 offset。**掉电后 offset 丢失**，需要把
    offset 存 Flash，上电后用 `BMA422_SetOffset` 恢复。

---

## 6. API 参考

### 6.1 bma422_cfg（传感器层）

| 函数 | 说明 |
| --- | --- |
| `uint8_t BMA422_Init(const BMA422_I2cOps *ops)` | 探测地址并初始化；返回 0 成功 |
| `uint8_t BMA422_DataReady(void)` | 1 = 有新样本（读 STATUS 会清标志，请尽快读数据） |
| `void BMA422_ReadRaw(int16_t *x,*y,*z)` | 读逻辑轴 LSB，含符号扩展/重映射/扣 offset |
| `void BMA422_SetOffset(int16_t ox,oy,oz)` | 设置软件零偏 |
| `void BMA422_CalibrateLevel(void)` | 水平一键校零（阻塞，采 8 样本） |
| `uint8_t BMA422_IsCalibrated(void)` | 是否已校零 |
| `void BMA422_Standby(void)` | 停止采样（保配置） |
| `void BMA422_Active(void)` | 恢复采样 |
| `void BMA422_Sleep(void)` | 低功耗前调用（同 Standby） |

### 6.2 tilt_calc（算法层）

| 函数 | 说明 |
| --- | --- |
| `void TiltCalc_Init(void)` / `TiltCalc_Reset(void)` | 清零滤波状态与结果缓存 |
| `void TiltCalc_Process(int16_t x,y,z)` | **核心**：喂一帧逻辑轴样本（有新样本才调） |
| `void TiltCalc_GetResult(TiltCalc_Result *out)` | 最近结果快照 |
| `uint16_t TiltCalc_GetAngleX100(void)` | 便捷：滤波后倾斜角 0.01°（0~9000） |
| `uint16_t TiltCalc_ComputeAngleX100(int16_t x,y,z)` | **无状态**一次性计算，不碰内部滤波状态 |

### 6.3 输出结构

```c
typedef struct {
    uint16_t angle_x100;     /* 滤波后的离水平总倾斜角：0=水平, 9000=90°(垂直) */
    uint16_t angle_raw_x100; /* 本帧未滤波的原始倾斜角（对比/调试用） */
    uint16_t horiz_lsb;      /* 水平分量幅值 L = sqrt(Y²+Z²)，LSB */
    uint16_t vert_lsb;       /* 敏感轴分量 |X|，LSB */
    int16_t  filt_x;         /* 滤波后逻辑轴 X（LSB，调试/可视化用） */
    int16_t  filt_y;         /* 滤波后逻辑轴 Y */
    int16_t  filt_z;         /* 滤波后逻辑轴 Z */
} TiltCalc_Result;
```

---

## 7. 关键参数与调参表

### 7.1 算法（tilt_calc.h）

| 宏 | 默认 | 含义 |
| --- | --- | --- |
| `TILTCALC_RAD_TO_X100` | 5729.578 | 弧度 → 0.01° 系数（量产沿用 5729.0，误差 <0.01%） |
| `TILTCALC_MAX_X100` | 9000 | 角度上限 90.00°，超出 clamp |
| `TILTCALC_FILTER_SHIFT` | 3 | IIR α = 1/8；**0 = 关闭滤波直通**，1=α1/2，2=α1/4，3=α1/8 |

α 越小越平滑但跟随越慢。默认 1/8 配合 BMA422 的 ~3.1Hz 有效输出率，
约 8 个样本（~2.6s）收敛到 99%。想要更快响应就调小 `FILTER_SHIFT`。

### 7.2 传感器配置（bma422_cfg.h）

| 宏 | 默认 | 说明 |
| --- | --- | --- |
| `BMA422_ACC_CONF_VALUE` | 0x47 | power-optimized \| AVG16 \| 50Hz → 有效 ~3.1Hz |
| `BMA422_ACC_RANGE_VALUE` | ±2g | 1024 LSB/g |
| `BMA422_AXIS_REMAP` | {1,2,3} | 物理轴 → 逻辑轴映射表 |

