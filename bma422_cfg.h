/**
 * @file    bma422_cfg.h
 * @brief   BMA422 加速度传感器 —— 可移植配置头文件（纯 C，无任何平台依赖）。
 *
 * ============================================================================
 *  模块定位
 * ============================================================================
 *  本文件 + bma422_cfg.c 只负责"传感器本身"的初始化与原始数据读取：
 *    - 通过函数指针注入 I2C 读写接口（本库【不实现】I2C 总线时序，
 *      由使用方提供，可接硬件 I2C、bit-bang I2C、模拟 I2C 任意实现）；
 *    - 寄存器初始化序列（量程 / 带宽 / 输出速率 / 电源模式）；
 *    - 12-bit 补码原始数据读取与符号扩展；
 *    - 轴重映射（把"物理安装轴"换算成"逻辑轴"，见 BMA422_AXIS_REMAP）；
 *    - 软件零点偏移（校零结果，在读取时减去）。
 *
 *  本文件【不含】任何角度计算与滤波 —— 那是 tilt_calc.c 的职责，
 *  两者通过"逻辑轴约定"衔接（见下方轴约定与 bma422_cfg.c 顶部注释）。
 *
 * ============================================================================
 *  数据手册依据
 * ============================================================================
 *  BMA422 datasheet（BST-BMA422-DS000-01, v0.2）。
 *  - 12-bit 两字节补码输出，数据寄存器低字节在前（LSB-first）。
 *  - ±2g 量程灵敏度 1024 LSB/g（与 MMA8452 一致）。
 *  - 寄存器地址 / 位定义均来自手册 Register Map，并已按实测工程核验
 *    （A1035_1036 huada 量产固件，2026-09 实测通过）。
 *
 * ============================================================================
 *  轴约定（务必先读）
 * ============================================================================
 *  本库约定"逻辑轴"为：产品水平放置时，重力应基本落在【逻辑 X 轴负方向】，
 *  即水平静止时读数约为 X = -1g(约 -1024 LSB)、Y ≈ Z ≈ 0。
 *  倾斜角计算全部基于该约定（tilt_calc 取 |X| 作分母，Y/Z 作水平分量）。
 *
 *  传感器在 PCB 上的实际安装角度可能与逻辑轴不一致（如 90° 旋转安装），
 *  此时用 BMA422_AXIS_REMAP 做物理轴 -> 逻辑轴映射，无需改算法。
 */

#ifndef BMA422_CFG_H
#define BMA422_CFG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 1. 器件地址与标识
 * ========================================================================== */

/** 7-bit I2C 地址：SDO 引脚接 GND 时为 0x18，接 VDDIO 时为 0x19。
 *  BMA422_WriteAddr 已左移 1 位得到 8-bit 写地址，供 I2C 发送使用。 */
#define BMA422_I2C_ADDR_7BIT    0x18u
#define BMA422_I2C_ADDR_7BIT_2  0x19u   /* SDO=VDDIO 时的备选地址 */

#define BMA422_CHIP_ID_REG       0x00u
#define BMA422_CHIP_ID_VALUE     0x12u   /* CHIP_ID 固定值，用于上电探测 */

/* ==========================================================================
 * 2. 寄存器地址（Register Map）
 * ========================================================================== */

#define BMA422_REG_STATUS       0x03u   /* bit7 = drdy_acc：新数据就绪标志 */
#define BMA422_REG_DATA_X_LSB   0x12u   /* 6 字节数据寄存器起点：X_LSB X_MSB Y_LSB Y_MSB Z_LSB Z_MSB */
#define BMA422_REG_ACC_CONF     0x40u   /* 输出速率 / 滤波带宽 / 性能模式 */
#define BMA422_REG_ACC_RANGE    0x41u   /* 量程：0x00=±2g 0x01=±4g 0x02=±8g */
#define BMA422_REG_PWR_CONF     0x7Cu   /* 高级省电模式(APS)开关 */
#define BMA422_REG_PWR_CTRL     0x7Du   /* 加速度计使能位 */

/* ==========================================================================
 * 3. ACC_CONF(0x40) 位定义与可选值
 * ==========================================================================
 *   bit[7]   acc_perf_mode : 0=power-optimized（硬件平均 AVG 模式要求此值）
 *   bit[6:4] acc_bwp       : 滤波 / 平均强度
 *   bit[3:0] acc_odr       : 基础输出速率（AVG 模式下实际输出 = odr / 平均倍数）
 *
 *   acc_bwp 取值：
 *     0 = OSR4_AVG1    1 = OSR2_AVG2    2 = NORMAL_AVG4  3 = CIC_AVG8
 *     4 = AVG16        5 = AVG32        6 = AVG64        7 = AVG128
 *
 *   acc_odr 取值（Hz）：0x1=0.78 0x2=1.56 0x3=3.12 0x4=6.25 0x5=12.5
 *                  0x6=25 0x7=50 0x8=100 0x9=200 0xA=400 0xB=800 0xC=1600
 */
#define BMA422_ACC_BWP_AVG16    0x4u    /* 16 次硬件平均 */
#define BMA422_ACC_BWP_AVG32    0x5u
#define BMA422_ACC_ODR_50HZ     0x7u

/** 默认配置 = 0x47：power-optimized | AVG16 | 50Hz。
 *  位分解：bit[7]=0(性能优化/平均模式) bit[6:4]=4(AVG16) bit[3:0]=7(50Hz)。
 *  实际效果：硬件 16 次平均后输出约 50/16 ≈ 3.1 Hz（每样本 ~0.32s），
 *  对桌面/墙面振动有很强的硬件级抑制，配合 tilt_calc 的软件滤波可得到
 *  稳定的角度读数。
 *
 *  ⚠ 实测注意（huada 平台）：此片 BMA422 在 AVG32 @ 100Hz 组合下
 *  【无 data-ready 输出】，请勿把 ODR 与平均倍数配到器件不支持的区域。
 *  若需更强去噪优先加大 acc_bwp；若需省电优先降低 acc_odr。 */
#define BMA422_ACC_CONF_VALUE   0x47u

#define BMA422_ACC_RANGE_2G     0x00u
#define BMA422_ACC_RANGE_4G     0x01u
#define BMA422_ACC_RANGE_8G     0x02u
#define BMA422_ACC_RANGE_VALUE  BMA422_ACC_RANGE_2G   /* 默认 ±2g = 1024 LSB/g */
#define BMA422_LSB_PER_G        1024u   /* ±2g 下每 g 对应的 LSB 数 */

#define BMA422_PWR_CONF_APS_OFF 0x00u   /* 关闭高级省电，保证寄存器写入时序稳定 */
#define BMA422_ACCEL_ENABLE     0x04u   /* PWR_CTRL bit2 = acc_en */

/* ==========================================================================
 * 4. 轴重映射（物理轴 -> 逻辑轴）
 * ==========================================================================
 *  表内三个元素分别对应输出的 X / Y / Z 取自哪一根物理轴：
 *     1 = 物理 X，2 = 物理 Y，3 = 物理 Z；负数表示取反（乘以 -1）。
 *  默认 {1, 2, 3} 为直通，即物理轴 == 逻辑轴。
 *
 *  【huada 平台实例】传感器 90° 旋转安装（重力落在物理 Y 轴），映射为：
 *     逻辑 X = 物理 Y   -> 表第 1 个元素填 2
 *     逻辑 Y = 物理 X   -> 表第 2 个元素填 1
 *     逻辑 Z = 物理 Z   -> 表第 3 个元素填 3
 *     即 #define BMA422_AXIS_REMAP { 2, 1, 3 }
 *  若某轴装反（读数符号与约定相反），在该元素前加负号即可，如 { -2, 1, 3 }。
 */
#define BMA422_AXIS_REMAP       { 1, 2, 3 }   /* 1=X 2=Y 3=Z；负号=取反 */

/* ==========================================================================
 * 5. I2C 底层接口（由使用方实现并注入）
 * ==========================================================================
 *  本库不实现 I2C 时序，只声明如下回调结构。把回调注册给 BMA422_Init 即可。
 *
 *  最少只需实现 write_reg / read_reg 两个回调：
 *    - read_multi 可传 NULL，库内自动退化为按单字节循环读取（慢一点，
 *      但功能一致）；建议硬件 I2C 或具备突发读能力时实现它，一次 6 字节
 *      读完 3 轴数据更高效。
 *    - delay_us 提供微秒级延时，用于上电/模式切换后的时序等待。
 *
 *  注：write_addr / read_addr 均为【8-bit 完整地址】(含 R/W 位的形式)。
 */
typedef struct {
    /** 写单字节寄存器。dev_addr 为 8-bit 写地址；返回 0 成功 / 非 0 失败。 */
    uint8_t (*write_reg)(uint8_t dev_addr, uint8_t reg, uint8_t val);
    /** 读单字节寄存器。dev_addr 为 8-bit 写地址；返回寄存器内容。 */
    uint8_t (*read_reg)(uint8_t dev_addr, uint8_t reg);
    /** 从 reg 起连续读 len 字节到 buf。可传 NULL（库内按单字节循环）。 */
    void    (*read_multi)(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);
    /** 微秒延时。 */
    void    (*delay_us)(uint32_t us);
} BMA422_I2cOps;

/* ==========================================================================
 * 6. 对外 API
 * ========================================================================== */

/**
 * @brief   初始化 BMA422。
 * @param   ops  使用方实现的 I2C 回调（不得为 NULL；内部会拷贝指针保存）。
 * @retval  0  = 成功（CHIP_ID 探测通过）；
 * @retval  非0 = 失败（0x18/0x19 两个地址都探测不到器件）。
 *
 *  流程：探测地址 -> 关 APS -> 写 ACC_CONF(0x47) -> 量程 ±2g -> 使能。
 */
uint8_t BMA422_Init(const BMA422_I2cOps *ops);

/** @brief 查询是否有新数据就绪。返回 1 = 有新样本。注意：读取 STATUS
 *         寄存器会清除 drdy 位，请在返回 1 后尽快调用 BMA422_ReadRaw。 */
uint8_t BMA422_DataReady(void);

/**
 * @brief   读取三轴原始数据（经符号扩展、轴重映射、软件 offset 补偿后）。
 * @param   x / y / z  输出逻辑轴加速度（LSB，±1024 ≈ ±1g）。
 *  数据寄存器为 12-bit 补码、左对齐存放，读回后做算术右移 4 位
 *  得到带符号 16-bit 值。offset 在映射之后减去（见 BMA422_SetOffset）。
 */
void BMA422_ReadRaw(int16_t *x, int16_t *y, int16_t *z);

/** @brief 设置软件零点偏移（逻辑轴 LSB）。在 BMA422_ReadRaw 内部自动减去。
 *         纯软件方案，不写硬件 OFFSET 寄存器(0x71~0x73)。 */
void BMA422_SetOffset(int16_t ox, int16_t oy, int16_t oz);

/** @brief 一键校零（阻塞）：假定设备当前处于"水平"姿态，连续采集
 *         BMA422_CAL_SAMPLES 个样本求平均，把平均值的偏差存为 offset。
 *         逻辑轴水平期望 = X=-1g、Y=Z=0，故 offset_x = avg_x + 1024。
 *         调用前设备必须确实放平；返回后可用 BMA422_IsCalibrated 查询。 */
void BMA422_CalibrateLevel(void);

/** @brief 查询是否已完成过校零。 */
uint8_t BMA422_IsCalibrated(void);

/** @brief 加速度计进入待机（关闭采样，保留寄存器配置）。 */
void BMA422_Standby(void);

/** @brief 加速度计恢复工作（重新使能采样）。 */
void BMA422_Active(void);

/** @brief 进入低功耗前调用（等同 Standby，便于语义区分；低功耗唤醒后需 Active）。 */
void BMA422_Sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* BMA422_CFG_H */
