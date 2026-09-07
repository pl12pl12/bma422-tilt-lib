/**
 * @file    bma422_cfg.c
 * @brief   BMA422 加速度传感器 —— 可移植配置实现（纯 C）。
 *
 * ============================================================================
 *  本文件职责（只做"传感器层"，不含角度计算与滤波）
 * ============================================================================
 *   1. I2C 回调注入与地址自动探测（0x18 / 0x19）；
 *   2. 初始化序列：关 APS -> ACC_CONF -> 量程 -> 使能；
 *   3. 12-bit 补码数据读取、符号扩展、物理->逻辑轴重映射、软件 offset 扣除；
 *   4. 水平校零（软件 offset 方案，不写硬件 OFFSET 寄存器 0x71~0x73）。
 *
 *  所有写寄存器之间保持 >= 1ms 间隔：数据手册 4.2 节要求 APS 关闭前
 *  寄存器写入间隔 >= 1000us，这里统一在每次写入后延时 1ms，稳妥覆盖。
 *
 * ============================================================================
 *  默认参数（与 huada 量产固件一致，2026-09 实测通过）
 * ============================================================================
 *   器件输出        ：±2g，1024 LSB/g，12-bit 补码
 *   ACC_CONF = 0x47 ：power-optimized | AVG16(硬件16次平均) | 50Hz
 *                     实际有效输出 ~3.1Hz（50/16），每样本 ~0.32s
 *   轴映射          ：由 bma422_cfg.h 的 BMA422_AXIS_REMAP 决定（默认直通）
 *   ⚠ 实测：AVG32@100Hz 组合该片无 data-ready，禁止使用。
 */

#include "bma422_cfg.h"

/* ---------------- 校零参数 ---------------- */
#define BMA422_CAL_SAMPLES       8u    /* 校零取样的样本数（取平均抑制噪声） */
#define BMA422_CAL_POLL_US       5000u /* 等待数据就绪的轮询间隔 */
#define BMA422_CAL_TIMEOUT_US    1000000u /* 单样本最长等待(1s) */

/* ---------------- 内部状态 ---------------- */
static const BMA422_I2cOps *s_ops = 0;      /* 使用方注入的 I2C 回调 */
static uint8_t s_dev_addr = BMA422_I2C_ADDR_7BIT << 1;  /* 8-bit 写地址 */
static int16_t s_offset_x = 0;              /* 软件零点偏移（逻辑轴 LSB） */
static int16_t s_offset_y = 0;
static int16_t s_offset_z = 0;
static uint8_t s_calibrated = 0;            /* 是否完成过校零 */

/* 物理轴 -> 逻辑轴映射表（编译期由宏展开），负数表示该轴取反。 */
static const int8_t s_axis_remap[3] = BMA422_AXIS_REMAP;

/* ---------------- 寄存器读写封装（依赖注入的 I2C 回调） ---------------- */

static uint8_t BMA422_WriteRegister(uint8_t reg, uint8_t val)
{
    if ((s_ops == 0) || (s_ops->write_reg == 0)) {
        return 1u;   /* 未注册 I2C */
    }
    return s_ops->write_reg(s_dev_addr, reg, val);
}

static uint8_t BMA422_ReadRegister(uint8_t reg)
{
    if ((s_ops == 0) || (s_ops->read_reg == 0)) {
        return 0xFFu;
    }
    return s_ops->read_reg(s_dev_addr, reg);
}

/* 连续读：优先用回调的突发读；未提供则退化为单字节循环。 */
static void BMA422_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if ((s_ops == 0) || (buf == 0)) {
        return;
    }
    if (s_ops->read_multi != 0) {
        s_ops->read_multi(s_dev_addr, reg, buf, len);
    } else {
        for (i = 0u; i < len; i++) {
            buf[i] = BMA422_ReadRegister((uint8_t)(reg + i));
        }
    }
}

static void BMA422_DelayUs(uint32_t us)
{
    if ((s_ops != 0) && (s_ops->delay_us != 0)) {
        s_ops->delay_us(us);
    }
}

/* 把 2 字节(LSB 在前)的 12-bit 左对齐补码扩展成带符号 16-bit。
 * 12-bit 数据占用高 12 位，低 4 位为 0，故直接算术右移 4 位即可。 */
static int16_t BMA422_Combine12(uint8_t lsb, uint8_t msb)
{
    uint16_t raw = (uint16_t)((uint16_t)((uint16_t)msb << 8) | lsb);
    return (int16_t)((int16_t)raw >> 4);
}

/* ---------------- 对外 API ---------------- */

uint8_t BMA422_Init(const BMA422_I2cOps *ops)
{
    uint8_t chip_id;

    if (ops == 0) {
        return 1u;   /* 必须提供 I2C 回调 */
    }
    s_ops = ops;

    /* 地址自动探测：SDO 接 GND -> 0x18；接 VDDIO -> 0x19。 */
    s_dev_addr = BMA422_I2C_ADDR_7BIT << 1;
    chip_id = BMA422_ReadRegister(BMA422_CHIP_ID_REG);
    if (chip_id != BMA422_CHIP_ID_VALUE) {
        s_dev_addr = BMA422_I2C_ADDR_7BIT_2 << 1;
        chip_id = BMA422_ReadRegister(BMA422_CHIP_ID_REG);
        if (chip_id != BMA422_CHIP_ID_VALUE) {
            return 2u;   /* 两个地址都探测不到器件 */
        }
    }

    /* 初始化序列（每次写后延时 >= 1ms，见文件头注释）：
     *  1) 关闭高级省电(APS)，否则后续寄存器写入需 >1ms 间隔；
     *  2) ACC_CONF = 0x47：power-optimized | AVG16 | 50Hz(->~3.1Hz)；
     *  3) 量程 ±2g（1024 LSB/g，与 MMA8452 一致）；
     *  4) PWR_CTRL bit2 = 1 使能加速度计。 */
    BMA422_WriteRegister(BMA422_REG_PWR_CONF, BMA422_PWR_CONF_APS_OFF);
    BMA422_DelayUs(1000u);
    BMA422_WriteRegister(BMA422_REG_ACC_CONF, BMA422_ACC_CONF_VALUE);
    BMA422_DelayUs(1000u);
    BMA422_WriteRegister(BMA422_REG_ACC_RANGE, BMA422_ACC_RANGE_VALUE);
    BMA422_DelayUs(1000u);
    BMA422_WriteRegister(BMA422_REG_PWR_CTRL, BMA422_ACCEL_ENABLE);
    BMA422_DelayUs(1000u);

    s_offset_x = 0;
    s_offset_y = 0;
    s_offset_z = 0;
    s_calibrated = 0;
    return 0u;
}

uint8_t BMA422_DataReady(void)
{
    /* STATUS(0x03) bit7 = drdy_acc。注意：读 STATUS 会清除 drdy 标志，
     * 故返回 1 后应尽快读数据，避免漏采。 */
    return (uint8_t)((BMA422_ReadRegister(BMA422_REG_STATUS) & 0x80u) != 0u);
}

void BMA422_ReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    int16_t phys[3];   /* 物理轴原始值（符号扩展后） */
    int16_t out[3];    /* 逻辑轴输出（映射 + offset 后） */
    uint8_t i;
    int8_t src;
    uint8_t idx;

    if ((x == 0) || (y == 0) || (z == 0)) {
        return;
    }

    /* 一次突发读 6 字节：X_LSB X_MSB Y_LSB Y_MSB Z_LSB Z_MSB（LSB 在前）。 */
    BMA422_ReadMulti(BMA422_REG_DATA_X_LSB, data, 6u);

    phys[0] = BMA422_Combine12(data[0], data[1]);   /* 物理 X */
    phys[1] = BMA422_Combine12(data[2], data[3]);   /* 物理 Y */
    phys[2] = BMA422_Combine12(data[4], data[5]);   /* 物理 Z */

    /* 物理轴 -> 逻辑轴重映射。表元素：1=X 2=Y 3=Z，负数取反。 */
    for (i = 0u; i < 3u; i++) {
        src = s_axis_remap[i];
        idx = (uint8_t)((src < 0) ? (-src) : src);
        if ((idx < 1u) || (idx > 3u)) {
            out[i] = 0;   /* 非法映射值按 0 处理，避免越界 */
        } else {
            out[i] = (src < 0) ? (int16_t)(-phys[idx - 1u]) : phys[idx - 1u];
        }
    }

    /* 软件零点 offset 扣除（offset 存的是逻辑轴偏差）。 */
    *x = (int16_t)(out[0] - s_offset_x);
    *y = (int16_t)(out[1] - s_offset_y);
    *z = (int16_t)(out[2] - s_offset_z);
}

void BMA422_SetOffset(int16_t ox, int16_t oy, int16_t oz)
{
    /* 纯软件校零：只存 RAM，不写硬件 OFFSET 寄存器(0x71~0x73)。
     * BMA422 硬件 offset 的权重/符号约定与常见器件不同，软件方案
     * 更直观且无写入时序风险。掉电后需由外部存储恢复后重新调用。 */
    s_offset_x = ox;
    s_offset_y = oy;
    s_offset_z = oz;
    s_calibrated = 1u;
}

/* 等待一个新样本就绪；超时返回 0。 */
static uint8_t BMA422_WaitDataReady(uint32_t timeout_us)
{
    uint32_t waited = 0u;

    while (waited < timeout_us) {
        if (BMA422_DataReady() != 0u) {
            return 1u;
        }
        BMA422_DelayUs(BMA422_CAL_POLL_US);
        waited += BMA422_CAL_POLL_US;
    }
    return 0u;
}

void BMA422_CalibrateLevel(void)
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    uint8_t i;

    /* 校零前先清 offset，读到的才是"未补偿"的安装偏差。 */
    s_offset_x = 0;
    s_offset_y = 0;
    s_offset_z = 0;

    /* 连续采 BMA422_CAL_SAMPLES 个样本取平均，抑制单样本噪声。 */
    for (i = 0u; i < BMA422_CAL_SAMPLES; i++) {
        int16_t x;
        int16_t y;
        int16_t z;

        if (BMA422_WaitDataReady(BMA422_CAL_TIMEOUT_US) == 0u) {
            s_calibrated = 0u;
            return;   /* 数据没来，放弃校零 */
        }
        BMA422_ReadRaw(&x, &y, &z);
        sum_x += x;
        sum_y += y;
        sum_z += z;
    }

    /* 水平姿态的逻辑轴期望：X = -1g(-1024)，Y = Z = 0。
     * offset = 实测均值 - 期望，之后读取自动扣掉该安装偏差。 */
    s_offset_x = (int16_t)(sum_x / (int32_t)BMA422_CAL_SAMPLES + (int32_t)BMA422_LSB_PER_G);
    s_offset_y = (int16_t)(sum_y / (int32_t)BMA422_CAL_SAMPLES);
    s_offset_z = (int16_t)(sum_z / (int32_t)BMA422_CAL_SAMPLES);
    s_calibrated = 1u;
}

uint8_t BMA422_IsCalibrated(void)
{
    return s_calibrated;
}

void BMA422_Standby(void)
{
    /* PWR_CTRL bit2(acc_en) 清 0：关闭采样，寄存器配置保留。 */
    BMA422_WriteRegister(BMA422_REG_PWR_CTRL, 0x00u);
}

void BMA422_Active(void)
{
    BMA422_WriteRegister(BMA422_REG_PWR_CTRL, BMA422_ACCEL_ENABLE);
}

void BMA422_Sleep(void)
{
    /* 低功耗前调用：等同待机。唤醒后需要再调 BMA422_Active()。 */
    BMA422_WriteRegister(BMA422_REG_PWR_CTRL, 0x00u);
}
