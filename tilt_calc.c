/**
 * @file    tilt_calc.c
 * @brief   倾角计算 —— 纯角度算法模块实现（纯 C，无平台依赖，可移植）。
 *
 * ============================================================================
 *  处理流水线（每次 TiltCalc_Process 喂入一帧新样本）
 * ============================================================================
 *   1) 无死区定点 IIR 低通（alpha = 1 / 2^TILTCALC_FILTER_SHIFT，默认 1/8）：
 *        状态以 value<<SHIFT 定点保存，累加余数不丢失，不存在传统
 *        `filter += (raw - filter) >> 3` 那种 1~7 LSB 追不到的截断死区，
 *        滤波后读数与真实值无静差（这一点直接决定角度读数准不准）。
 *        SHIFT = 0 时直通，等于关闭滤波。
 *   2) 计算离水平总倾斜角：
 *        水平分量 L = sqrt(Y^2 + Z^2)
 *        敏感轴分量 V = |X|
 *        angle      = atan2(L, V)          （0 = 水平，90° = 垂直）
 *   3) 结果存入缓存，供 TiltCalc_GetResult / TiltCalc_GetAngleX100 读取。
 *
 *  就这三步。没有阈值比较、没有状态机、没有方向/扇区、没有门控、没有回调。
 */

#include "tilt_calc.h"
#include <math.h>

/* ---------------- 内部状态 ---------------- */
typedef struct {
    /* 定点低通状态：真实值 = state >> TILTCALC_FILTER_SHIFT */
    int32_t filt_x;
    int32_t filt_y;
    int32_t filt_z;
} TiltCalc_Context;

static TiltCalc_Context s_ctx;

/* 最近一次计算结果缓存（供 GetResult / GetAngleX100 读取）。 */
static TiltCalc_Result s_last_result;

/* ==========================================================================
 * 内部：由三轴读数求 水平分量 / 敏感轴分量 / 角度
 * ========================================================================== */

/**
 * @brief  核心几何计算（纯函数，无状态、无副作用）。
 * @param  x / y / z  逻辑轴读数（LSB）
 * @param  horiz      输出：水平分量幅值 L = sqrt(Y^2+Z^2)
 * @param  vert       输出：敏感轴分量 V = |X|
 * @return 离水平总倾斜角，0.01°（0 ~ 9000）
 */
static uint16_t TiltCalc_Geometry(int16_t x, int16_t y, int16_t z,
                                  uint16_t *horiz, uint16_t *vert)
{
    int32_t xa;      /* |X|，用 32 位避免 -32768 取反溢出 */
    int32_t ys;
    int32_t zs;
    int32_t l2;      /* Y^2 + Z^2 */
    float   lf;      /* sqrt(Y^2+Z^2) */
    float   ang;

    xa = (x < 0) ? -((int32_t)x) : (int32_t)x;
    ys = (int32_t)y;
    zs = (int32_t)z;

    l2 = ys * ys + zs * zs;
    lf = sqrtf((float)l2);

    /* 角度 = atan2(水平分量, 敏感轴分量)。
     * 分母 |X| 为 0（设备完全侧立）时 atan2 自然给出 90°，无需特判。 */
    ang = atan2f(lf, (float)xa) * TILTCALC_RAD_TO_X100;

    if (horiz != 0) {
        *horiz = (lf > 65535.0f) ? 65535u : (uint16_t)lf;
    }
    if (vert != 0) {
        *vert = (xa > 65535) ? 65535u : (uint16_t)xa;
    }

    if (ang < 0.0f) {
        ang = 0.0f;
    }
    if (ang > (float)TILTCALC_MAX_X100) {
        ang = (float)TILTCALC_MAX_X100;
    }
    return (uint16_t)ang;
}

/* ==========================================================================
 * 对外 API
 * ========================================================================== */

void TiltCalc_Init(void)
{
    s_ctx.filt_x = 0;
    s_ctx.filt_y = 0;
    s_ctx.filt_z = 0;

    s_last_result.angle_x100 = 0u;
    s_last_result.angle_raw_x100 = 0u;
    s_last_result.horiz_lsb = 0u;
    s_last_result.vert_lsb = 0u;
    s_last_result.filt_x = 0;
    s_last_result.filt_y = 0;
    s_last_result.filt_z = 0;
}

void TiltCalc_Reset(void)
{
    TiltCalc_Init();
}

void TiltCalc_Process(int16_t x, int16_t y, int16_t z)
{
    int16_t xf;
    int16_t yf;
    int16_t zf;

    /* ---- 1. 无死区定点 IIR 低通 ----
     *  定点形式：acc += raw - (acc >> SHIFT)
     *  读回：    out  = acc >> SHIFT
     *  每步的小数部分全部保留在累加器低位，不产生截断死区与静差。
     *  SHIFT = 0 时：acc += raw - acc  =>  acc = raw，等价于直通。 */
    s_ctx.filt_x += (int32_t)x - (s_ctx.filt_x >> TILTCALC_FILTER_SHIFT);
    s_ctx.filt_y += (int32_t)y - (s_ctx.filt_y >> TILTCALC_FILTER_SHIFT);
    s_ctx.filt_z += (int32_t)z - (s_ctx.filt_z >> TILTCALC_FILTER_SHIFT);

    xf = (int16_t)(s_ctx.filt_x >> TILTCALC_FILTER_SHIFT);
    yf = (int16_t)(s_ctx.filt_y >> TILTCALC_FILTER_SHIFT);
    zf = (int16_t)(s_ctx.filt_z >> TILTCALC_FILTER_SHIFT);

    /* ---- 2. 原始角度（未滤波，供对比/调试） ---- */
    s_last_result.angle_raw_x100 = TiltCalc_Geometry(x, y, z, 0, 0);

    /* ---- 3. 滤波后角度 + 中间量 ---- */
    s_last_result.angle_x100 = TiltCalc_Geometry(xf, yf, zf,
                                                 &s_last_result.horiz_lsb,
                                                 &s_last_result.vert_lsb);
    s_last_result.filt_x = xf;
    s_last_result.filt_y = yf;
    s_last_result.filt_z = zf;
}

void TiltCalc_GetResult(TiltCalc_Result *out)
{
    if (out != 0) {
        *out = s_last_result;
    }
}

uint16_t TiltCalc_GetAngleX100(void)
{
    return s_last_result.angle_x100;
}

uint16_t TiltCalc_ComputeAngleX100(int16_t x, int16_t y, int16_t z)
{
    return TiltCalc_Geometry(x, y, z, 0, 0);
}
