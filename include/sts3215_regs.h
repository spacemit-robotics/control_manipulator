/*
 * Copyright (c) 2025, spacemit. All rights reserved.
 *
 * STS3215 舵机寄存器地址定义
 *
 * 补充 SMS_STS.h 中未定义但 assemble/calibrate 流程所需的寄存器。
 * 地址来源：飞特 STS3215 数据手册。
 */
#ifndef MANIPULATOR_INCLUDE_STS3215_REGS_H_
#define MANIPULATOR_INCLUDE_STS3215_REGS_H_

/* ---- 已在 SMS_STS.h 中定义的寄存器（仅注释，不重复定义） ----
 *  SMS_STS_ID                  5
 *  SMS_STS_BAUD_RATE           6
 *  SMS_STS_MIN_ANGLE_LIMIT_L   9
 *  SMS_STS_MAX_ANGLE_LIMIT_L  11
 *  SMS_STS_CW_DEAD            26
 *  SMS_STS_CCW_DEAD           27
 *  SMS_STS_OFS_L              31   (Homing_Offset 低字节)
 *  SMS_STS_OFS_H              32   (Homing_Offset 高字节)
 *  SMS_STS_MODE               33   (Operating_Mode)
 *  SMS_STS_TORQUE_ENABLE      40
 *  SMS_STS_ACC                41
 *  SMS_STS_LOCK               55
 *  SMS_STS_PRESENT_POSITION_L 56
 */

/* ---- EPROM 区域（读写）---- */
#define STS3215_RETURN_DELAY_TIME     7    /* 返回延迟时间 (byte, 单位 2μs) */
#define STS3215_RESPONSE_STATUS_LEVEL 8    /* 应答等级 (byte) */
#define STS3215_MAX_TORQUE_LIMIT_L    16   /* 最大扭矩限制 低字节 (word) */
#define STS3215_MAX_TORQUE_LIMIT_H    17   /* 最大扭矩限制 高字节 */
#define STS3215_HIGHEST_LIMIT_TEMP    19   /* 最高温度限制 (byte, °C) */
#define STS3215_HIGHEST_LIMIT_VOL     20   /* 最高电压限制 (byte, 0.1V) */
#define STS3215_P_COEFFICIENT         21   /* P 系数 (byte) */
#define STS3215_D_COEFFICIENT         22   /* D 系数 (byte) */
#define STS3215_I_COEFFICIENT         23   /* I 系数 (byte) */
#define STS3215_PUNCH_L               24   /* 最小输出占空比 低字节 (word) */
#define STS3215_PUNCH_H               25   /* 最小输出占空比 高字节 */
#define STS3215_PROTECTION_CURRENT_L  28   /* 保护电流 低字节 (word, mA) */
#define STS3215_PROTECTION_CURRENT_H  29   /* 保护电流 高字节 */
#define STS3215_DRIVE_MODE            33   /* 驱动模式 (byte) */
#define STS3215_PROTECTIVE_TORQUE     34   /* 保护扭矩 (byte, 比例值) */
#define STS3215_PROTECTION_TIME       35   /* 保护时间 (byte, ms) */
#define STS3215_OVERLOAD_TORQUE       36   /* 过载扭矩 (byte, 比例值) */

/* ---- SRAM 区域（读写）---- */
#define STS3215_MAXIMUM_ACCELERATION  85   /* 最大加速度 (byte) */

/* ---- 常用位置分辨率 ---- */
#define STS3215_POSITION_MIN          0    /* 最小位置 ticks */
#define STS3215_POSITION_MAX          4095 /* 最大位置 ticks */
#define STS3215_POSITION_MID          2047 /* 中间位置 ticks */

/* ---- 默认 PID 参数 (适用于 SO-101) ---- */
#define STS3215_DEFAULT_P             32   /* 出厂默认 P */
#define STS3215_SO101_P               16   /* SO-101 推荐 P */
#define STS3215_SO101_I               0    /* SO-101 推荐 I */
#define STS3215_SO101_D               0    /* SO-101 推荐 D */

/* ---- 默认加速度参数 ---- */
#define STS3215_SO101_ACC             254  /* SO-101 推荐加速度 */
#define STS3215_SO101_MAX_ACC         254  /* SO-101 推荐最大加速度 */

/* ----夹爪保护参数 ---- */
#define STS3215_GRIPPER_MAX_TORQUE    500  /* 夹爪最大扭矩限制 */
#define STS3215_GRIPPER_PROT_CURRENT  250  /* 夹爪保护电流 (mA) */
#define STS3215_GRIPPER_OVERLOAD_TRQ  25   /* 夹爪过载扭矩比例 */

/* ---- REG_* 别名 (与 SMS_STS.h 中的地址一致) ---- */
#define REG_MIN_ANGLE_L     9    /* SMS_STS_MIN_ANGLE_LIMIT_L */
#define REG_MAX_ANGLE_L     11   /* SMS_STS_MAX_ANGLE_LIMIT_L */
#define REG_OFS_L           31   /* SMS_STS_OFS_L */
#define REG_MODE            33   /* SMS_STS_MODE */
#define REG_TORQUE_ENABLE   40   /* SMS_STS_TORQUE_ENABLE */
#define REG_ACC             41   /* SMS_STS_ACC */
#define REG_LOCK            55   /* SMS_STS_LOCK */
#define REG_PRESENT_POS_L   56   /* SMS_STS_PRESENT_POSITION_L */

#endif  // MANIPULATOR_INCLUDE_STS3215_REGS_H_
