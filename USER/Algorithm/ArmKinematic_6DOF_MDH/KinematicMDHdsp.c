//
// Created by 刘嘉俊 on 26-3-7.
//

#include "KinematicMDHdsp.h"


/**  关节角θ  沿 z?轴的偏移量 d?  沿 x?轴的长度 a?  绕 x?轴的扭转角α  **/
// q[6] 要传弧度
// theta_offset 也必须是弧度
// alpha 也必须是弧度
const MDH_Param_t arm_mdh_table[6] = {
        {0.0f, 0.0f,   0.0f,  0.0f},
        {0.0f, 0.0f,   0.0f, -M_PI_2},
        {0.0f, 0.149f, 0.4318f,   0.0f},
        {0.0f, 0.433f, 0.0203f,   -M_PI_2},
        {0.0f, 0.0f,   0.0f,   M_PI_2},
        {0.0f, 0.0f,   0.0f,   -M_PI_2}
};

/**
 * @brief 旋转矩阵转欧拉角（Z-Y-X顺序）（YAW PITCH ROLL）（YPR）
 * @param _rotationM 输入3x3旋转矩阵
 * @param _eulerAngles 输出欧拉角 [A, B, C] = [yaw, pitch, roll]（单位：弧度）
 * cc = cos(C) cb = cos(B) ca = cos(A) sc = sin(C) sb = sin(B) sa = sin(A)
 */
static void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles) {
    float A, B, C, cb;
    // 欧拉角奇异位置处理，旋转矩阵解出来可能导致欧拉角有无穷解或者无解，此时需要特殊处理
    if (fabsf(_rotationM[6]) >= 1.0f - 0.0001f) {
        if (_rotationM[6] < 0) {
            A = 0.0f;
            B = M_PI_2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        } else {
            A = 0.0f;
            B = -M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    } else {
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(B);
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    _eulerAngles[0] = A; // yaw
    _eulerAngles[1] = B; // pitch
    _eulerAngles[2] = C; // roll
}

/**
 * @brief 欧拉角转旋转矩阵（Z-Y-X顺序）
 * @param _eulerAngles 输入欧拉角 [A, B, C] = [yaw, pitch, roll]（单位：弧度）
 * @param _rotationM 输出3x3旋转矩阵
 */
static void EulerAngleToRotMat(const float* _eulerAngles, float* _rotationM) {
    float ca, cb, cc, sa, sb, sc;

    cc = cosf(_eulerAngles[2]);
    cb = cosf(_eulerAngles[1]);
    ca = cosf(_eulerAngles[0]);
    sc = sinf(_eulerAngles[2]);
    sb = sinf(_eulerAngles[1]);
    sa = sinf(_eulerAngles[0]);

    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
}

// 使用改进型MDH参数法构建标准齐次变换矩阵，具体公式参照机器人运动学中标准DH法齐次变换矩阵的构建
static void MDH_Build_T(const MDH_Param_t *mdh_table, float joint_rad, float T_buf[16], Matrix *T)
{
    float theta = joint_rad + mdh_table->theta_offset;
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(mdh_table->alpha_prev);
    float sa = sinf(mdh_table->alpha_prev);

    // 按行存储 4x4 改进DH齐次变换矩阵
    // A_i = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
    // ct = cos_θ  st = sin_θ  ca = cos_α  sa = sin_α
    T_buf[0]  = ct;          T_buf[1]  = -st;         T_buf[2]  = 0.0f;   T_buf[3]  = mdh_table->a_prev;
    T_buf[4]  = st * ca;     T_buf[5]  = ct * ca;     T_buf[6]  = -sa;    T_buf[7]  = -mdh_table->d * sa;
    T_buf[8]  = st * sa;     T_buf[9]  = ct * sa;     T_buf[10] = ca;     T_buf[11] = mdh_table->d * ca;
    T_buf[12] = 0.0f;        T_buf[13] = 0.0f;        T_buf[14] = 0.0f;   T_buf[15] = 1.0f;

    Matrix_Init(T, 4, 4, T_buf);
}


// q为指定输入的关节角度，构建六个齐次变换矩阵
static void MDH_Build_T_6(const MDH_Param_t mdh_table[6], const float q[6], float T_buf6[6][16], Matrix T[6])
{
    for (int i = 0; i < 6; i++) {
        MDH_Build_T(&mdh_table[i], q[i], T_buf6[i], &T[i]);
    }
}

// DSP矩阵运算库的arm_status枚举值	简单描述
// ARM_MATH_SUCCESS	执行成功
// ARM_MATH_ARGUMENT_ERROR	参数错误
// ARM_MATH_LENGTH_ERROR	数据缓冲区长度错误
// ARM_MATH_SIZE_MISMATCH	矩阵尺寸不兼容
// ARM_MATH_NANINF	计算出 NaN / 无穷大
// ARM_MATH_SINGULAR	矩阵奇异（无法求逆）
// ARM_MATH_TEST_FAILURE	库自测失败

// q[6] 输入：关节角； T06_buf[16] 输出：4×4 总齐次变换矩阵的数据； T06 输出：这个 4×4 矩阵的 DSP 句柄/封装对象
static bool MDH_ForwardKinematics(const MDH_Param_t table[6], const float q[6], float T06_buf[16], Matrix *T06)
{
    float T_buf[6][16];
    Matrix T[6];

    float T02_buf[16];
    float T03_buf[16];
    float T04_buf[16];
    float T05_buf[16];

    Matrix T02, T03, T04, T05;

    MDH_Build_T_6(table, q, T_buf, T);

    Matrix_Init(&T02, 4, 4, T02_buf);
    Matrix_Init(&T03, 4, 4, T03_buf);
    Matrix_Init(&T04, 4, 4, T04_buf);
    Matrix_Init(&T05, 4, 4, T05_buf);
    Matrix_Init(T06,  4, 4, T06_buf);

    if (Matrix_Multiply(&T[0], &T[1], &T02) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T02,  &T[2], &T03) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T03,  &T[3], &T04) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T04,  &T[4], &T05) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T05,  &T[5], T06)  != ARM_MATH_SUCCESS) return false;

    return true;
}

static void T06_ToPose6D(const float T06_buf[16], Pose6D_t *pose)
{
    float R[9];
    float euler[3];   // [C, B, A]，单位：弧度

    // 取位置
    pose->X = T06_buf[3];
    pose->Y = T06_buf[7];
    pose->Z = T06_buf[11];
    // 取左上角3x3旋转矩阵
    R[0] = T06_buf[0];
    R[1] = T06_buf[1];
    R[2] = T06_buf[2];

    R[3] = T06_buf[4];
    R[4] = T06_buf[5];
    R[5] = T06_buf[6];

    R[6] = T06_buf[8];
    R[7] = T06_buf[9];
    R[8] = T06_buf[10];

    memcpy(pose->R, R, 9 * sizeof(float));
    pose->hasR = true;

    RotMatToEulerAngle(R, euler);

    // RotMatToEulerAngle输出: euler[0]=YAW, euler[1]=PITCH, euler[2]=ROLL
    pose->YAW = euler[0];
    pose->PITCH = euler[1];
    pose->ROLL = euler[2];
    // 弧度转角度
    pose->yaw_deg = euler[0] * RAD_TO_DEG;
    pose->pitch_deg = euler[1] * RAD_TO_DEG;
    pose->roll_deg = euler[2] * RAD_TO_DEG;
}

bool MDH_FK_ToPose6D(const MDH_Param_t table[6], const float q[6], Pose6D_t *pose)
{
    float T06_buf[16];
    Matrix T06;

    if (pose == NULL) {
        return false;
    }

    if (!MDH_ForwardKinematics(table, q, T06_buf, &T06)) {
        return false;
    }

    T06_ToPose6D(T06_buf, pose);
    return true;
}