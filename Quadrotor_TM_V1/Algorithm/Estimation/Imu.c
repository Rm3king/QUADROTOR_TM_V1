/*
 * 模块：姿态解算 (Attitude Estimation)
 * 职责：维护四元数姿态、方向余弦矩阵 (DCM) 和欧拉角
 *
 * 算法概览 —— 互补滤波 (Complementary Filter)
 * ──────────────────────────────────────────────
 *   姿态用单位四元数 q = (w, x, y, z) 表示，每个周期执行：
 *
 *   1. 从四元数计算方向余弦矩阵 (DCM, att_matrix[3][3])。
 *      DCM 三行分别是载体 x/y/z 轴在世界坐标系下的单位方向向量。
 *
 *   2. 利用加速度计测量值与 DCM 第三行（等效重力向量）做叉积，
 *      得到姿态"倾斜误差" vec_err。
 *
 *   3. 利用磁力计测量值投影到世界坐标水平面，
 *      与已知磁北方向做叉积，得到"航向误差" mag_yaw_err。
 *
 *   4. 将误差以 PI 增益 (kp, ki) 叠加到陀螺仪角速度上，
 *      再用一阶微分方程更新四元数：
 *        dq = 0.5 * q ⊗ (ω + correction) * dT
 *
 *   5. 四元数归一化，保证单位长度。
 *
 *   6. 自适应调节增益：静态对准阶段使用高增益 (kp=10)，
 *      正常飞行使用低增益 (kp≈0.2)，由 G_reset / M_reset 标志控制。
 *
 * 坐标系约定 (ANO 坐标)
 * ──────────────────────
 *   俯视图，机头方向为 x 正方向：
 *        +x
 *         |
 *    +y ──┼──
 *         |
 *   世界坐标系：北-西-天 (NWU)
 */
#include "Imu.h"
#include "Math.h"
#include "Filter.h"


/* ══════════════════════════════════════════════════════
 *  坐标变换
 * ══════════════════════════════════════════════════════ */

/* 世界坐标平面 XY → 航向坐标 XY (绕 Z 轴旋转到机头方向) */
void w2h_2d_trans(float w[VEC_XYZ], float ref_ax[VEC_XYZ], float h[VEC_XYZ])
{
	h[X] =  w[X] *  ref_ax[X]  + w[Y] *ref_ax[Y];
	h[Y] =  w[X] *(-ref_ax[Y]) + w[Y] *ref_ax[X];
}

/* 航向坐标 XY → 世界坐标平面 XY */
void h2w_2d_trans(float h[VEC_XYZ], float ref_ax[VEC_XYZ], float w[VEC_XYZ])
{
	w[X] = h[X] *ref_ax[X] + h[Y] *(-ref_ax[Y]);
	w[Y] = h[X] *ref_ax[Y] + h[Y] *  ref_ax[X];
}

/* 载体坐标 → 世界坐标 (使用 DCM) */
static float att_matrix[3][3];
static void a2w_3d_trans(float a[VEC_XYZ], float w[VEC_XYZ])
{
	for(u8 i = 0; i<3; i++)
	{
		float temp = 0;
		for(u8 j = 0; j<3; j++)
		{
			temp += a[j] *att_matrix[i][j];
		}
		w[i] = temp;
	}
}

/* ══════════════════════════════════════════════════════
 *  姿态解算状态变量
 * ══════════════════════════════════════════════════════ */

_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};

_imu_state_st imu_state = {1,1,1,1,1,1,1,1};

/* 互补滤波中间量 */
static float vec_err[VEC_XYZ];      /* 加速度计 - 重力向量叉积误差 */
static float vec_err_i[VEC_XYZ];    /* 误差积分项 */
static float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3; /* 四元数交叉乘积缓存 */

/* 磁力计融合中间量 */
static float mag_yaw_err;           /* 航向误差 (叉积结果) */
static float mag_err_dot_product;   /* 航向方向点积 (判断同向/反向) */
static float mag_val_f[VEC_XYZ];    /* 磁力计浮点副本 */
static float s_mag_heading_ref[2][2] = {{1,0},{1,0}}; /* [0]=磁北参考(1,0) [1]=当前测量 */

/* 增益自适应 */
static float s_kp_use, s_ki_use, s_mkp_use; /* 当前生效的 PI / 磁力计增益 */
static float s_imu_reset_error_sum;          /* 对准收敛判据 */
static u16 reset_cnt;                        /* 对准收敛计时器 */

/* ══════════════════════════════════════════════════════
 *  增益自适应调节 (在四元数更新之后执行，为下一周期准备)
 *
 *  快速对准 (G_reset=1): kp=10, ki=0  → 误差收敛后自动切回
 *  正常融合 (G_reset=0): kp=gkp, ki=gki
 *  磁力计同理，由 M_reset 控制。
 * ══════════════════════════════════════════════════════ */

static void adjust_fusion_gains(_imu_state_st *state)
{
#ifdef USE_MAG
	if(state->M_fix_en==0)
	{
		s_mkp_use = 0;
		state->M_reset = 0;
	}
	else
	{
		if(state->M_reset)
		{
			s_mkp_use = 10.0f;
			if(mag_yaw_err != 0 && ABS(mag_yaw_err)<0.01f)
			{
				state->M_reset = 0;
			}
		}
		else
		{
			s_mkp_use = state->mkp;
		}
	}
#endif

	if(state->G_fix_en==0)
	{
		s_kp_use = 0;
	}
	else
	{
		if(state->G_reset == 0)
		{
			s_kp_use = state->gkp;
			s_ki_use = state->gki;
		}
		else
		{
			s_kp_use = 10.0f;
			s_ki_use = 0.0f;

			s_imu_reset_error_sum = (ABS(vec_err[X]) + ABS(vec_err[Y]));
			s_imu_reset_error_sum = LIMIT(s_imu_reset_error_sum,0,1.0f);

			if((s_imu_reset_error_sum < 0.02f) && (state->M_reset == 0))
			{
				reset_cnt += 2;
				if(reset_cnt>400)
				{
					reset_cnt = 0;
					state->G_reset = 0;
				}
			}
			else
			{
				reset_cnt = 0;
			}
		}
	}
}

/* ══════════════════════════════════════════════════════
 *  姿态解算主更新函数
 * ══════════════════════════════════════════════════════ */

void IMU_update(float dT,_imu_state_st *state,float gyr[VEC_XYZ], s32 acc[VEC_XYZ],s16 mag_val[VEC_XYZ],_imu_st *imu)
{
	float acc_norm_l,acc_norm_l_recip,q_norm_l;
	float acc_norm[VEC_XYZ];
	float d_angle[VEC_XYZ];

	/* ① 缓存四元数交叉乘积 (后续 DCM 计算多次用到) */
	q0q1 = imu->w * imu->x;
	q0q2 = imu->w * imu->y;
	q1q1 = imu->x * imu->x;
	q1q3 = imu->x * imu->z;
	q2q2 = imu->y * imu->y;
	q2q3 = imu->y * imu->z;
	q3q3 = imu->z * imu->z;
	q1q2 = imu->x * imu->y;
	q0q3 = imu->w * imu->z;

	/* ② 减去观测到的运动加速度，提取纯重力分量 */
	if(state->obs_en)
	{
		for(u8 i = 0;i<3;i++)
		{
			s32 temp = 0;
			for(u8 j = 0;j<3;j++)
			{
				temp += imu->obs_acc_w[j] *att_matrix[j][i];
			}
			imu->obs_acc_a[i] = temp;
			imu->gra_acc[i] = acc[i] - imu->obs_acc_a[i];
		}
	}
	else
	{
		for(u8 i = 0;i<3;i++)
		{
			imu->gra_acc[i] = acc[i];
		}
	}

	/* ③ 加速度计归一化 */
	acc_norm_l_recip = my_sqrt_reciprocal(my_pow(imu->gra_acc[X]) + my_pow(imu->gra_acc[Y]) + my_pow(imu->gra_acc[Z]));
	acc_norm_l = safe_div(1,acc_norm_l_recip,0);

	for(u8 i = 0;i<3;i++)
	{
		acc_norm[i] = imu->gra_acc[i] *acc_norm_l_recip;
	}

	/* ④ 从四元数计算 DCM (同时填充方向向量到 imu_data) */

	/* 载体 x 轴方向 */
    att_matrix[0][0] = imu->x_vec[X] = 1 - (2*q2q2 + 2*q3q3);
    att_matrix[0][1] = imu->x_vec[Y] = 2*q1q2 - 2*q0q3;
    att_matrix[0][2] = imu->x_vec[Z] = 2*q1q3 + 2*q0q2;

	/* 载体 y 轴方向 */
    att_matrix[1][0] = imu->y_vec[X] = 2*q1q2 + 2*q0q3;
    att_matrix[1][1] = imu->y_vec[Y] = 1 - (2*q1q1 + 2*q3q3);
    att_matrix[1][2] = imu->y_vec[Z] = 2*q2q3 - 2*q0q1;

    /* 载体 z 轴方向 (等效重力向量) */
    att_matrix[2][0] = imu->z_vec[X] = 2*q1q3 - 2*q0q2;
    att_matrix[2][1] = imu->z_vec[Y] = 2*q2q3 + 2*q0q1;
    att_matrix[2][2] = imu->z_vec[Z] = 1 - (2*q1q1 + 2*q2q2);

	/* 水平面航向向量 (x_vec 投影到水平面并归一化) */
	float hx_vec_reci = my_sqrt_reciprocal(my_pow(att_matrix[0][0]) + my_pow(att_matrix[1][0]));
	imu->hx_vec[X] = att_matrix[0][0] *hx_vec_reci;
	imu->hx_vec[Y] = att_matrix[1][0] *hx_vec_reci;

	/* ⑤ 计算运动加速度 (载体 → 世界 → 航向) */
	for(u8 i = 0;i<3;i++)
	{
		imu->a_acc[i] = (s32)(acc[i] - GRAVITY_CMSS *imu->z_vec[i]);
	}

	for(u8 i = 0;i<3;i++)
	{
		s32 temp = 0;
		for(u8 j = 0;j<3;j++)
		{
			temp += imu->a_acc[j] *att_matrix[i][j];
		}
		imu->w_acc[i] = temp;
	}

	w2h_2d_trans(imu->w_acc,imu_data.hx_vec,imu->h_acc);

	/* ⑥ 加速度计误差：测量值与等效重力向量的叉积 */
    vec_err[X] =  (acc_norm[Y] * imu->z_vec[Z] - imu->z_vec[Y] * acc_norm[Z]);
    vec_err[Y] = -(acc_norm[X] * imu->z_vec[Z] - imu->z_vec[X] * acc_norm[Z]);
    vec_err[Z] = -(acc_norm[Y] * imu->z_vec[X] - imu->z_vec[Y] * acc_norm[X]);

#ifdef USE_MAG
	/* ⑦ 磁力计误差：载体磁场投影到水平面，与磁北参考做叉积 */
	for(u8 i = 0;i<3;i++)
	{
		mag_val_f[i] = (float)mag_val[i];
	}

	if(!(mag_val[X] ==0 && mag_val[Y] == 0 && mag_val[Z] == 0))
	{
		a2w_3d_trans(mag_val_f,imu->w_mag);
		float l_re_tmp = my_sqrt_reciprocal(my_pow(imu->w_mag[0]) + my_pow(imu->w_mag[1]));
		s_mag_heading_ref[1][0] = imu->w_mag[0] *l_re_tmp;
		s_mag_heading_ref[1][1] = imu->w_mag[1] *l_re_tmp;

		mag_yaw_err = vec_2_cross_product(s_mag_heading_ref[1],s_mag_heading_ref[0]);
		mag_err_dot_product = vec_2_dot_product(s_mag_heading_ref[1],s_mag_heading_ref[0]);

		/* 反向时直接给最大误差 */
		if(mag_err_dot_product<0)
		{
			mag_yaw_err = my_sign(mag_yaw_err) *1.0f;
		}
	}
#endif

	/* ⑧ 融合：将误差叠加到陀螺仪角速度，计算增量旋转 */
	for(u8 i = 0;i<3;i++)
	{
#ifdef USE_EST_DEADZONE
		if(state->G_reset == 0 && state->obs_en == 0)
		{
			vec_err[i] = my_deadzone(vec_err[i],0,imu->gacc_deadzone[i]);
		}
#endif
#ifdef USE_LENGTH_LIM
		if(acc_norm_l>ACC_NORM_MAX || acc_norm_l<ACC_NORM_MIN)
		{
			vec_err[X] = vec_err[Y] = vec_err[Z] = 0;
		}
#endif
		vec_err_i[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) *dT *s_ki_use;

#ifdef USE_MAG
		d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * s_kp_use + mag_yaw_err *imu->z_vec[i] *s_mkp_use) * dT / 2 ;
#else
		d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * s_kp_use ) * dT / 2 ;
#endif
	}

	/* ⑨ 四元数一阶微分方程 + 归一化 */
    imu->w = imu->w            - imu->x*d_angle[X] - imu->y*d_angle[Y] - imu->z*d_angle[Z];
    imu->x = imu->w*d_angle[X] + imu->x            + imu->y*d_angle[Z] - imu->z*d_angle[Y];
    imu->y = imu->w*d_angle[Y] - imu->x*d_angle[Z] + imu->y            + imu->z*d_angle[X];
    imu->z = imu->w*d_angle[Z] + imu->x*d_angle[Y] - imu->y*d_angle[X] + imu->z;

	q_norm_l = my_sqrt_reciprocal(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w *= q_norm_l;
    imu->x *= q_norm_l;
    imu->y *= q_norm_l;
    imu->z *= q_norm_l;

	/* ⑩ 自适应调节增益 (为下一周期准备) */
	adjust_fusion_gains(state);
}

/* ══════════════════════════════════════════════════════
 *  欧拉角计算
 * ══════════════════════════════════════════════════════ */

static float t_temp;

/* 从 DCM 提取 Roll / Pitch / Yaw 欧拉角 (度) */
void calculate_RPY()
{
	t_temp = LIMIT(1 - my_pow(att_matrix[2][0]),0,1);

	/* z_vec[Z] 接近零时接近万向锁奇点，跳过计算 */
	if(ABS(imu_data.z_vec[Z])>0.05f)
	{
		imu_data.pit =  fast_atan2(att_matrix[2][0],my_sqrt(t_temp))* RAD_TO_DEG;
		imu_data.rol =  fast_atan2(att_matrix[2][1], att_matrix[2][2])* RAD_TO_DEG;
		imu_data.yaw = -fast_atan2(att_matrix[1][0], att_matrix[0][0])* RAD_TO_DEG;
	}
}
