/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file imu_task.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief imu attitude calculation task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
  
#include "imu_task.h"
#include "bsp_imu.h"
#include "Rm_pid.h"
#include "math.h"
#include "tim.h"
#include "calibrate.h"
extern TIM_HandleTypeDef htim3;
#define DEFAULT_TUNE  300
//IMU temp control
#define IMU_PWM_PULSE      TIM3->CCR2
#define DEFAULT_IMU_TEMP   50

UBaseType_t imu_stack_surplus;

/* imu task global parameter */
mpu_data_t     mpu_data;
imu_data_t     imu;
imu_attitude_t atti;

/* imu task static parameter */
static volatile float q0 = 1.0f;
static volatile float q1 = 0.0f;
static volatile float q2 = 0.0f;
static volatile float q3 = 0.0f;

static volatile uint32_t last_update, now_update;
static volatile float exInt, eyInt, ezInt;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //
static volatile float q[4]; //　四元数

/**
  * @brief     Fast inverse square-root, to calculate 1/Sqrt(x)
  * @param[in] input:x
  * @retval    1/Sqrt(x)
  */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

#define BOARD_DOWN 1   //板子正面朝下摆放

void Init_Quaternion()//根据测量数据，初始化q0,q1,q2.q3，从而加快收敛速度
{
	
	q0 = 0.2164;
	q1 = 0;
	q2 = -0.9763;
	q3 = 0;
	
}

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 0.5f   // proportional gain governs rate of convergence to accelerometer/magnetometer originally 2.0f
#define Ki 0.00f   // integral gain governs rate of convergence of gyroscope biases originally 0.01f
float ex_s = 0;
float ey_s = 0;
float ez_s = 0;
float gx_s = 0;
float gy_s = 0;
float gz_s = 0;
float vx_s = 0;
float vy_s = 0;
float vz_s = 0;
float ax_s = 0;
float ay_s = 0;
float az_s = 0;
float qe0_s,qe1_s,qe2_s,qe3_s;
float errAng_s,norm_s;
float halfT;
void IMU_AHRSupdate(void) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;//,halfT;
    float tempq0,tempq1,tempq2,tempq3;
		
    float q0q0 = q0*q0;
    float q0q3 = q0*q3;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

		float errAng;
		float qe0,qe1,qe2,qe3;
		float q0_t,q1_t,q2_t,q3_t;
    gx = imu.wx;
  gy = imu.wy;
  gz = imu.wz;
  ax = imu.ax;
  ay = imu.ay;
  az = imu.az;
  mx = imu.mx;
  my = imu.my;
  mz = imu.mz;

  now_update = HAL_GetTick(); //ms
	if(now_update>=last_update)
      halfT =  ((float)(now_update - last_update) / 2000.0f);
  last_update = now_update;
    //快速求平方根算法
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
		
		ax_s = ax;
		ay_s = ay;
		az_s = az;
    //把加计的三维向量转成单位向量。
    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3; //verified
		vx_s = vx;
		vy_s = vy;
		vz_s = vz;
		
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors

		ex = -(ay*vz - az*vy);
    ey = -(az*vx - ax*vz);
    ez = -(ax*vy - ay*vx); //cross(v,a)
		
		ex_s = ex;
		ey_s = ey;
		ez_s = ez;
		norm = invSqrt(ex*ex+ey*ey+ez*ez);
		ex = ex * norm;
		ey = ey * norm;
		ez = ez * norm; // normalize vector e
		norm_s = norm;
		
		//errAng = asin(1.0f/norm)*0.02; //0.02 is the weight
		errAng = -0.002*atan2((1.0f/norm),(vx*ax+vy*ay+vz*az)); //errAng = atan(cross(A,B)/dot(A,B)),to eliminate the obtuse angle problem
		qe0 = cos(errAng/2.0f);
		qe1 = ex * sin(errAng/2.0f);
		qe2 = ey * sin(errAng/2.0f);
		qe3 = ez * sin(errAng/2.0f);
		
		qe0_s = qe0;
		qe1_s = qe1;
		qe2_s = qe2;
		qe3_s = qe3;
		
		errAng_s = errAng;
		
		/*
    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        // 用叉积误差来做PI修正陀螺零偏
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
		*/
		gx_s = gx;
		gy_s = gy;
		gz_s = gz;
		
		q0_t = q0;
		q1_t = q1;
		q2_t = q2;
		q3_t = q3;
		
/*
		q0 = qe0*q0_t - qe1*q1_t - qe2*q2_t - qe3*q3_t;
		q1 = qe0*q1_t + qe1*q0_t + qe2*q3_t - qe3*q2_t;
		q2 = qe0*q2_t - qe1*q3_t + qe2*q0_t + qe3*q1_t;
		q3 = qe0*q3_t + qe1*q2_t - qe2*q1_t + qe3*q0_t; //complementary filter
*/
		q0 = q0_t*qe0 - q1_t*qe1 - q2_t*qe2 - q3_t*qe3;
		q1 = q0_t*qe1 + q1_t*qe0 + q2_t*qe3 - q3_t*qe2;
		q2 = q0_t*qe2 - q1_t*qe3 + q2_t*qe0 + q3_t*qe1;
		q3 = q0_t*qe3 + q1_t*qe2 - q2_t*qe1 + q3_t*qe0; //complementary filter
		
		
    // 四元数微分方程
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

}

static void imu_attitude_update(void)
{
  imu.rol = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3; // roll       -pi----pi
  imu.pit = asin(-2*q1*q3 + 2*q0*q2)* 57.3;                         // pitch    -pi/2----pi/2 
  imu.yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; // yaw        -pi----pi
  
  
  if (imu.yaw - atti.last_yaw > 330)
    atti.yaw_cnt--;
  else if (imu.yaw - atti.last_yaw < -330)
    atti.yaw_cnt++;
  
  atti.last_yaw = imu.yaw;
  
  atti.yaw   = imu.yaw + atti.yaw_cnt*360;
  atti.pitch = imu.pit;
  atti.roll  = imu.rol;
  
  //gim.sensor.gyro_angle = atti.yaw;
}

void imu_param_init(void)
{
  Init_Quaternion();
  imu_temp_ctrl_init();
	cali_param_init();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	mpu_device_init();
}

uint32_t imu_time_last;
int imu_time_ms;
void imu_task()
{
  
    imu_time_ms = HAL_GetTick() - imu_time_last;
    imu_time_last = HAL_GetTick();
    
    imu_temp_keep();
    
    mpu_get_data();
	  IMU_AHRSupdate();
    imu_attitude_update();
    

}


void imu_temp_ctrl_init(void)
{
  PID_struct_init(&pid_imu_tmp, POSITION_PID, DEFAULT_TUNE, 150,
                  10, 1, 0);
}

void mpu_heat_ctrl(uint16_t pwm_pulse)
{
  IMU_PWM_PULSE = pwm_pulse;
}

void imu_temp_keep(void)
{
  imu.temp_ref = DEFAULT_IMU_TEMP;
  pid_calc(&pid_imu_tmp, imu.temp, imu.temp_ref);
  mpu_heat_ctrl(pid_imu_tmp.out);
}
