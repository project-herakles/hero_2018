#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_
#include "stm32f4xx.h"
#include "pid.h"

//initiate status: 
typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态
    NORMAL_STATE,			//无输入状态
    STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;

typedef enum
{
	FRIC_STATIC,
	FRIC_ACCELERATING,
	FRIC_MAX_SPEED,
}frictionState_e;

typedef struct CM_SPEED_REF
{
	float CM1;
	float CM2;
	float CM3;
	float CM4;
}CM_Speed_Ref;

#define PREPARE_TIME_TICK_MS 4000      //prapare time in ms

#define STATE_SWITCH_DELAY_TICK 100000   //mode change delay count in ms

#define YAW_POSITION_KP_DEFAULTS  8
#define YAW_POSITION_KI_DEFAULTS  0
#define YAW_POSITION_KD_DEFAULTS  0
#define YAW_SPEED_KP_DEFAULTS  45 //original 15
#define YAW_SPEED_KI_DEFAULTS  0.01
#define YAW_SPEED_KD_DEFAULTS  0

// avoid bang --->  position:20.0  speed:19.0
//big bang   22.5 20.0
#define PITCH_POSITION_KP_DEFAULTS  15
#define PITCH_POSITION_KI_DEFAULTS  0
#define PITCH_POSITION_KD_DEFAULTS  0

#define PITCH_SPEED_KP_DEFAULTS  25
#define PITCH_SPEED_KI_DEFAULTS  0
#define PITCH_SPEED_KD_DEFAULTS  0

#define PID_SHOOT_MOTOR_SPEED_INIT (7)
#define PID_SHOOT_MOTOR_SPEED_HIGH      (30)
#define PID_SHOOT_MOTOR_SPEED_LOW       (10)
#define PID_SHOOT_MOTOR_CHECK_SPEED (3)

#define CHASSIS_SPEED_ATTENUATION   (1.0f)
#define MOTOR_MAX_REF (800.0f) //cruising limit 1000.0f
#define MOTOR_INC_MAX (800.0f) //was 800
#define ACC_FACTOR (10)
//P:1.0 follow very closely

extern PID_Regulator_t CMRotatePID;      		//底盘跟随云台PID
extern PID_Regulator_t GMPPositionPID;      	//pitch轴位置环PID
extern PID_Regulator_t GMPSpeedPID;      		//pitch轴速度换PID
extern PID_Regulator_t GMYPositionPID;			//yaw轴位置环PID
extern PID_Regulator_t GMYSpeedPID;      		//yaw轴速度环PID

extern PID_Regulator_t ShootMotorPositionPID;    //射击电机位置环PID
extern PID_Regulator_t ShootMotorSpeedPID;       //射击电机速度环PID

extern PID_Regulator_t CM1SpeedPID;				 //底盘电机速度环PID
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;

WorkState_e GetWorkState(void);
void ControtLoopTaskInit(void);
void WorkStateFSM(void);
static void WorkStateSwitchProcess(void);
void Control_Task(void);
void GMPitchControlLoop(void);
void GMYawControlLoop(void);
void GMPitchControlLoop(void);
void GimbalYawControlModeSwitch(void);   //模式切换处理，得到位置环的设定值和给定值
void SetGimbalMotorOutput(void);
uint32_t getCurrentTimeTick(void);
//void SetWorkState(WorkState_e);
#endif
