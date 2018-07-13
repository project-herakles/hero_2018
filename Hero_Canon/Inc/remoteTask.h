#ifndef _REMOTETASK_H_
#define _REMOTETASK_H_

#include "stm32f4xx.h"

typedef __packed struct
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;

typedef __packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctrl_t;

typedef enum
{
	REMOTE_MODE,
	MOUSE_KEY_MODE,
	STOP_MODE
}InputMode_e;

typedef __packed struct
{
	int16_t forward_back_ref;
	int16_t left_right_ref;
	int16_t rotate_ref;
}Chassis_speed_ref_t;

typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ref_t;


#define REMOTE_CONTROLLER_STICK_OFFSET (1024)
#define STICK_TO_CHASSIS_SPEED_REF_FACT   (0.45f)
#define STICK_TO_CHASSIS_ROTATE_REF_FACT (0.002f)
#define NORMAL_FORWARD_BACK_SPEED 			150
#define NORMAL_LEFT_RIGHT_SPEED   			180
#define HIGH_FORWARD_BACK_SPEED 			250
#define HIGH_LEFT_RIGHT_SPEED   			300
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.004f

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

void remoteTaskInit(void);
void remoteDataProcess(uint8_t* pData);
void setInputMode(Remote *rc);
void Remote_Control(Remote* rc);
void MouseKey_Control(Mouse* mouse,Key* key);

InputMode_e getInputMode(void);

#endif
