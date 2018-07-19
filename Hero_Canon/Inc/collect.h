#ifndef COLLECT_H_
#define COLLECT_H_

//unit: ms
#define POUR_DELAY 4000
#define CLAW_OUT_DELAY 5000
#define RELEASE_DELAY 2000
#define GRAB_DELAY 5000

#define ARM_Angle_to_Encoder 77116.0f/180.0f
#define WITH_BOX 1
#define WITHOUT_BOX 0

typedef enum
{
	COLLECT_REST,
	COLLECT_ALIGN,
	COLLECT_GRAB,
	COLLECT_POUR,
	COLLECT_RELEASE,
}Collect_State_t;

typedef enum
{
	ARM_REST,
	ARM_IN,
	ARM_OUT,
}Arm_Mode_t;

void Collect_Control_Init(void);
void Collect_Control(void);
void Collect_Mode_Switch(void);


#endif
