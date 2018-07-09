#ifndef COLLECT_H_
#define COLLECT_H_

//unit: ms
#define POUR_DELAY 2000
#define CLAW_OUT_DELAY 5000
//uint degree

typedef enum
{
	COLLECT_REST,
	COLLECT_ALIGN,
	COLLECT_GRAB,
	COLLECT_POUR,
	COLLECT_RELEASE,
}Collect_State_t;

void Collect_Control(void);
void Collect_Mode_Switch(void);


#endif
