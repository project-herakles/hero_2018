#include "main.h"

void BSP_Init(void)//BSP stands for Board Support Package, providing bootloader and toolchain
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     
	PWM_Configuration();//PWM Frenquency: TIM5 400Hz,TIM9 500Hz AF:GPIOA PIN0|PIN1|PIN2
	Led_Configuration();//Init GPIOC PIN1|PIN2
	Laser_Configuration();//Init GPIOA PIN8
	TIM2_Configuration();	//Init TIM2,1M clock	
	MPU6050_Initialize(); //Init MPU6050
	TIM6_Configuration();	//Init TIM6 and set up 1000Hz INT
	Quad_Encoder_Configuration();//Init encoder (with something beyond understanding)
	CAN1_Configuration();
	CAN2_Configuration();//Init CAN bus AF:GPIOA PIN11||PIN12            
	USART1_Configuration(100000);//Init serial with baund 100000, interrupt: RX_IDLE(?) AF:GPIOB PIN7
	USART3_Configuration();//Init with baund 115200 INT:RXNE AF: GPIOB PIN10||PIN11
	TIM6_Start();//Start TIM6 and its INT at 1000Hz
	MPU6050_IntConfiguration();     
	MPU6050_EnableInt();    	
	
}

/*
In embedded systems, a board support package (BSP) is the layer of software containing hardware-specific 
drivers and other routines that allow a particular operating system (traditionally a real-time operating 
system, or RTOS) to function in a particular hardware environment (a computer or CPU card), integrated with 
the RTOS itself. Third-party hardware developers who wish to support a particular RTOS must create
a BSP that allows that RTOS to run on their platform. In most cases the RTOS image and license, the BSP containing it,
and the hardware are bundled together by the hardware vendor.

BSPs are typically customizable, allowing the user to specify which drivers and routines should be included in the build 
based on their selection of hardware and software options. For instance, a particular single-board computer might 
be paired with any of several graphics cards; in that case the BSP might include a driver for each graphics card supported;
when building the BSP image the user would specify which graphics driver to include based in his choice of hardware.

¡¾wikipedia¡¿
*/
