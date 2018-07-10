#include "main.h"
#include "sineWave.h"

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;     
PID_Regulator_t GMPSpeedPID = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;			
PID_Regulator_t GMYSpeedPID = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;

/*--------------------------------------------CTRL Variables----------------------------------------*/
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState = PREPARE_STATE;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
CM_Speed_Ref CM_sref;
uint32_t time_tick_1ms = 0;
extern int8_t emergency;
extern Speed_Mode_e speedMode;
static uint32_t start_time_1ms;
extern float swayRef;
/*
*********************************************************************************************************
*                                            FUNCTIONS 
*********************************************************************************************************
*/

static void SetWorkState(WorkState_e state)
{
    workState = state;
}

WorkState_e GetWorkState()
{
	return workState;
}


//底盘控制任务
static float CM1Speed_l_output=0;
static float CM2Speed_l_output=0;
static float CM3Speed_l_output=0;
static float CM4Speed_l_output=0;
int32_t abs_int(int32_t num)
{
	if(num>0) return num;
	else return -num;
}

float signOf(int32_t num)
{
	if(num>0) return 1.0;
	else return -1.0;
}

float acc_avg = 0;


void CMControlLoop(void)
{  
	//底盘旋转量计算
	if(GetWorkState()==PREPARE_STATE) //启动阶段，底盘不旋转
	{
		ChassisSpeedRef.rotate_ref = 0;	 
	}
	else
	{
		switch(GetDuckMode())
		{
			case DUCKING:
			{
				if(speedMode == SWAY)
					speedMode = FAST;//default:turn sway mode into fast mode
				CMRotatePID.ref = 45;
			}break;
			case NORMALPOSITION:
			default:
			{
			}
		}
		
		switch(speedMode)
		{
			case SWAY://In SWAY mode, give step inputs 
			{
				//底盘跟随编码器旋转PID计算
				if(CMRotatePID.ref==0) // initializing SWAY
				{
					CMRotatePID.ref = 45;
					start_time_1ms = time_tick_1ms;
				}
				else if(((float)(time_tick_1ms-start_time_1ms)) > 1000.0f/SWAY_FREQ) // flip input if time (1/freq) exceeds 
				{
					CMRotatePID.ref = -CMRotatePID.ref;
					start_time_1ms = time_tick_1ms; //reset start_time_1ms to start another cycle
				}
				//CMRotatePID.ref = sineWave(45,SWAY_FREQ,(time_tick_1ms-start_time_1ms)/1000.0);
				CMRotatePID.fdb = GMYawEncoder.ecd_angle;
				CMRotatePID.Calc(&CMRotatePID);   
				ChassisSpeedRef.rotate_ref = CMRotatePID.output;
			}break;
			default:// Chassis follows gimbal if not swaying
			{
				CMRotatePID.ref = 0;
				CMRotatePID.fdb = GMYawEncoder.ecd_angle;
				CMRotatePID.Calc(&CMRotatePID);   
				ChassisSpeedRef.rotate_ref = CMRotatePID.output;
			}
		}
	}
	if(Is_Lost_Error_Set(LOST_ERROR_RC))      //如果遥控器丢失，强制将速度设定值reset
	{
		ChassisSpeedRef.forward_back_ref = 0;
		ChassisSpeedRef.left_right_ref = 0;
	}
/*
	CM1SpeedPID.ref =  (-ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
	CM2SpeedPID.ref = (ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
	CM3SpeedPID.ref = (ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
	CM4SpeedPID.ref = (-ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
*/
/*
	CM_sref.CM1= (-ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
	CM_sref.CM2 = (ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
	CM_sref.CM3 = (ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
	CM_sref.CM4 = (-ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
*/
	
	CM_sref.CM1= (-ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
	CM_sref.CM2 = (ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
	CM_sref.CM3 = (ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
	CM_sref.CM4 = (-ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
	if(speedMode != SWAY)
	{
		if(fabs(CM_sref.CM1)>MOTOR_MAX_REF||fabs(CM_sref.CM2)>MOTOR_MAX_REF||fabs(CM_sref.CM3)>MOTOR_MAX_REF||fabs(CM_sref.CM4)>MOTOR_MAX_REF)//MOTOR_MAX_REF corresponds to roughly 1.5m/s
		{
			float offset = ChassisSpeedRef.rotate_ref;
			(CM_sref.CM1>MOTOR_MAX_REF) ? (CM_sref.CM1 -= offset) : (CM_sref.CM1+=offset);
			(CM_sref.CM2>MOTOR_MAX_REF) ? (CM_sref.CM2 -= offset) : (CM_sref.CM2+=offset);
			(CM_sref.CM3>MOTOR_MAX_REF) ? (CM_sref.CM3 -= offset) : (CM_sref.CM3+=offset);
			(CM_sref.CM4>MOTOR_MAX_REF) ? (CM_sref.CM4 -= offset) : (CM_sref.CM4+=offset);
		} // offset the rotate_ref, the only source of exceeding limit
		// in SWAY mode, don't care about excession
	}
	
	//feedback acceleration to restrict power
	//acc_avg = (abs_int(CM1Encoder.acc)+abs_int(CM2Encoder.acc)+abs_int(CM3Encoder.acc)+abs_int(CM4Encoder.acc))/4.0f;
/*
	if(acc_avg>5)//power control
	{
		float CM1_sign = signOf(CM1Encoder.acc);
		float CM2_sign = signOf(CM2Encoder.acc);
		float CM3_sign = signOf(CM3Encoder.acc);
		float CM4_sign = signOf(CM4Encoder.acc);

		CM1SpeedPID.ref = CM_sref.CM1 - (float)150.0*acc_avg*CM1_sign;
		CM2SpeedPID.ref = CM_sref.CM2 - (float)150.0*acc_avg*CM2_sign;
		CM3SpeedPID.ref = CM_sref.CM3 - (float)150.0*acc_avg*CM3_sign;
		CM4SpeedPID.ref = CM_sref.CM4 - (float)150.0*acc_avg*CM4_sign;
	}
	else
	{
		CM1SpeedPID.ref = CM_sref.CM1;
		CM2SpeedPID.ref = CM_sref.CM2;
		CM3SpeedPID.ref = CM_sref.CM3;
		CM4SpeedPID.ref = CM_sref.CM4;
	}
*/
	
	CM1SpeedPID.ref = CM_sref.CM1;
	CM2SpeedPID.ref = CM_sref.CM2;
	CM3SpeedPID.ref = CM_sref.CM3;
	CM4SpeedPID.ref = CM_sref.CM4;
	
/*
	CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075;
	CM2SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075;
	CM3SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075;
	CM4SpeedPID.ref = -ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075;
*/
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
	
	CM1SpeedPID.Calc(&CM1SpeedPID);
	CM2SpeedPID.Calc(&CM2SpeedPID);
	CM3SpeedPID.Calc(&CM3SpeedPID);
	CM4SpeedPID.Calc(&CM4SpeedPID);
	
		
	//control increase rate. Prevent overshoot of current and speed PID
	if(CM1SpeedPID.output-CM1Speed_l_output>MOTOR_INC_MAX||CM1SpeedPID.output-CM1Speed_l_output<-MOTOR_INC_MAX)
	{
		if(CM1SpeedPID.output-CM1Speed_l_output>MOTOR_INC_MAX) CM1SpeedPID.output = CM1Speed_l_output + MOTOR_INC_MAX;
		else CM1SpeedPID.output = CM1Speed_l_output - MOTOR_INC_MAX;
	}
	if(CM2SpeedPID.output-CM2Speed_l_output>MOTOR_INC_MAX||CM2SpeedPID.output-CM2Speed_l_output<-MOTOR_INC_MAX)
	{
		if(CM2SpeedPID.output-CM2Speed_l_output>MOTOR_INC_MAX) CM2SpeedPID.output = CM2Speed_l_output + MOTOR_INC_MAX;
		else CM2SpeedPID.output = CM2Speed_l_output - MOTOR_INC_MAX;
	}
	if(CM3SpeedPID.output-CM3Speed_l_output>MOTOR_INC_MAX||CM3SpeedPID.output-CM3Speed_l_output<-MOTOR_INC_MAX)
	{
		if(CM3SpeedPID.output-CM3Speed_l_output>MOTOR_INC_MAX) CM3SpeedPID.output = CM3Speed_l_output + MOTOR_INC_MAX;
		else CM3SpeedPID.output = CM3Speed_l_output - MOTOR_INC_MAX;
	}
	if(CM4SpeedPID.output-CM4Speed_l_output>MOTOR_INC_MAX||CM4SpeedPID.output-CM4Speed_l_output<-MOTOR_INC_MAX)
	{
		if(CM4SpeedPID.output-CM4Speed_l_output>MOTOR_INC_MAX) CM4SpeedPID.output = CM4Speed_l_output + MOTOR_INC_MAX;
		else CM4SpeedPID.output = CM4Speed_l_output - MOTOR_INC_MAX;
	}
	
	//power restriction
	float output_sum = fabs(CM1SpeedPID.output)+fabs(CM2SpeedPID.output)+fabs(CM3SpeedPID.output)+fabs(CM4SpeedPID.output);
	if(output_sum > 3000.0*4) //if power exceeds, reduce output proportionally
	{
		float factor = fabs(3000.0*4/output_sum);
		CM1SpeedPID.output *= factor;
		CM2SpeedPID.output *= factor;
		CM3SpeedPID.output *= factor;
		CM4SpeedPID.output *= factor;
	}
	
	 if((GetWorkState() == STOP_STATE) ||Is_Serious_Error() || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE)    //|| dead_lock_flag == 1紧急停车，编码器校准，无控制输入时都会使底盘控制停止
	 {
		 Set_CM_Speed(CAN2, 0,0,0,0);
	 }
	 else
	 {
		 Set_CM_Speed(CAN2,(int16_t)CM1SpeedPID.output,(int16_t)CM2SpeedPID.output,(int16_t)CM3SpeedPID.output,(int16_t)CM4SpeedPID.output);	
		 //Set_CM_Speed(CAN2,-1500,1500,1500,-1500);
	 } 
	 
	CM1Speed_l_output = CM1SpeedPID.output;
	CM2Speed_l_output = CM2SpeedPID.output;
	CM3Speed_l_output = CM3SpeedPID.output;
	CM4Speed_l_output = CM4SpeedPID.output;
	 
}
//发射机构射击电机任务
int16_t pwm_ccr = 0;
uint32_t last_shoot_start = 0;//shoot cycle: approx 1000ms per cycle
//ShootMotorState_e shooterState = PREPARE;SHOOTING;COOLING

void ShooterMControl(void)	
{	 
	ShootMotorSpeedPID.fdb = GetQuadEncoderDiff();
	ShootMotorPositionPID.fdb += (float)ShootMotorSpeedPID.fdb*360.0f/5041.2f;// Reduction ratio 13:1 
	ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
	  //ShootMotorSpeedPID.ref =  (PID_SHOOT_MOTOR_SPEED<ShootMotorPositionPID.output?PID_SHOOT_MOTOR_SPEED:ShootMotorPositionPID.output);
	if(GetShootMode()==BURST&&ShootMotorPositionPID.output>=50)//deadzone:output<50 degree: apply when 2 or more bullets at a time
	{
		ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED_HIGH;
	    ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);
	}
	else if(GetShootMode()==NORMAL&&ShootMotorPositionPID.output>=10)//deadzone:output<10 degree: apply when single bullet is needed
		{
			ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED_LOW;
		  ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);
		}
	else
	{
		ShootMotorSpeedPID.output = 0;
	}
	if(ShootMotorSpeedPID.output<0)
			ShootMotorSpeedPID.output=0;
	PWM3 = ShootMotorSpeedPID.output;	
}

void ShooterMControlLoop(void)
{
	//SetFrictionWheelSpeed(1600); //start friction wheel
	if(time_tick_1ms - last_shoot_start >= 1000 && GetShootState() == SHOOTING)
	{
		last_shoot_start = time_tick_1ms;
		if(GetShootMode() == BURST)
			ShootMotorPositionPID.ref += 180;
		else if(GetShootMode() == NORMAL)
			ShootMotorPositionPID.ref +=60;
	}

	ShooterMControl();
}


//控制任务，放在timer6 1ms定时中断中执行
void Control_Task(void)
{
	time_tick_1ms++; //Keep track of time
	WorkStateFSM(); //State machine: PREPARE NORMAL STANDBY CALI STOP
	WorkStateSwitchProcess();
	//启动后根据磁力计的数据初始化四元数
	if(time_tick_1ms <100)	
	{
		Init_Quaternion();	// Only carried out in the first 100ms
	}
	//平台稳定启动后，复位陀螺仪模块
	if(time_tick_1ms == PREPARE_TIME_TICK_MS/2) //time_tick == 2000ms
	{
		GYRO_RST();
	}
		
	//step 1: 云台控制
	GimbalYawControlModeSwitch();   //模式切换处理，得到位置环的设定值和给定值
	GMPitchControlLoop();
	GMYawControlLoop();
	SetGimbalMotorOutput();
	CalibrateLoop();   //校准任务，当接收到校准命令后才有效执行，否则直接跳过
	//chassis motor control
	if(time_tick_1ms%4 == 0)         //motor control frequency 4ms
	{
		//监控任务
		SuperviseTask();    
		//底盘控制任务
		CMControlLoop();			 
		ShooterMControlLoop();       //发射机构控制任务
	}
	
}
/**********************************************************
*工作状态切换状态机,与1ms定时中断同频率
**********************************************************/

void WorkStateFSM(void)
{
	lastWorkState = workState;
	switch(workState)
	{
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP || Is_Serious_Error())   /*InputMode = remote_input or key_mouse_input or stop. Specify how 
																														the system is controlled 
																													Serious_error = MPU6050_ERR or Deadclock_ERR or ZGYRO_ERR or NOCALL_ERR(?)*/
			{
				workState = STOP_STATE;
			}
			else if(GetCaliCmdFlagGrp())   
			{
				workState = CALI_STATE;
			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)
			{
				workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
			if(GetInputMode() == STOP || Is_Serious_Error() || emergency == -1)
			{
				workState = STOP_STATE;
			}
			else if(GetCaliCmdFlagGrp())   
			{
				workState = CALI_STATE;
			}
			else if((!IsRemoteBeingAction() ||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC)) && GetShootState() != SHOOTING)
			{
				workState = STANDBY_STATE;      
			}			
		}break;
		case STANDBY_STATE:     
		{
			if(GetInputMode() == STOP || Is_Serious_Error() || emergency == -1)
			{
				workState = STOP_STATE;
			}
			else if(GetCaliCmdFlagGrp())   
			{
				workState = CALI_STATE;
			}
			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
			{
				workState = NORMAL_STATE;
			}				
		}break;
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP && !Is_Serious_Error() && emergency != -1)
			{
				workState = PREPARE_STATE;   
			}
		}break;
		case CALI_STATE:      
		{
			if(GetInputMode() == STOP || Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
		}break;	    
		default:
		{
			
		}
	}	
}

static void WorkStateSwitchProcess(void)
{
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		ControtLoopTaskInit();
		RemoteTaskInit();
	}
}

/*
************************************************************************************************************************
*Name        : GimbalYawControlModeSwitch
* Description: This function process the yaw angle ref and fdb according to the WORKSTATE.
* Arguments  : void     
* Returns    : void
* Note(s)    : 1) from NORMAL to STANDBY it need to delay a few seconds to wait for the IMU to be steady.  
                  STATE_SWITCH_DELAY_TICK represents the delay time.
************************************************************************************************************************
*/

void GimbalYawControlModeSwitch(void)
{
	static uint8_t normalFlag = 0;   //正常工作模式标志
	static uint8_t standbyFlag = 1;  //IMU工作模式标志
	static uint32_t modeChangeDelayCnt = 0;
	//static float angleSave = 0.0f;    //用于切换模式时保存切换前的角度值，用于角度给定值切换
	switch(GetWorkState())
	{
		case PREPARE_STATE:   //启动过程，加入斜坡
		{
			GMYPositionPID.ref = 0.0f;
			GMYPositionPID.fdb = -GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);
			standbyFlag = 1; //for emergency, make it robust.
			//angleSave = ZGyroModuleAngle;			
		}break;
		case NORMAL_STATE:
		{
			if(standbyFlag == 1)
			{
				standbyFlag = 0;
				normalFlag = 1;
				GimbalRef.yaw_angle_dynamic_ref = yaw_angle;   //修改设定值为STANDBY状态下记录的最后一个ZGYROMODULEAngle值
				modeChangeDelayCnt = 0;   //delay清零
			}
			GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //设定给定值
			GMYPositionPID.fdb = yaw_angle; 					//设定反馈值
			//angleSave = yaw_angle;   //时刻保存IMU的值用于从NORMAL向STANDBY模式切换
		}break;
		case STANDBY_STATE:   //IMU模式
		{
			modeChangeDelayCnt++;
			if(modeChangeDelayCnt < STATE_SWITCH_DELAY_TICK)    //delay的这段时间与NORMAL_STATE一样
			{
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //设定给定值
				GMYPositionPID.fdb = yaw_angle; 					//设定反馈值
				//angleSave = yaw_angle;
			}
			else     //delay时间到，切换模式到IMU
			{
				if(normalFlag == 1)   //修改模式标志
				{
					normalFlag = 0;
					standbyFlag = 1;
					GimbalRef.yaw_angle_dynamic_ref = yaw_angle;    //保存的是delay时间段内保存的
				}
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //设定给定值
				GMYPositionPID.fdb = yaw_angle; 					//设定反馈值	
				//angleSave = ZGyroModuleAngle;           //IMU模式时，保存ZGyro的值供模式切换时修改给定值使用						
			}
		}break;
		case STOP_STATE:    //停止工作模式
		{
			
		}break;
		case CALI_STATE:    //校准模式
		{
			
		}break;
	}	
}

//云台pitch轴控制程序
void GMPitchControlLoop(void)
{
	GMPPositionPID.kp = PITCH_POSITION_KP_DEFAULTS + PitchPositionSavedPID.kp_offset;
	GMPPositionPID.ki = PITCH_POSITION_KI_DEFAULTS + PitchPositionSavedPID.ki_offset;
	GMPPositionPID.kd = PITCH_POSITION_KD_DEFAULTS + PitchPositionSavedPID.kd_offset;
		
	GMPSpeedPID.kp = PITCH_SPEED_KP_DEFAULTS + PitchSpeedSavedPID.kp_offset;
	GMPSpeedPID.ki = PITCH_SPEED_KI_DEFAULTS + PitchSpeedSavedPID.ki_offset;
	GMPSpeedPID.kd = PITCH_SPEED_KD_DEFAULTS + PitchSpeedSavedPID.kd_offset;
	
	GMPPositionPID.ref = GimbalRef.pitch_angle_dynamic_ref;
	GMPPositionPID.fdb = -GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp);    //加入斜坡函数
	GMPPositionPID.Calc(&GMPPositionPID);   //得到pitch轴位置环输出控制量
	//pitch speed control
	GMPSpeedPID.ref = GMPPositionPID.output;
	GMPSpeedPID.fdb = MPU6050_Real_Data.Gyro_Y;
	GMPSpeedPID.Calc(&GMPSpeedPID);
}

void GMYawControlLoop(void)
{
		/*
	GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS + YawPositionSavedPID.kp_offset;//  gAppParamStruct.YawPositionPID.kp_offset;  //may be bug if more operation  done
	GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS + YawPositionSavedPID.ki_offset;
	GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS + YawPositionSavedPID.kd_offset;
	*/
	GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS;// Manually set PID parameters
	GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS;
	GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS;
	
	/*GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS + YawSpeedSavedPID.kp_offset;
	GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS + YawSpeedSavedPID.ki_offset;
	GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS + YawSpeedSavedPID.kd_offset;*/
	GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS; // Manually set
	GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS;
	GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS;
	GMYPositionPID.Calc(&GMYPositionPID);
	//yaw speed control
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb = MPU6050_Real_Data.Gyro_Z;
	GMYSpeedPID.Calc(&GMYSpeedPID);			
}

void SetGimbalMotorOutput(void)
{
	//云台控制输出								
	if((GetWorkState() == STOP_STATE) ||Is_Serious_Error() || GetWorkState() == CALI_STATE)   
	{
		Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch			
	}
	else
	{		
		Set_Gimbal_Current(CAN2, (int16_t)GMYSpeedPID.output, (int16_t)GMPSpeedPID.output);     //yaw + pitch		
		//Set_Gimbal_Current(CAN2, 0, (int16_t)GMPSpeedPID.output);		
	}		
}
//控制任务初始化程序
void ControtLoopTaskInit(void)
{
	//计数初始化
	time_tick_1ms = 0;   //中断中的计数清零
	//程序参数初始化
	AppParamInit();
	//校准后参数偏差值初始化
	Sensor_Offset_Param_Init(&gAppParamStruct);
	//设置工作模式
	SetWorkState(PREPARE_STATE);
	//斜坡初始化
	GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
	GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);
	//云台给定角度初始化
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
    //监控任务初始化
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));
    //LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_NOCALI));
    
	//PID初始化
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	GMPPositionPID.Reset(&GMPPositionPID);
	GMPSpeedPID.Reset(&GMPSpeedPID);
	GMYPositionPID.Reset(&GMYPositionPID);
	GMYSpeedPID.Reset(&GMYSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
	
}


