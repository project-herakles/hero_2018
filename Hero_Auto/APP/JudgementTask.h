#ifndef _JUDGEMENT_TASK__
#define _JUDGEMENT_TASK__

#include "main.h"


typedef void * TaskHandle_t;

/** 
  * @brief  judgement data command id
  */

typedef enum
{
  GAME_INFO_ID       = 0x0001,
  REAL_BLOOD_DATA_ID = 0x0002,
  REAL_SHOOT_DATA_ID = 0x0003,
  REAL_POWER_DATA_ID = 0x0004,
  FIELD_RFID_DATA_ID = 0x0005,
  GAME_RESULT_ID     = 0x0006,
  GAIN_BUFF_ID       = 0x0007,
  ROBOT_POS_DATA_ID  = 0x0008,
  
  STU_CUSTOM_DATA_ID = 0x0100,
  ROBOT_TO_CLIENT_ID = 0x0101,
  CLIENT_TO_ROBOT_ID = 0x0102,
} judge_data_id_e;


/** 
  * @brief  game information structures definition(0x0001)
  */
typedef __packed struct
{
  uint16_t   stage_remain_time;
  uint8_t    game_process;
  /* current race stage
   0 not start
   1 preparation stage
   2 self-check stage
   3 5 seconds count down
   4 fighting stage
   5 result computing stage */
  uint8_t    reserved;
  uint16_t   remain_hp;
  uint16_t   max_hp;
} game_robot_state_t;

/** 
  * @brief  real time blood volume change data(0x0002)
  */
typedef __packed struct
{
  uint8_t armor_type:4;
 /* 0-3bits: the attacked armor id:
    0x00: 0 front
    0x01£º1 left
    0x02£º2 behind
    0x03£º3 right
    others reserved*/
  uint8_t hurt_type:4;
 /* 4-7bits: blood volume change type
    0x00: armor attacked
    0x01£ºmodule offline
    0x02: bullet over speed
    0x03: bullet over frequency */
} robot_hurt_data_t;

/** 
  * @brief  real time shooting data(0x0003)
  */
typedef __packed struct
{
  uint8_t bullet_type;
  uint8_t bullet_freq;
  float   bullet_spd;
} real_shoot_t;

/** 
  * @brief  real chassis power and shoot heat data(0x0004)
  *         icra need not this data
  */
typedef __packed struct
{
  float chassis_volt;
  float chassis_current;
  float chassis_power;
  float chassis_pwr_buf;
  uint16_t shooter1_heat;
  uint16_t shooter2_heat;
} real_power_data_t;

/** 
  * @brief  field rfid data(0x0005)
  */
typedef __packed struct
{
  uint8_t card_type;
  uint8_t card_idx;
} field_rfid_t;

/** 
  * @brief  game result data(0x0006)
  */
typedef __packed struct
{
  uint8_t winner;
} game_result_t;

/** 
  * @brief  the data of get field buff(0x0007)
  */
typedef __packed struct
{
  uint16_t buff_musk;
} get_buff_t;

/** 
  * @brief  field UWB data(0x0008)
  */
typedef __packed struct
{
  float x;
  float y;
  float z;
  float yaw;
} robot_position_t;

/** 
  * @brief  student custom data
  *         icra need not these data
  */
typedef __packed struct
{
  float data1;
  float data2;
  float data3;
} client_show_data_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

typedef enum{
	SUCCESSFUL=0,
	UNSUCCESSFUL=1,
}Receive_Info_e;

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
  game_robot_state_t game_information;   //0x0001
  robot_hurt_data_t  blood_changed_data; //0x0002
  real_shoot_t       real_shoot_data;    //0x0003
  real_power_data_t  power_heat_data;    //0x0004
  field_rfid_t       rfid_data;          //0x0005
  game_result_t      game_result_data;   //0x0006
  get_buff_t         get_buff_data;      //0x0007
  robot_position_t   robot_pos_data;     //0x0008
} receive_judge_t;


//for processing data
//used to receive frame header(5 bits)
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
	STEP_CMD_ID_LOW  = 5,
	STEP_CMD_ID_HIGH = 6,
  STEP_DATA_CRC16  = 7,
} unpack_step_e;


typedef struct
{
  frame_header_t *p_header;
  uint16_t       data_len;
	uint8_t        seq;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
	uint16_t       command_id;
} unpack_data_t;

/* data send (forward) */
/* data receive */
extern receive_judge_t judge_rece_mesg;

void  judgement_handler(uint8_t data,uint8_t sof);//proceed received data
void  judgement_data_handler(uint8_t *p_frame);
//void  judge_unpack_task(void const *argu);






#endif

