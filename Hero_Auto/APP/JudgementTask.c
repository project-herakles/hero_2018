#include "main.h"

static unpack_data_t judge_rx_obj;//raw data
receive_judge_t judge_rece_mesg;//processed data
//for debug
Receive_Info_e judgement_fetch_flag = UNSUCCESSFUL; //UNSUCCESSFUL: fail to fetch data
Receive_Info_e judgement_process_flag = UNSUCCESSFUL;

void judgement_handler(uint8_t byte,uint8_t sof)
{
    switch(judge_rx_obj.unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          judge_rx_obj.unpack_step = STEP_LENGTH_LOW;
          judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        }
        else
        {
          judge_rx_obj.index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        judge_rx_obj.data_len = byte;
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        judge_rx_obj.unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        judge_rx_obj.data_len |= (byte << 8);
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;

        if(judge_rx_obj.data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
        {
          judge_rx_obj.unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          judge_rx_obj.unpack_step = STEP_HEADER_SOF;
          judge_rx_obj.index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
				judge_rx_obj.seq = byte;
        judge_rx_obj.unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;

        if (judge_rx_obj.index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(judge_rx_obj.protocol_packet, HEADER_LEN) )
          {
            judge_rx_obj.unpack_step = STEP_CMD_ID_LOW;
          }
          else
          {
						judgement_process_flag = UNSUCCESSFUL;
            judge_rx_obj.unpack_step = STEP_HEADER_SOF;
            judge_rx_obj.index = 0;
          }
        }
      }break; 
			case STEP_CMD_ID_LOW:
			{
				judge_rx_obj.command_id = byte;
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        judge_rx_obj.unpack_step = STEP_CMD_ID_HIGH;
			}break;
			case STEP_CMD_ID_HIGH:
			{
				judge_rx_obj.command_id |= (byte << 8);
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
				judge_rx_obj.unpack_step = STEP_DATA_CRC16;
			}break;
      case STEP_DATA_CRC16:
      {
        if (judge_rx_obj.index < (HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN))
        {
           judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;  
        }
        if (judge_rx_obj.index >= (HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN))
        {
          judge_rx_obj.unpack_step = STEP_HEADER_SOF;
          judge_rx_obj.index = 0;

          if ( verify_crc16_check_sum(judge_rx_obj.protocol_packet, HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN) )
          {
            if (sof == UP_REG_ID)
            {
              //pc_data_handler(judge_rx_obj.protocol_packet);
            }
            else  //DN_REG_ID
            {
							judgement_process_flag = SUCCESSFUL;
              judgement_data_handler(judge_rx_obj.protocol_packet);
            }
          }
					else
						judgement_process_flag = UNSUCCESSFUL;
        }
      }break;

      default:
      {
        judge_rx_obj.unpack_step = STEP_HEADER_SOF;
        judge_rx_obj.index = 0;
      }break;
    }
  }


//for debug
uint16_t test_cmd_id;	

void judgement_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  uint8_t  invalid_cmd = 0;
//  test_cmd_id = *(p_frame + HEADER_LEN);
//	test_cmd_id |= (*(p_frame + HEADER_LEN +1)) << 8;
	switch (cmd_id)
  {
    case GAME_INFO_ID:
      memcpy(&judge_rece_mesg.game_information, data_addr, data_length);
    break;

    case REAL_BLOOD_DATA_ID:
      memcpy(&judge_rece_mesg.blood_changed_data, data_addr, data_length);
    break;

    case REAL_SHOOT_DATA_ID:
      memcpy(&judge_rece_mesg.real_shoot_data, data_addr, data_length);
    break;

    case FIELD_RFID_DATA_ID:
      memcpy(&judge_rece_mesg.rfid_data, data_addr, data_length);
    break;

    case GAME_RESULT_ID:
      memcpy(&judge_rece_mesg.game_result_data, data_addr, data_length);
    break;

    case GAIN_BUFF_ID:
      memcpy(&judge_rece_mesg.get_buff_data, data_addr, data_length);
    break;
    
    case ROBOT_POS_DATA_ID:
      memcpy(&judge_rece_mesg.robot_pos_data, data_addr, data_length);
    break;
    
    default:
      invalid_cmd = 1;
    break;
  }
  test_cmd_id = cmd_id;
  /* valid forward data */
  if (!invalid_cmd)
  {
    //data_packet_pack(cmd_id, data_addr, data_length, UP_REG_ID);
    judgement_fetch_flag = UNSUCCESSFUL;
  }
	else
	{
		judgement_fetch_flag = SUCCESSFUL;
	  
	}
}

