#include "SlaveFunc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
//#include "main.h"
#include "mockFunc.h"

//global variable
volatile spi_data spi_receive_buffer[BUFFER_SIZE],spi_send_buffer[BUFFER_SIZE];
volatile uint8_t spi_receive[SPI_SIZE];
volatile uint8_t spi_receive_position, spi_use_position;
volatile uint8_t buttonPressed = 0;


void Slave_StateMachine() {
  switch (slave_sm_state){
    case Init:
    	slave_sm_state = Idle;
    break;

    case Idle:
      IDLE_Func();
    break;

    default:
    break;

  }
}

//place inside SPI-DMA interrupt
void SPI_Receive_Buffer(){
  if(spi_receive_position >= BUFFER_SIZE-1){
    spi_receive_position = 0;
  }
  spi_receive_buffer[spi_receive_position].module = spi_receive[0];
  spi_receive_buffer[spi_receive_position].size = spi_receive[1];
  spi_receive_buffer[spi_receive_position].data = spi_receive[2];
  spi_receive_position++;
}

void DMAS2_Func(spi_data spi){
  switch (spi_receive[0]){
    case MODULE_Self :
    	if(spi_receive[2] == ACTION_Open || spi_receive[2] == ACTION_Read){
      	Open_Self();
        SPI_Reply(MODULE_Self, REPLY_Here);
      }
      else{
        SPI_Reply(MODULE_Self, REPLY_NA);
      }
    break;

    case MODULE_Buzzer :
      if(spi_receive[2] == ACTION_Open){
        Open_Buzzer();
        SPI_Reply(MODULE_Buzzer, ACTION_Open);
      }
      else if(spi_receive[2] == ACTION_Close){
        Close_Buzzer();
        SPI_Reply(MODULE_Buzzer, ACTION_Close);
      }
      else if(spi_receive[2] == ACTION_Read){
      	Read_Buzzer();
      }
      else
        SPI_Reply(MODULE_Buzzer, REPLY_NA);
    break;

    case MODULE_Lock :
      if(spi_receive[2] == ACTION_Open){
        OpenThenClose_Lock();
        SPI_Reply(MODULE_Lock, ACTION_Open);
      }
      else if(spi_receive[2] == ACTION_Read){
				Read_Lock();
			}
      else
        SPI_Reply(MODULE_Lock, REPLY_NA);
    break;

    case MODULE_Led :
      if(spi_receive[2] == ACTION_Open){
        Open_LED();
        SPI_Reply(MODULE_Led, ACTION_Open);
      }
      else if(spi_receive[2] == ACTION_Close){
        Close_LED();
        SPI_Reply(MODULE_Led, ACTION_Close);
      }
      else if(spi_receive[2] == ACTION_Read){
				Read_Led();
			}
      else
        SPI_Reply(MODULE_Led, REPLY_NA);
    break;

    case MODULE_Button:
    	if(spi_receive[2] == ACTION_Open || spi_receive[2] == ACTION_Read){
				SPI_Reply(MODULE_Button, buttonPressed);
			}
    	else if(spi_receive[2] == ACTION_Close){
    		buttonPressed = 0;
    		SPI_Reply(MODULE_Button, ACTION_Close);
    	}
    	else
				SPI_Reply(MODULE_Button, REPLY_NA);
    	break;

    default :
      SPI_Reply(REPLY_NA, REPLY_NA);
      break;
  }
}

////place in state Idle
//void IDLE_Func(){
//  if(spi_use_position != spi_receive_position){
//    switch (spi_receive_buffer[spi_use_position].module){
//      case MODULE_Self :
//        if(spi_receive_buffer[spi_use_position].data == ACTION_Open)
//          SPI_Reply(MODULE_Self, REPLY_Here);
//        else if(spi_receive_buffer[spi_use_position].data == ACTION_Close)
//          SPI_Reply(MODULE_Self, REPLY_NA);
//        else
//          SPI_Reply(MODULE_Self, REPLY_NA);
//      break;
//
//      case MODULE_Buzzer :
//        if(spi_receive_buffer[spi_use_position].data == ACTION_Open){
//          Open_Buzzer();
//          SPI_Reply(MODULE_Buzzer, ACTION_Open);
//        }
//        else if(spi_receive_buffer[spi_use_position].data == ACTION_Close){
//          Close_Buzzer();
//          SPI_Reply(MODULE_Buzzer, ACTION_Close);
//        }
//        else
//          SPI_Reply(MODULE_Buzzer, REPLY_NA);
//      break;
//
//      case MODULE_Lock :
//        if(spi_receive_buffer[spi_use_position].data == ACTION_Open){
//          OpenThenClose_Lock();
//          SPI_Reply(MODULE_Lock, ACTION_Open);
//        }
//        else
//          SPI_Reply(MODULE_Lock, REPLY_NA);
//      break;
//
//      case MODULE_Led :
//        if(spi_receive_buffer[spi_use_position].data == ACTION_Open){
//          Open_LED();
//          SPI_Reply(MODULE_Led, ACTION_Open);
//        }
//        else if(spi_receive_buffer[spi_use_position].data == ACTION_Close){
//          Close_LED();
//          SPI_Reply(MODULE_Led, ACTION_Close);
//        }
//        else
//          SPI_Reply(MODULE_Led, REPLY_NA);
//      break;
//
//      default :
//        SPI_Reply(REPLY_NA, REPLY_NA);
//        break;
//
//    }
//    if(spi_use_position <= BUFFER_SIZE-1){
//    	spi_use_position ++;
//      }
//    else
//    	spi_use_position = 0;
//  }
//}
