#include "SlaveFunc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"

//global variable
volatile spi_data spi_receive_buffer[BUFFER_SIZE],spi_send_buffer[BUFFER_SIZE];
volatile uint8_t spi_receive[SPI_SIZE];
volatile uint8_t spi_receive_position, spi_use_position;



void Slave_StateMachine() {
  switch (slave_sm_state){
    case Init:
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
  if(spi_receive_position >= BUFFER_SIZE){
    spi_receive_position = 0;
  }
  spi_receive_buffer[spi_receive_position].module = spi_receive[0];
  spi_receive_buffer[spi_receive_position].size = spi_receive[1];
  spi_receive_buffer[spi_receive_position].data = spi_receive[2];
  spi_receive_position++;
}

//place in state Idle
void IDLE_Func(){
  if(spi_use_position != spi_receive_position){
    switch (spi_receive_buffer[spi_use_position].module){
      case MODULE_Self :
        if(spi_receive_buffer[spi_use_position].data == ACTION_Open)
          SPI_Reply(MODULE_Self, REPLY_Here);
        else if(spi_receive_buffer[spi_use_position].data == ACTION_Close)
          SPI_Reply(MODULE_Self, REPLY_NA);
        else
          SPI_Reply(MODULE_Self, REPLY_NA);
      break;

      case MODULE_Buzzer :
        if(spi_receive_buffer[spi_use_position].data == ACTION_Open)
          Open_Buzzer();
        else if(spi_receive_buffer[spi_use_position].data == ACTION_Close)
          Close_Buzzer();
        else
          SPI_Reply(MODULE_Buzzer, REPLY_NA);
      break;

      case MODULE_Lock :
        if(spi_receive_buffer[spi_use_position].data == ACTION_Open)
          OpenThenClose_Lock();
        else
          SPI_Reply(MODULE_Lock, REPLY_NA);
      break;

      case MODULE_Led :
        if(spi_receive_buffer[spi_use_position].data == ACTION_Open)
          Open_LED();
        else if(spi_receive_buffer[spi_use_position].data == ACTION_Close)
          Close_LED();
        else
          SPI_Reply(MODULE_Led, REPLY_NA);
      break;

      default :
        SPI_Reply(REPLY_NA, REPLY_NA);
        break;

    }
  }
}