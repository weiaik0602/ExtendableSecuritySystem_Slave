#ifndef _SLAVEFUNC_H
#define _SLAVEFUNC_H
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum sm_state sm_state;
enum sm_state{
  Init,
  Idle
};

typedef struct spi_data spi_data;
struct spi_data{
  uint8_t module;
  uint8_t size;
  uint8_t data;
};

//define
#define SPI_SIZE 10
#define BUFFER_SIZE 10
//module
#define MODULE_Self 0
#define MODULE_Buzzer 1
#define MODULE_Lock 2
#define MODULE_Led 3
//action
#define ACTION_Open 0
#define ACTION_Close 1
//reply
#define REPLY_Here 0
#define REPLY_NA 0xF

//variables
sm_state slave_sm_state;
extern volatile spi_data spi_receive_buffer[BUFFER_SIZE],spi_send_buffer[BUFFER_SIZE];
extern volatile uint8_t spi_receive[SPI_SIZE];
extern volatile uint8_t spi_receive_position, spi_use_position;





//Functions
void Slave_StateMachine();
void IDLE_Func();
void SPI_Receive_Buffer();
#endif // _SLAVEFUNC_H
