#ifndef _MOCKFUNC_H
#define _MOCKFUNC_H
#include <stdint.h>


void Open_LED();
void Close_LED();
void Open_Buzzer();
void Close_Buzzer();
void OpenThenClose_Lock();
void SPI_Reply(uint8_t module, uint8_t data);
#endif // _MOCKFUNC_H
