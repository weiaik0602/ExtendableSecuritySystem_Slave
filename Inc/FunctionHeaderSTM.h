/*
 * myHeader.h
 *
 *  Created on: 10 Jul 2018
 *      Author: ng_we
 */
#include <stdint.h>
#include <stdio.h>
#include "LAFunction.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"


extern ADC_HandleTypeDef hadc1;
//global variable
// define variables
//private definition
//#define SEMIHOSTING


#ifndef SEMIHOSTING
#define log(...)
#else
#define log printf
#endif
