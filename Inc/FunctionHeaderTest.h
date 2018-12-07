/*
 * myHeader.h
 *
 *  Created on: 10 Jul 2018
 *      Author: ng_we
 */
#include <stdint.h>
#include <stdio.h>
#include "LAFunction.h"


// definition

#define SEMIHOSTING


#ifndef SEMIHOSTING
#define log(...)
#else
#define log printf
#endif
