#ifndef __PRINTGUN_H
#define __PRINTGUN_H


#include "stdio.h"
#include "main.h"
#include "stm32f1xx_hal.h"

#define println(format,...) printf( format"\r\n", ##__VA_ARGS__)  

int fputc(int ch, FILE *f);

#endif
