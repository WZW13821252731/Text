#include "printfun.h"

/**********************************************
*                LOG DEFINE                   *
**********************************************/
// OFF FATAL ERROR WARN INFO DEBUG TRACE
//  0    1     2     3    4    5     6
#define OFF   (0)
#define FATAL (1)
#define ERROR (2)
#define WARN  (3)
#define INFO  (4)
#define DEBUG (5)
#define TRACE (6)

#define LOG_LEVEL DEBUG

#define LOG(level, format, ...) \
    if(LOG_LEVEL >= level) { \
        fprintf(stderr, "[%s|%s@%s,%d] " format "\n", \
            #level, __func__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
    } 

extern UART_HandleTypeDef huart1;
int fputc(int ch,FILE *f)
{
   HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
   return ch;
}
