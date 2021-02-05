#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#define NCSO 0X22
#define CNOS 0X19
#define NYO  0X0D
#define YTN  0X11
typedef uint8_t  UBYTE;
typedef uint16_t UWORD;
typedef uint32_t UDOUBLE;
#define NBYO NYO
#define YTNB YTN
typedef enum{
	NFR=0,
	NRR,
}esp_c_t;
typedef enum{
    BNCO=NBYO,
	TNCB=YTNB,
}BOT_t;
#define PSR     0X00
#define PWR     0X01
#define POF     0X02
#define PFS     0X03
#define PON     0X04
#define BTST    0X06
#define DTM     0X10
#define DRF     0X12
#define PLL     0X30
#define TSE     0X41
#define CDI     0X50
#define TCON    0X60
#define TRES    0X61
#define PWS     0XE3
#define Black   0X00
#define White   0X11
#define Green   0X22
#define Blue    0X33
#define Red     0X44
#define Yellow  0X55
#define Orange  0X66
#define Clean   0X77
#define AENCRESET 0X00
#define NCHREET 1
#define NCOD    0X1B
#define NCLKC   0X0C
#define NCOC    0X0F
#define DNCO    0X0E
#define RNCO    0X1A
#define BONC    0X0D
#define NTOC    0X05
#define BTNC    0X11
#define NCTO    0X10
#define NCTC    0X04
#define RNST    0X00
#define AENCSET 0X01
#define NCIS  ((1ULL<<TNCB)|(1ULL<<CNOS)|(1ULL<<BNCO))
#define NCLSET 0
void WK_AC(void);
void OTCO_O(UBYTE temp);





