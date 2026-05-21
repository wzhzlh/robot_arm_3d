#include "cmsis_os.h"
#include "main.h"
#include "drive.h"
#include "commuction.h"
#include "k230.h"
extern ServoBus_t arm;
//void requirement_1(void  * argument);
void requirement_2(void * argument);
void requirement_3(void  * argument);
void requirement_4(void  * argument);
void requirement_5(void  * argument);
static void set_angles(float th1, float th2, float th3, uint16_t move_time);