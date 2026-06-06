#include "FreeRTOS.h"
#include "task_init.h"
#include "start_task.h"
#include "math.h"

TaskHandle_t requirement_Handle;
TaskHandle_t mot_rece_Handle;
TaskHandle_t requirement_2_Handle;  
void task_init()
{
	vPortEnterCritical();

	xTaskCreate(requirement,
       "requirement",
        512,
        NULL,
        4,
        &requirement_Handle);
	// xTaskCreate(mot_rece,
    //      "mot_rece",
    //       512,  /* 增大栈空间，避免栈溢出 */
    //       NULL,
    //       4,
    //       &mot_rece_Handle);

//	xTaskCreate(requirement_2,
//        "requirement_2",
//         256,
//         NULL,
//         4,
//         &requirement_2_Handle);
//				 

	      vPortExitCritical();
}