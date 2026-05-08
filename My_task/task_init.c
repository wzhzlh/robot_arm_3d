#include "FreeRTOS.h"
#include "task_init.h"
#include "start_task.h"
#include "math.h"

TaskHandle_t requirement_1_Handle;
TaskHandle_t requirement_2_Handle;
TaskHandle_t requirement_3_Handle;
TaskHandle_t requirement_4_Handle;  
TaskHandle_t requirement_5_Handle;
void task_init()
{
	vPortEnterCritical();

	xTaskCreate(requirement_1,
        "requirement_1",
         256,
         NULL,
         4,
         &requirement_1_Handle);
	xTaskCreate(requirement_2,
         "requirement_2",
          256,
          NULL,
          4,
          &requirement_2_Handle);
	xTaskCreate(requirement_3,
        "requirement_3",
         256,
         NULL,
         4,
         &requirement_3_Handle);

	xTaskCreate(requirement_4,
        "requirement_4",
         256,
         NULL,
         4,
         &requirement_4_Handle);
				 
	xTaskCreate(requirement_5,
        "sensor",       
         128,
         NULL,
         4,
         &requirement_5_Handle);
				 
	      vPortExitCritical();
}
