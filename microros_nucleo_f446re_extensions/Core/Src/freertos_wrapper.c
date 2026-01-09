#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>
#include "allocators.h"
#include "usart.h"

extern void appMain(void *argument);

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void);

void StartDefaultTask(void *argument)
{
  // Setup transport
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    freertos_serial_open,
    freertos_serial_close,
    freertos_serial_write,
    freertos_serial_read);

  // Setup allocator
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = __freertos_allocate;
  freeRTOS_allocator.deallocate = __freertos_deallocate;
  freeRTOS_allocator.reallocate = __freertos_reallocate;
  freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    while(1); // Error
  }

  // Call the actual app
  appMain(argument);
}

void MX_FREERTOS_Init(void) {
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}
