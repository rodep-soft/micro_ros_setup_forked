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
  .stack_size = 10000 * 4,  // 40KB stack
  .priority = (osPriority_t) osPriorityNormal,
};

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void);

void StartDefaultTask(void *argument)
{
  // Debug: 1 quick blink - Task started
  for(int i=0; i<2; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    osDelay(100);
  }
  osDelay(500);
  
  // Setup transport
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    freertos_serial_open,
    freertos_serial_close,
    freertos_serial_write,
    freertos_serial_read);

  // Debug: 3 quick blinks - Transport configured
  for(int i=0; i<3; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    osDelay(100);
  }
  osDelay(500);

  // Setup allocator
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = __freertos_allocate;
  freeRTOS_allocator.deallocate = __freertos_deallocate;
  freeRTOS_allocator.reallocate = __freertos_reallocate;
  freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    // Error: rapid blink forever
    while(1) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      osDelay(50);
    }
  }

  // Debug: 5 quick blinks - Allocator configured
  for(int i=0; i<5; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    osDelay(100);
  }
  osDelay(500);

  // Wait for serial to be ready
  osDelay(2000);

  // Debug: LED ON - About to call appMain
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  osDelay(1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  
  // Call the actual app
  appMain(argument);
  
  // If we get here, appMain returned (shouldn't happen)
  while(1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(200);
  }
}

void MX_FREERTOS_Init(void) {
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}
