/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  * www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "allocators.h"
#include "cmsis_os2.h"
#include <usart.h>
#include <string.h>
#include <math.h>

// micro-ROS headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <ucdr/microcdr.h>
#include <uxr/client/client.h>
#include <rmw_microros/rmw_microros.h> 
#include <microros_transports.h> 

// Sensor & Driver headers
#include "imu.h"
#include <sensor_msgs/msg/imu.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c1; // main.c で定義されたI2Cハンドル

// BNO055 & micro-ROS Entities
BNO055_Handler bno;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

// Node & Support
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1500 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void bno055_imu_task(void *argument); // IMUタスクのプロトタイプ宣言
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); // Debug LED
  bool availableNetwork = false;

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM 
  availableNetwork = true; 
  rmw_uros_set_custom_transport( 
    true, 
    (void *) &huart2, 
    freertos_serial_open, 
    freertos_serial_close, 
    freertos_serial_write, 
    freertos_serial_read); 
#elif defined(RMW_UXRCE_TRANSPORT_UDP) 
  printf("Ethernet Initialization\r\n");

  // Waiting for an IP
  printf("Waiting for IP\r\n");
  int retries = 0;
  while (gnetif.ip_addr.addr == 0 && retries < 10) {
    osDelay(500);
    retries++;
  };

  availableNetwork = (gnetif.ip_addr.addr != 0);
  if (availableNetwork) {
    printf("IP: %s\r\n", ip4addr_ntoa(&gnetif.ip_addr));
  } else {
    printf("Impossible to retrieve an IP\n");
  }
#endif

  // Launch app thread when IP configured
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = __freertos_allocate;
  freeRTOS_allocator.deallocate = __freertos_deallocate;
  freeRTOS_allocator.reallocate = __freertos_reallocate;
  freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // --- タスク生成の変更点 ---
  osThreadAttr_t attributes;
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "microROS_imu";
  attributes.stack_size = 5 * 3000;
  attributes.priority = (osPriority_t)osPriorityNormal1;

  // ここで appMain ではなく、独自の bno055_imu_task を呼び出します
  if (availableNetwork) {
      osThreadNew(bno055_imu_task, NULL, &attributes);
  }

  osDelay(500);
  char ptrTaskList[500];
  vTaskList(ptrTaskList);
  printf("**********************************\n");
  printf("Task  State    Prio     Stack     Num\n");
  printf("**********************************\n");
  printf(ptrTaskList);
  printf("**********************************\n");

  // メインループ: 通信状態に応じてLEDを点滅させるだけ
  TaskHandle_t xHandle;
  xHandle = xTaskGetHandle("microROS_imu");

  while (1) 
  {
    if (xHandle != NULL && eTaskGetState(xHandle) != eSuspended && availableNetwork)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      osDelay(150);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      osDelay(500);
    } 
    else 
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      osDelay(1000);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      osDelay(1000);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief BNO055 IMU 読み取り & micro-ROS Publish タスク
 */
void bno055_imu_task(void *argument)
{
    // 1. micro-ROS 初期化
    allocator = rcl_get_default_allocator();

    // サポート構造体の初期化
    rclc_support_init(&support, 0, NULL, &allocator);

    // ノードの作成 ("stm32_imu_node")
    rclc_node_init_default(&node, "stm32_imu_node", "", &support);

    // Publisherの作成 ("/imu/data", sensor_msgs/Imu)
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );

    // Executorの作成 (Subscriptionがない場合は必須ではないが、spin用に作っておくのが定石)
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // 2. BNO055 センサーの初期化
    BNO055_Init_Struct(&bno, &hi2c1, BNO055_ADDRESS_A);

    // 接続待ちループ
    bool sensor_ready = false;
    while(!sensor_ready) {
        if(BNO055_Begin(&bno, OPERATION_MODE_NDOF)) {
            sensor_ready = true;
        } else {
            // 初期化失敗時は少し待ってリトライ
            osDelay(1000); 
        }
    }

    // 外部水晶の使用 (精度向上)
    BNO055_SetExtCrystalUse(&bno, true);

    // メッセージのフレームID設定
    imu_msg.header.frame_id.data = "imu_link";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

    // 定数: Degree per Second -> Radian per Second 変換用
    const double dps_to_rads = 0.01745329252;

    // 3. メイン処理ループ
    for(;;)
    {
        // ----------------------------------------
        // A. Orientation (Quaternion)
        // ----------------------------------------
        BNO055_Quaternion q = BNO055_GetQuat(&bno);
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;

        // ----------------------------------------
        // B. Angular Velocity (Gyroscope)
        // ----------------------------------------
        BNO055_Vector3 gyro = BNO055_GetVector(&bno, VECTOR_GYROSCOPE);
        imu_msg.angular_velocity.x = gyro.x * dps_to_rads;
        imu_msg.angular_velocity.y = gyro.y * dps_to_rads;
        imu_msg.angular_velocity.z = gyro.z * dps_to_rads;

        // ----------------------------------------
        // C. Linear Acceleration (Gravity removed)
        // ----------------------------------------
        BNO055_Vector3 accel = BNO055_GetVector(&bno, VECTOR_LINEARACCEL);
        imu_msg.linear_acceleration.x = accel.x;
        imu_msg.linear_acceleration.y = accel.y;
        imu_msg.linear_acceleration.z = accel.z;

        // ----------------------------------------
        // データ配信 (Publish)
        // ----------------------------------------
        rcl_ret_t rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
        if (rc != RCL_RET_OK) {
            // エラーハンドリング (必要なら)
        }

        // Executor処理 (TimerやSubがあればここで処理される)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // ループ周期 (100ms = 10Hz)
        osDelay(100);
    }
}

/* USER CODE END Application */

