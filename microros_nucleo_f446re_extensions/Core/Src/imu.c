/*
 * imu.c
 * BNO055 STM32 HAL Driver for micro-ROS
 */

#include "imu.h"

// 内部ヘルパー関数
static bool BNO055_Write8(BNO055_Handler *dev, bno055_reg_t reg, uint8_t value) {
  if (HAL_I2C_Mem_Write(dev->i2c_handle, dev->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK) {
    return false;
  }
  return true;
}

static uint8_t BNO055_Read8(BNO055_Handler *dev, bno055_reg_t reg) {
  uint8_t value = 0;
  HAL_I2C_Mem_Read(dev->i2c_handle, dev->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
  return value;
}

static bool BNO055_ReadLen(BNO055_Handler *dev, bno055_reg_t reg, uint8_t *buffer, uint8_t len) {
  if (HAL_I2C_Mem_Read(dev->i2c_handle, dev->address, reg, I2C_MEMADD_SIZE_8BIT, buffer, len, 100) != HAL_OK) {
    return false;
  }
  return true;
}

// 構造体の初期化
void BNO055_Init_Struct(BNO055_Handler *dev, I2C_HandleTypeDef *i2c_handle, uint8_t address) {
  dev->i2c_handle = i2c_handle;
  dev->address = address;
  dev->mode = OPERATION_MODE_CONFIG;
}

// センサーの初期化
bool BNO055_Begin(BNO055_Handler *dev, bno055_opmode_t mode) {
  // BNO055のID確認
  uint8_t id = BNO055_Read8(dev, BNO055_CHIP_ID_ADDR);
  if (id != BNO055_ID) {
    HAL_Delay(1000); // 起動直後かもしれないので待つ
    id = BNO055_Read8(dev, BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
      return false;
    }
  }

  // Configモードに切り替え
  BNO055_SetMode(dev, OPERATION_MODE_CONFIG);

  // システムリセット
  BNO055_Write8(dev, BNO055_SYS_TRIGGER_ADDR, 0x20);
  HAL_Delay(30);
  while (BNO055_Read8(dev, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    HAL_Delay(10);
  }
  HAL_Delay(50);

  // Normal Power Mode
  BNO055_Write8(dev, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  HAL_Delay(10);

  BNO055_Write8(dev, BNO055_PAGE_ID_ADDR, 0);

  BNO055_Write8(dev, BNO055_SYS_TRIGGER_ADDR, 0x00);
  HAL_Delay(10);

  // 動作モード設定
  BNO055_SetMode(dev, mode);
  HAL_Delay(20);

  return true;
}

void BNO055_SetMode(BNO055_Handler *dev, bno055_opmode_t mode) {
  dev->mode = mode;
  BNO055_Write8(dev, BNO055_OPR_MODE_ADDR, mode);
  HAL_Delay(30);
}

void BNO055_SetExtCrystalUse(BNO055_Handler *dev, bool usextal) {
  bno055_opmode_t modeback = dev->mode;
  BNO055_SetMode(dev, OPERATION_MODE_CONFIG);
  HAL_Delay(25);
  BNO055_Write8(dev, BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    BNO055_Write8(dev, BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    BNO055_Write8(dev, BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  HAL_Delay(10);
  BNO055_SetMode(dev, modeback);
  HAL_Delay(20);
}

void BNO055_GetSystemStatus(BNO055_Handler *dev, uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error) {
  BNO055_Write8(dev, BNO055_PAGE_ID_ADDR, 0);
  if (system_status) *system_status = BNO055_Read8(dev, BNO055_SYS_STAT_ADDR);
  if (self_test_result) *self_test_result = BNO055_Read8(dev, BNO055_SELFTEST_RESULT_ADDR);
  if (system_error) *system_error = BNO055_Read8(dev, BNO055_SYS_ERR_ADDR);
  HAL_Delay(200);
}

void BNO055_GetCalibration(BNO055_Handler *dev, uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
  uint8_t calData = BNO055_Read8(dev, BNO055_CALIB_STAT_ADDR);
  if (sys) *sys = (calData >> 6) & 0x03;
  if (gyro) *gyro = (calData >> 4) & 0x03;
  if (accel) *accel = (calData >> 2) & 0x03;
  if (mag) *mag = calData & 0x03;
}

int8_t BNO055_GetTemp(BNO055_Handler *dev) {
  return (int8_t)(BNO055_Read8(dev, BNO055_TEMP_ADDR));
}

// ベクトルデータの取得 (加速度、ジャイロ、磁気など)
BNO055_Vector3 BNO055_GetVector(BNO055_Handler *dev, bno055_vector_type_t vector_type) {
  BNO055_Vector3 xyz = {0, 0, 0};
  uint8_t buffer[6];
  int16_t x, y, z;

  if (BNO055_ReadLen(dev, (bno055_reg_t)vector_type, buffer, 6)) {
    x = ((int16_t)buffer[1] << 8) | buffer[0];
    y = ((int16_t)buffer[3] << 8) | buffer[2];
    z = ((int16_t)buffer[5] << 8) | buffer[4];

    // スケール変換 (Adafruitライブラリ準拠)
    switch (vector_type) {
      case VECTOR_MAGNETOMETER:
        // 1uT = 16 LSB
        xyz.x = ((double)x) / 16.0;
        xyz.y = ((double)y) / 16.0;
        xyz.z = ((double)z) / 16.0;
        break;
      case VECTOR_GYROSCOPE:
        // 1dps = 16 LSB
        xyz.x = ((double)x) / 16.0;
        xyz.y = ((double)y) / 16.0;
        xyz.z = ((double)z) / 16.0;
        break;
      case VECTOR_EULER:
        // 1 degree = 16 LSB
        xyz.x = ((double)x) / 16.0;
        xyz.y = ((double)y) / 16.0;
        xyz.z = ((double)z) / 16.0;
        break;
      case VECTOR_ACCELEROMETER:
      case VECTOR_LINEARACCEL:
      case VECTOR_GRAVITY:
        // 1m/s^2 = 100 LSB
        xyz.x = ((double)x) / 100.0;
        xyz.y = ((double)y) / 100.0;
        xyz.z = ((double)z) / 100.0;
        break;
    }
  }
  return xyz;
}

// クォータニオンの取得
BNO055_Quaternion BNO055_GetQuat(BNO055_Handler *dev) {
  BNO055_Quaternion quat = {0, 0, 0, 0};
  uint8_t buffer[8];
  int16_t w, x, y, z;

  if (BNO055_ReadLen(dev, BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8)) {
    w = ((int16_t)buffer[1] << 8) | buffer[0];
    x = ((int16_t)buffer[3] << 8) | buffer[2];
    y = ((int16_t)buffer[5] << 8) | buffer[4];
    z = ((int16_t)buffer[7] << 8) | buffer[6];

    // 1 Quaternion = 2^14 LSB
    const double scale = (1.0 / (1 << 14));
    quat.w = scale * w;
    quat.x = scale * x;
    quat.y = scale * y;
    quat.z = scale * z;
  }
  return quat;
}
