/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "icache.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lsm6dsv80x_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SENSOR_BUS hi2c1


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
#define    CNT_FOR_OUTPUT       100

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_motion[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static lsm6dsv80x_filt_settling_mask_t filt_settling_mask;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);


typedef struct
{
  float yaw;
  float pitch;
  float roll;
} euler_angle_t;

typedef struct
{
  float quat_w;
  float quat_x;
  float quat_y;
  float quat_z;
} quaternion_t;
void quaternion_to_euler_angle(quaternion_t *q,euler_angle_t *euler);
float halfPrecisionToFloat(uint16_t h);
static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = lsm6dsv80x_from_f16_to_f32(h);
    return conv.ret;
}

static void sflp2q(float_t quat[4], uint16_t sflp[3])
{
  float_t sumsq = 0;

  quat[0] = npy_half_to_float(sflp[0]);
  quat[1] = npy_half_to_float(sflp[1]);
  quat[2] = npy_half_to_float(sflp[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f) {
    float_t n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("HELLO!\n");
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SA0_GPIO_Port, SA0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	
	lsm6dsv80x_reset_t rst;
  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
//  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
	
	
 /* 读取并检查器件 ID，确认当前 I2C/SPI 设备就是 LSM6DSV80X */
  lsm6dsv80x_device_id_get(&dev_ctx, &whoamI);
  printf("LSM6DSV80X_ID=0x%x,whoamI=0x%x",LSM6DSV80X_ID,whoamI);
  if (whoamI != LSM6DSV80X_ID)
    while (1);   // 如果ID不匹配则停机，说明连线/芯片错误

  /* 恢复默认配置：对所有寄存器进行软件复位，回到出厂状态 */
  lsm6dsv80x_reset_set(&dev_ctx, LSM6DSV80X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv80x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV80X_READY); // 等待复位完成

  /* 使能 BDU (Block Data Update)：在读完所有轴的寄存器前锁定数据，避免数据更新导致不一致 */
  lsm6dsv80x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	
  /* 设置输出数据速率 ODR：
   * 注意：SFLP 融合引擎的 ODR 不能高于输入传感器的 ODR
   */
  lsm6dsv80x_xl_data_rate_set(&dev_ctx, LSM6DSV80X_ODR_AT_120Hz);      // 低g加速度计 120Hz
  lsm6dsv80x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV80X_HG_XL_ODR_AT_480Hz, 1); // 高g加速度计 480Hz，并打开 REGOUT 输出
  lsm6dsv80x_gy_data_rate_set(&dev_ctx, LSM6DSV80X_ODR_AT_120Hz);      // 陀螺仪 120Hz
	

  /* 设置满量程 FS */
  lsm6dsv80x_xl_full_scale_set(&dev_ctx, LSM6DSV80X_16g);      // 低g加速度计 ±16g
  lsm6dsv80x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV80X_80g);   // 高g加速度计 ±80g
  lsm6dsv80x_gy_full_scale_set(&dev_ctx, LSM6DSV80X_4000dps);  // 陀螺仪 ±4000 dps
	
  /* 配置工作模式 */
  lsm6dsv80x_xl_mode_set(&dev_ctx, LSM6DSV80X_XL_HIGH_PERFORMANCE_MD); // 低g加速度计高性能模式
  lsm6dsv80x_gy_mode_set(&dev_ctx, LSM6DSV80X_GY_HIGH_PERFORMANCE_MD); // 陀螺仪高性能模式

  /* 配置 SFLP (Sensor Fusion Low Power) 融合引擎的数据输出速率 */
  lsm6dsv80x_sflp_data_rate_set(&dev_ctx, LSM6DSV80X_SFLP_120Hz);
	
  /* 开启 Game Rotation Vector 融合算法输出（即四元数输出） */
  lsm6dsv80x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
	
  /* 配置陀螺零偏 (gBias)，避免融合计算时因偏置导致姿态漂移 */
  lsm6dsv80x_sflp_gbias_t gbias;
  lsm6dsv80x_all_sources_t all_sources;
  lsm6dsv80x_quaternion_t quaternion;	
  /*
   * 应用层可以在这里加载上一次运行时保存的陀螺零偏值
   * （例如从 Flash 读取），这样能减少姿态解算漂移。
   */
  gbias.gbias_x = 0.0f;
  gbias.gbias_y = 0.0f;
  gbias.gbias_z = 0.0f;
  lsm6dsv80x_sflp_game_gbias_set(&dev_ctx, &gbias);

  /* 定义欧拉角和四元数变量，用于后续计算/输出 */
  euler_angle_t euler;
  quaternion_t quat;
	
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;		
	
	uint8_t data_angular_rate_raw[16]={0};
	data_angular_rate_raw[0]=0xAB;//帧头
	data_angular_rate_raw[1]=0xFD;//源地址
	data_angular_rate_raw[2]=0xFE;//目标地址		
	data_angular_rate_raw[3]=0x03;//功能码ID	
	data_angular_rate_raw[4]=0x08;//数据长度LEN
	data_angular_rate_raw[5]=0x00;//数据长度LEN 8
	data_angular_rate_raw[6]=0x01;//mode = 1	

	data_angular_rate_raw[13]=0x00;//FUSION _STA：融合状态
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// 读取所有状态标志位（包括加速度/陀螺/温度 DRDY 以及嵌入式功能状态）
		lsm6dsv80x_all_sources_get(&dev_ctx, &all_sources);
		// 判断嵌入式功能（SFLP 融合引擎）是否处于可读状态
		if(all_sources.emb_func_stand_by)
		{
			// 从芯片寄存器读取 SFLP 融合四元数结果 (w, x, y, z)
			lsm6dsv80x_sflp_quaternion_get(&dev_ctx, &quaternion);
//			printf("w=%.2f,x=%.2f,y=%.2f,z=%.2f \n",
//									quaternion.quat_w,quaternion.quat_x,
//									quaternion.quat_y,quaternion.quat_z);
			// 对四元数做坐标系映射：
			// 这里根据板子安装方向，将芯片坐标系转换为应用坐标系			
			quat.quat_w = quaternion.quat_w;
			quat.quat_x = - quaternion.quat_y;
			quat.quat_y = quaternion.quat_z;
			quat.quat_z = - quaternion.quat_x;
			// 将四元数转换为欧拉角（Roll/Pitch/Yaw）
			quaternion_to_euler_angle(&quat,&euler);
			// 打印欧拉角（单位：度）
//			printf("1:Roll=%.2f,Pitch=%.2f,Yaw=%.2f \n",
//											euler.roll,euler.pitch,euler.yaw);	
			
			// 定义用于存放欧拉角的整数变量（放大100倍，用于发送）
			int16_t	Roll_int16;
			int16_t	Pitch_int16;					
			int16_t	Yaw_int16;	
			// 将浮点欧拉角转成定点整数，单位：度 × 100
			Roll_int16 = (int16_t)((euler.roll) * 100);
			Pitch_int16 = (int16_t)((euler.pitch) * 100);
			Yaw_int16 = (int16_t)((euler.yaw) * 100 - 18000);
			
//			printf("2:Roll=%d,Pitch=%d,Yaw=%d \n",
//											Roll_int16,Pitch_int16,Yaw_int16);	
			
			
			data_angular_rate_raw[7] = (uint8_t)(Roll_int16 >> 8);//roll
			data_angular_rate_raw[8] = (uint8_t)(Roll_int16 );
			data_angular_rate_raw[9] = (uint8_t)(Pitch_int16 >> 8);//pitch
			data_angular_rate_raw[10] = (uint8_t)(Pitch_int16 );
			data_angular_rate_raw[11] = (uint8_t)(Yaw_int16 >> 8);//yaw
			data_angular_rate_raw[12] = (uint8_t)(Yaw_int16 );
								
			data_angular_rate_raw[13] = 0;
			sumcheck = 0;
			addcheck = 0;
			for(uint16_t i=0; i < 14; i++)
			{
				sumcheck += data_angular_rate_raw[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
				addcheck += sumcheck; //每一字节的求和操作，进行一次 sumcheck 的累加
			}
			data_angular_rate_raw[14]=sumcheck;
			data_angular_rate_raw[15]=addcheck;	
			
			HAL_UART_Transmit(&huart1 , (uint8_t *)&data_angular_rate_raw, 16, 0xFFFF);				

					}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/* USER CODE BEGIN 4 */
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSV80X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSV80X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}



void quaternion_to_euler_angle(quaternion_t *q,euler_angle_t *euler)
{
		if (q->quat_w < 0.0f) {
			q->quat_x *= -1.0f;
			q->quat_y *= -1.0f;
			q->quat_z *= -1.0f;
			q->quat_w *= -1.0f;
		}

		float sqx = q->quat_x * q->quat_x;
		float sqy = q->quat_y * q->quat_y;
		float sqz = q->quat_z * q->quat_z;

		euler->yaw = -atan2f(2.0f * (q->quat_y * q->quat_w + q->quat_x * q->quat_z), 1.0f - 2.0f * (sqy + sqx));
		euler->pitch = -atan2f(2.0f * (q->quat_x * q->quat_y + q->quat_z * q->quat_w), 1.0f - 2.0f * (sqx + sqz));
		euler->roll = -asinf(2.0f * (q->quat_x * q->quat_w - q->quat_y * q->quat_z));

		if (euler->yaw < 0.0f)
		{
			euler->yaw += 2.0f * 3.1415926;
		}	
		
		euler->yaw = 57.29578 * euler->yaw;
		euler->pitch = 57.29578 * euler->pitch;
		euler->roll = 57.29578 * euler->roll;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
