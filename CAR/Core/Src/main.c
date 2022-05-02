/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "I2c-lcd.h"
#include<stdio.h>
#include<stdlib.h>
#include "Kalman.h"
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_SCALE 2
#define LEFT_START_PWM 1600
#define RIGHT_START_PWM 1400
#define LEFT_MAX_PWM 1700
#define RIGHT_MAX_PWM 1300
#define LEFT_STOP_PWM 1500
#define RIGHT_STOP_PWM 1500
#define GAUGE_START_PWM 2500
#define GAUGE_MAX_PWM 500
//#define

#define RESTRICT_PITCH
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.01745329251994329576923690768489

#define MAG0MAX 603
#define MAG0MIN -578
#define MAG1MAX 542
#define MAG1MIN -701
#define MAG2MAX 547
#define MAG2MIN -556


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;
//
struct Kalman kalmanX, kalmanY, kalmanZ;
double roll, pitch, yaw;

double gyroXangle, gyroYangle, gyroZangle;
double kalAngleX, kalAngleY, kalAngleZ;
double compAngleX, compAngleY, compAngleZ;

int magX, magY, magZ;
float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];

uint8_t start_flag = 0;

void MPU6050_Init(void)  // 초기?��
{
	//printf("mpu6050 test\n\r");
	uint8_t check, Data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if(check == 104)
	{
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void MPU6050_Read_Accel (void)
{
	//printf(" test1\n\r");
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;


}
//
void MPU6050_Read_Gyro (void)
{
	//printf(" test2\n\r");
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}
//
void updatePitchRoll() {
// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	roll = atan2(Accel_Y_RAW, Accel_Z_RAW) * RAD_TO_DEG;
	//printf("roll : %f\n\r", roll);
	pitch = atan(-Accel_X_RAW / sqrt(Accel_Y_RAW * Accel_Y_RAW + Accel_Z_RAW * Accel_Z_RAW)) * RAD_TO_DEG;
	//printf("pitch : %f\n\r", pitch);
#else // Eq. 28 and 29
	roll = atan(Accel_Y_RAW / sqrt(Accel_X_RAW * Accel_X_RAW + Accel_Z_RAW * Accel_Z_RAW)) * RAD_TO_DEG;
	pitch = atan2(-Accel_X_RAW, Accel_Z_RAW) * RAD_TO_DEG;
	#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	double rollAngle,pitchAngle,Bfy,Bfx;

     magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
     magZ *= -1;

     magX *= magGain[0];
     magY *= magGain[1];
     magZ *= magGain[2];

     magX -= magOffset[0];
     magY -= magOffset[1];
     magZ -= magOffset[2];


     rollAngle  = kalAngleX * DEG_TO_RAD;
     pitchAngle = kalAngleY * DEG_TO_RAD;

     Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
     Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
     yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

    yaw *= -1;
}

void InitAll()
{
    /* Set Kalman and gyro starting angle */
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
    updatePitchRoll();
    updateYaw();

    //printf("roll %lf\tpitch : %lf\n\r", roll, pitch);
    setAngle(&kalmanX,roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;

    setAngle(&kalmanY,pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;

    setAngle(&kalmanZ,yaw); // And finally yaw
    gyroZangle = yaw;
    compAngleZ = yaw;

//    timer = micros; // Initialize the timer
}

void func()
{
	double gyroXrate, gyroYrate, gyroZrate, dt = 0.01;

	updatePitchRoll();
	gyroXrate = (double)Gyro_X_RAW / 131.0;     // Convert to deg/s
	//printf("gyroXrate : %lf\n\r", gyroXrate);
	gyroYrate = (double)Gyro_Y_RAW / 131.0;     // Convert to deg/s
	//printf("gyroYrate : %lf\n\r", gyroYrate);

#ifdef RESTRICT_PITCH
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		//printf("test 1\n\r");
		setAngle(&kalmanX, roll);
		compAngleX = roll;
		kalAngleX = roll;
		gyroXangle = roll;
	} else{
		//printf("test 2\n\r");
		//printf("roll : %lf\tgyroXrate : %lf\tdt : %lf\n\r", roll, gyroXrate, dt);
		kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
		//printf("kalAngleX : %lf\n\r", kalAngleX);
	}

	if (fabs(kalAngleX) > 0){
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, dt);
		//printf("kalAngleY : %lf\n\r",kalAngleY);

	}
	printf("kalAngleX : %lf, kalAngleY : %lf\n\r", kalAngleX, kalAngleY);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			compAngleY = pitch;
			kalAngleY = pitch;
			gyroYangle = pitch;
		} else
			kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

			if (abs(kalAngleY) > 90)
				gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
				kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	#endif

	updateYaw();
	gyroZrate = Gyro_Z_RAW / 131.0; // Convert to deg/s
	// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
	if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
		setAngle(&kalmanZ, yaw);
		compAngleZ = yaw;
		kalAngleZ = yaw;
		gyroZangle = yaw;
	} else
		kalAngleZ = getAngle(&kalmanZ, yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

	/* Estimate angles using gyro only */
	//gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	//gyroYangle += gyroYrate * dt;
	//gyroZangle += gyroZrate * dt;

	/* Estimate angles using complimentary filter */
	//compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	//compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
	//compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

	//printf("compAngleX : %lf, compAngleY : %lf\n\r", compAngleX, compAngleY);

	// Reset the gyro angles when they has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = kalAngleY;
	if (gyroZangle < -180 || gyroZangle > 180)
		gyroZangle = kalAngleZ;
}


int convertValue(){
	int angle = (int)(Ay*100) / GYRO_SCALE;///GYRO_SCALE;
	if(angle >= -5 && angle <= 5) return 0;
	else if(angle >5 && angle <= 10) return 1;
	else if(angle > 10 && angle <= 15) return 2;
	else if(angle > 15 && angle <= 20) return 3;
	else if(angle > 20 && angle <= 25) return 4;
	else if(angle > 25 && angle <= 30) return 5;
	else if(angle > 35 && angle <= 40) return 6;
	else if(angle > 40 && angle <= 45) return 7;
	else if(angle > 45 && angle <= 50) return 8;
	else if(angle > 50 && angle <= 55) return 9;

	else if(angle >= -10 && angle < -5) return -1;
	else if(angle >= -15 && angle < -10) return -2;
	else if(angle >= -20 && angle < -15) return -3;
	else if(angle >= -25 && angle < -20) return -4;
	else if(angle >= -30 && angle < -25) return -5;
	else if(angle >= -35 && angle < -30) return -6;
	else if(angle >= -40 && angle < -35) return -7;
	else if(angle >= -45 && angle < -40) return -8;
	else if(angle >= -50 && angle < -45) return -9;
	else return 0;

}


void gyroTopwm(){




//	static int pre_value;
//	int ay_value;
//	if(cur_value > -10 && cur_value < 10)
//	{
//		ay_value = cur_value / 3;
//		int delta_val = abs(pre_value - cur_value);
//
//		if(delta_val < 2){
//			ay_value = cur_value;
//			pre_value = cur_value;
//		}
//		else{
//			ay_value = pre_value;
//		}
		//printf("cur_val : %d\tpre_val : %d\tdelta : %d\n\r", cur_value, pre_value, delta_val);
//	}
//
//	else return;


	//int ay_value = (int)(Ay*100) / GYRO_SCALE;///GYRO_SCALE;
	int ay_value = convertValue();
	//int cur_ay_value;
	//printf("ay_value : %d\n\r", ay_value);
	int left_pwm = LEFT_START_PWM + ay_value*20;
	int right_pwm = RIGHT_START_PWM - ay_value*20;
	int gauge_pwm = GAUGE_START_PWM - (left_pwm - LEFT_STOP_PWM)*10;// GAUGE_START_PWM + ay_value*20*5; // wheel : gage = 1 : 5
	//printf("ay_value : %d\n\r", ay_value);
	//printf("left_pwm %d\tright_pwm : %d\tgauge_pwm : %d\n\r", left_pwm, right_pwm, gauge_pwm);

	// uphill
	if(ay_value >= 0 && ay_value < 10){

		if(left_pwm <= LEFT_MAX_PWM && right_pwm >= RIGHT_MAX_PWM && gauge_pwm >= GAUGE_MAX_PWM){
			TIM1->CCR1 = left_pwm;
			TIM1->CCR2 = right_pwm;
			TIM1->CCR3 = gauge_pwm;
			printf("left_pwm %d\tright_pwm : %d\tgauge_pwm : %d\n\r", left_pwm, right_pwm, gauge_pwm);
			//printf("ay_value : %d, left_pwm : %ld, gage_pwm : %ld\n\r",ay_value, TIM1->CCR1, TIM1->CCR3);
		}
		else{
			TIM1->CCR1 = LEFT_MAX_PWM;
			TIM1->CCR2 = RIGHT_MAX_PWM;
			TIM1->CCR3 = GAUGE_MAX_PWM;
			//printf("left_pwm : %ld, gage_pwm : %ld\n\r",TIM1->CCR1, TIM1->CCR3);
			//printf("left_pwm : %d, gage_pwm : %d\n\r",left_pwm, gauge_pwm);
		}
		//printf("left_pwm : %d, gage_pwm : %d\n\r",left_pwm, gauge_pwm);

		//printf("ay_val : %d, left_pwm : %d, right_pwm : %d gage_pwm : %d\n\r", ay_value, left_pwm, right_pwm, gauge_pwm);
	}
	// downhill
	else if(ay_value < 0 && ay_value > -10){
		if(left_pwm > LEFT_STOP_PWM && right_pwm < RIGHT_STOP_PWM){
			TIM1->CCR1 = left_pwm;
			TIM1->CCR2 = right_pwm;
			TIM1->CCR3 = gauge_pwm;
			//printf("ay_value : %d, left_pwm : %ld, gage_pwm : %ld\n\r",ay_value, TIM1->CCR1, TIM1->CCR3);

		}
		//printf("ay_val : %d, left_pwm : %d, right_pwm : %d\n\r", ay_value, left_pwm, right_pwm);

	}

	//printf("ay_val : %d, left_pwm : %d, right_pwm : %d\n\r", ay_value, left_pwm, right_pwm);
	//return left_pwm;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3);
  HAL_Delay(200);
  lcd_init();
  MPU6050_Init();
  //Init(&kalmanX);
  //Init(&kalmanY);
  //InitAll();

  lcd_send_string("initialized");

  HAL_Delay(1000);

  lcd_clear();

  lcd_send_cmd(0x80|0x5A);
  lcd_send_string("MPU6050");
  //HAL_Delay(500);

  char buf[4];

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
	//func();
	//printf("ay : %d\n\r", (int)(Ay*100.0));

	//printf("angle Y : %d\n\r", convertValue());


	  lcd_send_cmd (0x80|0x00);  // goto 1,1
	  	  lcd_send_string ("smart car");
//	  	  //sprintf (buf, "%.2f", Ax);
//	  	  //lcd_send_string (buf);
//	  	  //lcd_send_string ("g ");
//
	  	  lcd_send_cmd (0x80|0x40);  // goto 2,1
	  	  lcd_send_string ("Angle=");
//	  	  //sprintf (buf, "%.2f", Ay);

	  	  sprintf (buf, " %d", (int)(Ay*100));
	  	  lcd_send_string (buf);
	  	lcd_send_string (" ");
	  	  //printf("buf : %s\n\r", buf);
	  	  //lcd_send_string ("º ");

//	  	  lcd_send_cmd (0x80|0x14);  // goto 3,1
//	  	  lcd_send_string ("Az=");
//	  	  sprintf (buf, "%.2f", Az);
//	  	  lcd_send_string (buf);
//	  	  lcd_send_string ("g ");

//	  	  lcd_send_cmd (0x80|0x0A);  // goto 1,11
//	  	  lcd_send_string ("Gx=");
//	  	  sprintf (buf, "%.2f", Gx);
//
//	  	  lcd_send_string (buf);
//
//	  	  lcd_send_cmd (0x80|0x4A);  // goto 2,11
//	  	  lcd_send_string ("Gy=");
//	  	  sprintf (buf, "%.2f", Gy);
//	  	//printf("Gy : %f\n\r", Gy);
//	  	  lcd_send_string (buf);

//	  	  lcd_send_cmd (0x80|0x1E);  // goto 3,11
//	  	  lcd_send_string ("Gz=");
//	  	  sprintf (buf, "%.2f", Gz);
//	  	  lcd_send_string (buf);

	//printf("kal_angle_X : %lf, kal_angle_Y : %lf\n\r", kalAngleX, kalAngleY);

	if(start_flag){
		gyroTopwm();
	}

	HAL_Delay(250);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2000;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		TIM1->CCR1 = 1600; // left wheel
		TIM1->CCR2 = 1400; // right wheel
		start_flag = 1;
	}



//	switch(GPIO_Pin){
//		case GPIO_PIN_0 : // accel
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
//			if(TIM1->CCR1 == 1310 && TIM1->CCR2 == 1690 && TIM1->CCR3 == 1000) break;
//			else{
//				TIM1->CCR1 -= 10;
//				TIM1->CCR2 += 10;
//				TIM1->CCR3 -= 25;
//			}
//			printf("sw2 -> CCR1 : %ld, CCR2 : %ld\n\r", TIM1->CCR1, TIM1->CCR2);
//			break;
//		case GPIO_PIN_1 : // decel
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
//			if(TIM1->CCR1 == 1500 && TIM1->CCR2 == 1500 && TIM1->CCR3 == 2000) break;
//			else{
//				TIM1->CCR1 += 10;
//				TIM1->CCR2 -= 10;
//				TIM1->CCR3 += 25;
//			}
//			printf("sw3 -> CCR1 : %ld, CCR2 : %ld\n\r", TIM1->CCR1, TIM1->CCR2);
//			break;
//		case GPIO_PIN_10 : // decel car duty : 25%
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
////						if(TIM1->CCR1 == 1310 && TIM1->CCR2 == 1690) break;
////						else{
////							TIM1->CCR1 -= 10;
////							TIM1->CCR2 += 10;
////						}
//						//TIM1->CCR1 = (uint32_t)1400;
//						//TIM1->CCR2 = (uint32_t)1600;
//			break;
//		default :
//			break;
//	}
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

#ifdef  USE_FULL_ASSERT
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
