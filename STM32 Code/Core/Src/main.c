/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
#include "MPU6050.h"
//#include "MPU6050I2C.h"

#include "inverse_kinematics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MOTOR_MIN_DC 5.24
#define MOTOR_MAX_DC 24.76

#define CMD_MAX_LEN 10
#define RX_BUFF_LEN CMD_MAX_LEN
#define CMD_BUFF_LEN CMD_MAX_LEN*2
#define NUM_OF_MOTORS 6
#define ACC_THRESHOLD 80
#define GYRO_THRESHOLD 5

#define ACC_I2C_THRESHOLD 0.8
#define GYRO_I2C_THRESHOLD 5

#define FLEX_LOW_THRESHOLD 5
#define FLEX_HIGH_THRESHOLD 5

#define ALPHA 0
#define BETA 1
#define ZETA 2
#define GAMMA 3



#define BASE_MOTOR_NUM GAMMA
#define SHOULDER_MOTOR_NUM ALPHA
#define ELBOW_MOTOR_NUM BETA
#define WRIST_MOTOR_NUM ZETA
#define GRIPPER_MOTOR_NUM 5


#define END_EFF_DEFAULT_POSITION {0, 0, LINK_1_LEN+LINK_1_LEN+LINK_1_LEN}
#define ARM_DEFAULT_ANGLES {M_PI/2, M_PI/2,M_PI/2,M_PI/2}

#define MAX_POSITION_INC 0.05

#define ORIENTATAION_NEUTRAL 0
#define ORIENTATAION_X_PLUS 1
#define ORIENTATAION_X_NEG 2
#define ORIENTATAION_Y_PLUS 3
#define ORIENTATAION_Y_NEG 4
#define ORIENTATAION_Z_PLUS 5
#define ORIENTATAION_Z_NEG 6

#define ARM_STATIONARY 0
#define ARM_MOVING 0


typedef enum
{
  ACCELERATION,
  ANGULAR_SPEED
} ACC_DataType;


typedef struct
{
	volatile uint32_t *ccr;
	volatile double cur_DC;
	double min_duty_cycle;
	double max_duty_cycle;
	double offset;
}MotorTypedef;

typedef struct
{
	volatile uint8_t *buff;
	volatile uint16_t curr_pos;
	uint16_t read_pos;
	volatile uint8_t buffer_full;
	volatile uint8_t buff_init;
	volatile uint8_t catch_up;

}CommandBuffTypedef;

typedef struct
{
	float accel[3];
	float gyro[3];
	float flex[2];
}SensorsStateDefType;

typedef struct
{
	uint8_t orientation;
	uint8_t arm_moving;
	double eef_curr_pos[3];

}SystemStateDefType;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_buff[RX_BUFF_LEN];

volatile uint8_t command_buffer[CMD_BUFF_LEN];

CommandBuffTypedef cmd_buff =  {command_buffer, 0, 0, 0, 1, 1};

volatile double motor_angles[NUM_OF_MOTORS];

volatile uint8_t EXTI4_FLAG = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void move_arm(MotorTypedef *motors, SystemStateDefType *system_state);
void run_trajectory(double *trajectory, uint8_t steps, double time);
void putchar_(char c);
void set_duty_cycle(MotorTypedef *motor, double duty_cycle);
HAL_StatusTypeDef process_cmds(CommandBuffTypedef *Command_Buff, MotorTypedef *motors, SystemStateDefType *system_state);
void process_motor_command(uint8_t *command,  MotorTypedef *motors);
void process_accelerometer_command(uint8_t *command);
int get_hand_orientation(volatile float acceleration[], volatile float ang_speed[], volatile float flex_read[]);
void set_end_effector_position(MotorTypedef motors[], double position[]);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void move_arm(MotorTypedef *motors, SystemStateDefType *system_state){
//
//	if (system_state->orientation!= ORIENTATAION_NEUTRAL){
//		double eef_final_pos[3] = {system_state->eef_curr_pos[0], system_state->eef_curr_pos[1], system_state->eef_curr_pos[2]};
//		switch (system_state->orientation){
//			case ORIENTATAION_Y_PLUS:
//				eef_final_pos[1] = system_state->eef_curr_pos[1] + MAX_POSITION_INC;
//				break;
//			case ORIENTATAION_Y_NEG:
//				eef_final_pos[1] = system_state->eef_curr_pos[1] - MAX_POSITION_INC;
//				break;
//			case ORIENTATAION_X_PLUS:
//				eef_final_pos[0] = system_state->eef_curr_pos[0] + MAX_POSITION_INC;
//				break;
//			case ORIENTATAION_X_NEG:
//				eef_final_pos[0] = system_state->eef_curr_pos[0] - MAX_POSITION_INC;
//				break;
//			case ORIENTATAION_Z_PLUS:
//				eef_final_pos[2] = system_state->eef_curr_pos[2] + MAX_POSITION_INC;
//				break;
//			case ORIENTATAION_Z_NEG:
//				eef_final_pos[2] = system_state->eef_curr_pos[2] - MAX_POSITION_INC;
//				break;
//			default:
//				break;
//		}
//		printf_("Position [%0.2f][%0.2f][%0.2f] \n", eef_final_pos[0],eef_final_pos[1],eef_final_pos[2]);
//
//		double alpha_f, beta_f, zeta_f, gamma_f;
//		calculate_angles(eef_final_pos, &alpha_f, &beta_f, &zeta_f, &gamma_f);
//
//
//		if(check_angle_validty(alpha_f, beta_f, zeta_f, gamma_f) == 0){
//			uint8_t steps = 10;
//			double time_period = 0.01;
//			uint32_t delay = time_period*1000/steps;
//			double angle_trajectory[4][steps+1];
//			calculate_trajectory(system_state->eef_curr_pos, eef_final_pos, 0, 0, time_period, steps, (double*)angle_trajectory);
//			//Run the Trajectory
//			for(uint8_t i = 0; i<=steps; i++ ){
//				printf_("alpha: %0f, beta: %f, zeta: %f, gamma: %f\n", angle_trajectory[ALPHA][i], angle_trajectory[BETA][i],angle_trajectory[ZETA][i], angle_trajectory[GAMMA][i]);
//				set_duty_cycle(&motors[BASE_MOTOR_NUM],angle_trajectory[BASE_MOTOR_NUM][i] + BASE_MOTOR_OFFSET_RAD);
//				set_duty_cycle(&motors[SHOULDER_MOTOR_NUM],angle_trajectory[SHOULDER_MOTOR_NUM][i] +SHOULDER_MOTOR_OFFSET_RAD);
//				set_duty_cycle(&motors[ELBOW_MOTOR_NUM],angle_trajectory[ELBOW_MOTOR_NUM][i]+ELBOW_MOTOR_OFFSET_RAD);
//				set_duty_cycle(&motors[WRIST_MOTOR_NUM],angle_trajectory[WRIST_MOTOR_NUM][i]+WRIST_MOTOR_OFFSET_RAD);
//				HAL_Delay(delay);
//			}
//			system_state->eef_curr_pos[0] = eef_final_pos[0];
//			system_state->eef_curr_pos[1] = eef_final_pos[1];
//			system_state->eef_curr_pos[2] = eef_final_pos[2];
//		}
//		else{
//			printf_("Position [%0.2f][%0.2f][%0.2f] not reachable \n", eef_final_pos[0],eef_final_pos[1],eef_final_pos[2]);
//		}
//	}
//}

void move_arm(MotorTypedef *motors, SystemStateDefType *system_state){


	if (system_state->orientation!= ORIENTATAION_NEUTRAL){

		double eef_final_pos[3] = {system_state->eef_curr_pos[0], system_state->eef_curr_pos[1], system_state->eef_curr_pos[2]};

		uint8_t axis = 0;
		int8_t direction = 0;

//		double eef_final_pos[3];
		switch (system_state->orientation){
			case ORIENTATAION_Y_PLUS:
//				eef_final_pos[1] = system_state->eef_curr_pos[1] + 5;
				axis = 1;
				direction = +1;
				break;
			case ORIENTATAION_Y_NEG:
//				eef_final_pos[1] = system_state->eef_curr_pos[1] - 5;
				axis = 1;
				direction = -1;
				break;
			case ORIENTATAION_X_PLUS:
//				eef_final_pos[0] = system_state->eef_curr_pos[0] + 5;
				axis = 0;
				direction = +1;
				break;
			case ORIENTATAION_X_NEG:
//				eef_final_pos[0] = system_state->eef_curr_pos[0] - 5;
				axis = 0;
				direction = -1;
				break;
			case ORIENTATAION_Z_PLUS:
//				eef_final_pos[2] = system_state->eef_curr_pos[2] + 5;
				axis = 2;
				direction = +1;
				break;
			case ORIENTATAION_Z_NEG:
//				eef_final_pos[2] = system_state->eef_curr_pos[2] - 5;
				axis = 2;
				direction = -1;
				break;
			default:
				break;

		}

		printf_("Orientation:%d\n",system_state->orientation );

		double alpha_f, beta_f, zeta_f, gamma_f;
		double increment = 0.01;
		for (uint8_t i = 1;i<=50; i++){
			eef_final_pos[axis] += ((double)direction)*increment;
//			printf_("dir - %d, dir*inc - %f\n",direction, ((double)direction)*increment );
			calculate_angles(eef_final_pos, &alpha_f, &beta_f, &zeta_f, &gamma_f);
			if(check_angle_validty(alpha_f, beta_f, zeta_f, gamma_f) == 0){
				system_state->eef_curr_pos[axis] = eef_final_pos[axis];
//				printf_("Position [%0.2f][%0.2f][%0.2f]\n", system_state->eef_curr_pos[0],system_state->eef_curr_pos[1],system_state->eef_curr_pos[2]);
				set_duty_cycle(&motors[BASE_MOTOR_NUM],gamma_f + BASE_MOTOR_OFFSET_RAD);
				set_duty_cycle(&motors[SHOULDER_MOTOR_NUM],alpha_f + SHOULDER_MOTOR_OFFSET_RAD);
				set_duty_cycle(&motors[ELBOW_MOTOR_NUM],beta_f + ELBOW_MOTOR_OFFSET_RAD);
				set_duty_cycle(&motors[WRIST_MOTOR_NUM],zeta_f + WRIST_MOTOR_OFFSET_RAD);
				HAL_Delay(5);
			}
			else{
//				printf_("Position [%0.2f][%0.2f][%0.2f] not reachable \n", eef_final_pos[0],eef_final_pos[1],eef_final_pos[2]);
				break;
			}
		}
//		printf_("Position [%0.2f][%0.2f][%0.2f]\n", system_state->eef_curr_pos[0],system_state->eef_curr_pos[1],system_state->eef_curr_pos[2]);
	}
}



void set_duty_cycle(MotorTypedef *motor, double angle){
	// Angle(Radians)
	double duty_cycle = (double)(motor->min_duty_cycle + ((angle)*(motor->max_duty_cycle - motor->min_duty_cycle)/M_PI));
	*(motor->ccr) = (uint32_t)((duty_cycle * (double)(0xFFFF))/100);
	motor->cur_DC = duty_cycle;
}

//int set_motor_angle(MotorTypedef *motor, double angle){
//	// Angle(Radians)
//	if ((angle<(M_PI+motor->offset))&&(angle>(0+motor->offset))){
//		double duty_cycle = (double)(motor->min_duty_cycle + ((angle)*(motor->max_duty_cycle - motor->min_duty_cycle)/M_PI));
//		*(motor->ccr) = (uint32_t)((duty_cycle * (double)(0xFFFF))/100);
//		motor->cur_DC = duty_cycle;
//		return 0;
//	}
//	return 1;
//}


void process_motor_command(uint8_t *command, MotorTypedef *motors){
	uint8_t motor = command[1] - '0'; // convert the second letter to an integer
	char *ptr;

	if (motor >= 0 && motor < NUM_OF_MOTORS){
	  float angle = strtof((char*)command + 3, &ptr); // convert the rest of the string to a double

	  if (angle >= 0 && angle <= 190){

		set_duty_cycle(&motors[motor], (angle*M_PI)/180);
		printf_("Motor %d set to %.2f degrees.\n", motor, angle); // print the result

	  }

	  else{
		printf_("Invalid angle.\n"); // print an error message
	  }
	}

	else{
	  printf_("Invalid motor number.\n"); // print an error message
	}

}

int get_hand_orientation(volatile float acceleration[], volatile float ang_speed[], volatile float flex_read[]){

	if ((abs(ang_speed[0])>GYRO_THRESHOLD) || (abs(ang_speed[0])>GYRO_THRESHOLD) ||(abs(ang_speed[0])>GYRO_THRESHOLD)){
		printf_("Hand Shaky");
		return ORIENTATAION_NEUTRAL;
	}
	else{

		if(acceleration[0]>ACC_THRESHOLD){
			printf_("Backward\n");
			return ORIENTATAION_Y_NEG;
		}

		else if(acceleration[0]<-ACC_THRESHOLD){
			printf_("Forward\n");
			return ORIENTATAION_Y_PLUS;
		}

		else if(acceleration[1]<-ACC_THRESHOLD){
			printf_("Left\n");
			return ORIENTATAION_X_PLUS;
		}

		else if(acceleration[1]>ACC_THRESHOLD){
			printf_("Right\n");
			return ORIENTATAION_X_NEG;
		}

		else if(acceleration[2]<-ACC_THRESHOLD){
			printf_("Down\n");
			return ORIENTATAION_Z_NEG;
		}

		else if(acceleration[2]>ACC_THRESHOLD){
			printf_("Neutral\n");
			return ORIENTATAION_NEUTRAL;
		}
		else{
			printf_("Invalid Orientation\n");
			return ORIENTATAION_NEUTRAL;
		}

	}
}


void process_accelerometer_command(uint8_t *command){

	uint8_t type = command[2]-'0';

	float data[3];

	if(type == ACCELERATION){
		MPU6050_Read_Accel(&data[0], &data[1], &data[2]);
	}
	else if (type == ANGULAR_SPEED){
		MPU6050_Read_Gyro(&data[0], &data[1], &data[2]);

	}

	else{
		printf_("Accelerometer Command Error");
	}

}


void save_cmd(UART_HandleTypeDef *huart){

	uint8_t buff_init_copy = cmd_buff.buff_init;

	for(uint8_t i = 0; i<RX_BUFF_LEN; i++){

		if(cmd_buff.buffer_full==1){
				printf_("Buffer Full\n");
				return;
			}

		else{
			if(cmd_buff.curr_pos == cmd_buff.read_pos){

				if(!(cmd_buff.buff_init || cmd_buff.catch_up)){
					cmd_buff.buffer_full = 1;
					printf_("Buffer Full\n");
					return;
				}
			}
		}


		cmd_buff.buff_init = 0;
		cmd_buff.buff[cmd_buff.curr_pos] = rx_buff[i];

		uint8_t end_of_message = 0;

		if(cmd_buff.buff[cmd_buff.curr_pos] == '$'){
			if(i != RX_BUFF_LEN -1)
			{
				HAL_UART_DMAStop(&huart5);
				cmd_buff.curr_pos = cmd_buff.curr_pos -(i);
				if (buff_init_copy == 1){
					cmd_buff.buff_init = 1;
				}

				printf_("Invalid Command\n");
				break;
			}

			end_of_message = 1;
		}


		if(cmd_buff.curr_pos==CMD_BUFF_LEN-1){
			cmd_buff.curr_pos=0;
		}
		else{
			cmd_buff.curr_pos+=1;
		}

		if((i == RX_BUFF_LEN -1) && (end_of_message == 0) ){
			HAL_UART_DMAStop(&huart5);
			cmd_buff.curr_pos = cmd_buff.curr_pos -(i+1);
			if (buff_init_copy == 1){
				cmd_buff.buff_init = 1;
			}

			printf_("Invalid Command\n");
			break;
		}

		if(end_of_message == 1){
//			HAL_UART_Transmit(huart, rx_buff, i+1, HAL_MAX_DELAY);
			cmd_buff.catch_up = 0;
			break;
		}
	}
}

HAL_StatusTypeDef process_cmds(CommandBuffTypedef *Command_Buff, MotorTypedef *motors, SystemStateDefType *system_state){
	if(Command_Buff->read_pos == Command_Buff->curr_pos){
		if ((cmd_buff.buff_init || cmd_buff.catch_up)){
			return HAL_ERROR;
		}
	}

	while(Command_Buff->catch_up != 1){
		uint8_t cmd[CMD_MAX_LEN];
		uint8_t cmd_len = 0;
		for(uint8_t i = 0; i<CMD_MAX_LEN; i++){
			cmd_len = i+1;
			if (Command_Buff->buff[Command_Buff->read_pos]!='$'){
				cmd[i] = Command_Buff->buff[Command_Buff->read_pos];
				if(Command_Buff->read_pos == CMD_BUFF_LEN-1){
					Command_Buff->read_pos = 0;
				}
				else{
					Command_Buff->read_pos+=1;
				}
			}
			else{
				cmd[i] = '$';
				if(Command_Buff->read_pos == CMD_BUFF_LEN-1){
					Command_Buff->read_pos = 0;
				}
				else{
					Command_Buff->read_pos+=1;
				}
				break;
			}
		}

		if(Command_Buff->read_pos == Command_Buff->curr_pos){
			cmd_buff.catch_up = 1;
		}

		if(Command_Buff->buffer_full == 1){
			Command_Buff->buffer_full = 0;
		}

		uint8_t new_line = '\n';
		HAL_UART_Transmit(&huart5, cmd, cmd_len, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart5, &new_line, 1, HAL_MAX_DELAY);


		uint8_t a = cmd[0];
		switch (a){
		case 'M':
			process_motor_command(cmd, motors);
			break;

		case 'A':
			process_accelerometer_command(cmd);
			break;

		case 'O':
			system_state->orientation = cmd[2]-'0';
			break;

		default:
			return HAL_ERROR;
		}
	}

	return HAL_OK;

}



void putchar_(char c) {
    // Transmit the string
    if (HAL_UART_Transmit(&huart5, (uint8_t*)&c, 1, HAL_MAX_DELAY)!= HAL_OK)
    {
    	Error_Handler();
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	save_cmd(huart);
    HAL_UART_Receive_DMA(huart, rx_buff, 10);

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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_PWM_Init(&htim9);

  HAL_UART_Receive_DMA(&huart5, rx_buff, 10);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

//  char buf[4];

//  MPU6050_init();
//  MPU6050_Initialization();

  MotorTypedef motors[NUM_OF_MOTORS] = {
  		  {&(htim2.Instance->CCR1), 0.0, MOTOR_MIN_DC, MOTOR_MAX_DC},
  		  {&(htim2.Instance->CCR2), 0.0, MOTOR_MIN_DC, MOTOR_MAX_DC},
  		  {&(htim3.Instance->CCR1), 0.0, MOTOR_MIN_DC, MOTOR_MAX_DC},
  		  {&(htim3.Instance->CCR3), 0.0, MOTOR_MIN_DC, MOTOR_MAX_DC},
  		  {&(htim9.Instance->CCR1), 0.0, MOTOR_MIN_DC, MOTOR_MAX_DC},
  		  {&(htim9.Instance->CCR2), 0.0, MOTOR_MIN_DC, MOTOR_MAX_DC}
  };



  // Setting the arm to default position
  set_duty_cycle(&motors[BASE_MOTOR_NUM], ((90 + BASE_MOTOR_OFFSET)*M_PI)/180);
  set_duty_cycle(&motors[SHOULDER_MOTOR_NUM], ((91 +SHOULDER_MOTOR_OFFSET)*M_PI)/180);
  set_duty_cycle(&motors[ELBOW_MOTOR_NUM], ((99 + ELBOW_MOTOR_OFFSET)*M_PI)/180);
  set_duty_cycle(&motors[WRIST_MOTOR_NUM], ((78 + WRIST_MOTOR_OFFSET)*M_PI)/180);

//  SensorsStateDefType sensors;
  SystemStateDefType system_state;
  system_state.orientation = ORIENTATAION_NEUTRAL;
  system_state.eef_curr_pos[0] = 0.1;
  system_state.eef_curr_pos[1] = 14;
  system_state.eef_curr_pos[2] = 4;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    process_cmds(&cmd_buff, motors, &system_state);
    move_arm(motors, &system_state);
//    printf_("Orientation - %d\n",system_state.orientation);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	EXTI4_FLAG = 1;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

//  switch(err_type):
//				  case 0:
//					  printf_("")
//
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
