/* USER CODE BEGIN Header */

/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ps2.h"
#include "pca9685.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */
PS2Buttons ps2_data;

float arm_A = 307;
float arm_B = 307;
float arm_C = 307;
float arm_D = 307;
/* USER CODE END PV */

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void GPIOB_LED_Init(void);
void MotorPins_SafeEarly(void);
void Motor_Lock(uint8_t motor_id);
void Motor_SelfTest(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void MotorPins_SafeEarly(void) {
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                             GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gi);

    gi.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gi);
}

void GPIOB_LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

#define MOTOR_PWM_MAX        3599
#define MOTOR_SPEED_MAX      100

int Limit(int speed) {
    if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) return -MOTOR_SPEED_MAX;
    return speed;
}

static void Motor_PWM(TIM_HandleTypeDef *htim, uint32_t pwm_ch, uint32_t pwm_ch_inv,
                      int16_t speed) {
    uint16_t duty;

    speed = (int16_t)Limit(speed);

    duty = (uint16_t)(((speed >= 0) ? speed : -speed) * MOTOR_PWM_MAX / MOTOR_SPEED_MAX);

    if (speed >= 0) {
        __HAL_TIM_SET_COMPARE(htim, pwm_ch, duty);
        __HAL_TIM_SET_COMPARE(htim, pwm_ch_inv, 0);
    } else {
        __HAL_TIM_SET_COMPARE(htim, pwm_ch, 0);
        __HAL_TIM_SET_COMPARE(htim, pwm_ch_inv, duty);
    }
}

void Set_Motor(uint8_t motor_id, int16_t speed) {
    switch (motor_id) {
        case 1:
            Motor_PWM(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, speed);
            break;
        case 2:
            Motor_PWM(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, speed);
            break;
        case 3:
            Motor_PWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, speed);
            break;
        case 4:
            Motor_PWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, speed);
            break;
        default:
            break;
    }
}

#define MOTOR_LOCK_BOTH_HIGH   1

void Motor_Lock(uint8_t motor_id) {
    uint16_t lv = (MOTOR_LOCK_BOTH_HIGH != 0) ? (uint16_t)MOTOR_PWM_MAX : 0u;

    switch (motor_id) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, lv);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, lv);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, lv);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, lv);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, lv);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, lv);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, lv);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, lv);
            break;
        default:
            break;
    }
}

void Motor_SelfTest(void) {
    static const uint8_t seq[4] = { 1, 2, 3, 4 };

    for (;;) {
        for (uint8_t i = 0; i < 4; i++) {

            Set_Motor(seq[i], MOTOR_SPEED_MAX);
            osDelay(2000);

            Set_Motor(seq[i], 0);
            osDelay(800);

            Set_Motor(seq[i], -MOTOR_SPEED_MAX);
            osDelay(2000);

            Set_Motor(seq[i], 0);
            osDelay(800);
        }
    }
}

#define LEFT_SIDE_A     1
#define LEFT_SIDE_B     3
#define RIGHT_SIDE_A    2
#define RIGHT_SIDE_B    4

#define TURN_MODE_PIVOT   1

void Kinematics(int Vx, int Omega) {
    int vL, vR;

#if TURN_MODE_PIVOT

    if (Vx == 0 && Omega != 0) {
        int mag = (Omega < 0) ? -Omega : Omega;
        if (mag > 100) {
            mag = 100;
        }

        if (Omega > 0) {

            Motor_Lock(RIGHT_SIDE_A);
            Motor_Lock(RIGHT_SIDE_B);
            Set_Motor(LEFT_SIDE_A, (int16_t)mag);
            Set_Motor(LEFT_SIDE_B, (int16_t)mag);
        } else {

            Motor_Lock(LEFT_SIDE_A);
            Motor_Lock(LEFT_SIDE_B);
            Set_Motor(RIGHT_SIDE_A, (int16_t)mag);
            Set_Motor(RIGHT_SIDE_B, (int16_t)mag);
        }
        return;
    }
#endif

    if (Vx != 0) {
        int room = 100 - ((Omega < 0) ? -Omega : Omega);
        if (room < 0) {
            room = 0;
        }
        Vx = Vx * room / 100;
    }

    vL = Vx + Omega;
    vR = Vx - Omega;

    Set_Motor(LEFT_SIDE_A, (int16_t)vL);
    Set_Motor(LEFT_SIDE_B, (int16_t)vL);
    Set_Motor(RIGHT_SIDE_A, (int16_t)vR);
    Set_Motor(RIGHT_SIDE_B, (int16_t)vR);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  MotorPins_SafeEarly();

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  hi2c1.Init.ClockSpeed = 50000;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  hi2c1.Instance->CR1 &= ~I2C_CR1_PE;
  hi2c1.Instance->OAR1 &= ~(I2C_OAR1_ADDMODE | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0);
  hi2c1.Instance->CR1 |= I2C_CR1_PE;

  GPIOB_LED_Init();

  /* USER CODE END 2 */

  MX_FREERTOS_Init();

  osKernelStart();

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/* USER CODE END 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}
#endif
