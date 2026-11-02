/* USER CODE BEGIN Header */

/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "ps2.h"
#include "pca9685.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
extern PS2Buttons ps2_data;
extern float arm_A, arm_B, arm_C;

int Limit(int speed);
void Set_Motor(uint8_t motor_id, int16_t speed);
int  deadband(int v, int dead);
void Mecanum_Drive(int Vx, int Vy, int W);
void Mecanum_SelfTest(void);
float limit_angle(float val, float min, float max);

/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN PM */
float limit_angle(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

#define HOME_A             350
#define HOME_B             350
#define HOME_C             400

#define LIM_A_MIN          200
#define LIM_A_MAX          450
#define LIM_B_MIN          114
#define LIM_B_MAX          514
#define LIM_C_MIN          150
#define LIM_C_MAX          500

#define PRESET1_A          340
#define PRESET1_B          120
#define PRESET1_C          255

#define PRESET2_A          340
#define PRESET2_B          350
#define PRESET2_C          450

#define PRESET_MOVE_MS     3000

#define GRAB_CLAW_OPEN     400
#define GRAB_CLAW_CLOSE    255

#define GRAB_CLAW_HOLD     230

#define GRAB2_CLAW_OPEN    340

#define CLAW_SLOW_STEP     1

#define GRAB_DOWN_B        325
#define GRAB_DOWN_C        200

#define GRAB_UP_B          450
#define GRAB_UP_C          250

#define GRAB_OPEN_MS       600
#define GRAB_DOWN_MS       3000
#define GRAB_CLOSE_MS      800
#define GRAB_HOLD_MS       300
#define GRAB_UP_MS         3000

#define GRAB_PAUSE1_MS     1000
#define GRAB_RELEASE_MS    800
#define GRAB_PAUSE2_MS     1000
#define GRAB_RETURN_MS     3000

#define GRAB_MOVE_STEP     8

#define CLAW_AUTO_STEP     4

static uint8_t  home_state = 0;
static uint32_t home_tick  = 0;

static uint8_t  preset_state = 0;
static uint32_t preset_tick  = 0;

static uint8_t grab_at_target(float b, float c) {
    return (uint8_t)((arm_B == b) && (arm_C == c));
}

static void move_toward_step(float *cur, float target, uint16_t step) {
    if (*cur < target) {
        *cur += (float)step;
        if (*cur > target) *cur = target;
    } else if (*cur > target) {
        *cur -= (float)step;
        if (*cur < target) *cur = target;
    }
}

static void grab_move_toward(float *cur, float target) {
    move_toward_step(cur, target, GRAB_MOVE_STEP);
}

static void claw_move_toward(float *cur, float target) {
    move_toward_step(cur, target, CLAW_AUTO_STEP);
}

static uint8_t home_task_auto(uint8_t tri) {
    static uint8_t tri_prev = 0;

    if (tri == 1 && tri_prev == 0 && home_state == 0 && preset_state == 0) {
        home_state = 1;
        home_tick = 0;
    }
    tri_prev = tri;

    if (home_state == 0) {
        return 0;
    }

    home_tick += 20;

    switch (home_state) {
        case 1:
            arm_A = GRAB_CLAW_HOLD;
            grab_move_toward(&arm_B, GRAB_UP_B);
            grab_move_toward(&arm_C, GRAB_UP_C);
            if (grab_at_target((float)GRAB_UP_B, (float)GRAB_UP_C)
                || home_tick >= GRAB_UP_MS) {
                home_state = 2;
                home_tick = 0;
            }
            break;

        case 2:
            arm_A = GRAB_CLAW_HOLD;
            if (home_tick >= GRAB_PAUSE1_MS) {
                home_state = 3;
                home_tick = 0;
            }
            break;

        case 3:
            arm_A = GRAB_CLAW_OPEN;
            if (home_tick >= GRAB_RELEASE_MS) {
                home_state = 4;
                home_tick = 0;
            }
            break;

        case 4:
            arm_A = GRAB_CLAW_OPEN;
            if (home_tick >= GRAB_PAUSE2_MS) {
                home_state = 5;
                home_tick = 0;
            }
            break;

        case 5:
            claw_move_toward(&arm_A, HOME_A);
            grab_move_toward(&arm_B, HOME_B);
            grab_move_toward(&arm_C, HOME_C);
            if ((arm_A == (float)HOME_A && arm_B == (float)HOME_B && arm_C == (float)HOME_C)
                || home_tick >= GRAB_RETURN_MS) {
                home_state = 0;
                home_tick = 0;
            }
            break;

        default:
            home_state = 0;
            break;
    }
    return 1;
}

static uint8_t preset_task_auto(uint8_t l1, uint8_t l2)
{
    static uint8_t l1_prev = 0;
    static uint8_t l2_prev = 0;
    float ta, tb, tc;

    if (preset_state == 0)
    {
        if (l1 == 1 && l1_prev == 0 && home_state == 0)
        {
            preset_state = 1;
            preset_tick  = 0;
        }
        else if (l2 == 1 && l2_prev == 0 && home_state == 0)
        {
            preset_state = 2;
            preset_tick  = 0;
        }
    }
    l1_prev = l1;
    l2_prev = l2;

    if (preset_state == 0)
    {
        return 0;
    }

    preset_tick += 20;

    if (preset_state == 1)
    {
        ta = (float)PRESET1_A;
        tb = (float)PRESET1_B;
        tc = (float)PRESET1_C;
    }
    else
    {
        ta = (float)PRESET2_A;
        tb = (float)PRESET2_B;
        tc = (float)PRESET2_C;
    }

    claw_move_toward(&arm_A, ta);
    grab_move_toward(&arm_B, tb);
    grab_move_toward(&arm_C, tc);

    if ((arm_A == ta && arm_B == tb && arm_C == tc) || preset_tick >= PRESET_MOVE_MS)
    {
        preset_state = 0;
        preset_tick  = 0;
    }

    return 1;
}

#define SERVO_MIRROR_CENTER   307

static uint16_t Servo_Mirror(int16_t logical) {
    int32_t v = (int32_t)(2 * SERVO_MIRROR_CENTER) - (int32_t)logical;

    if (v < 100) {
        v = 100;
    }
    if (v > 512) {
        v = 512;
    }
    return (uint16_t)v;
}

static void Arm_Output(float a, float b, float c) {
    PCA9685_SetPWM(0, (uint16_t)a);
    PCA9685_SetPWM(1, Servo_Mirror((int16_t)b));
    PCA9685_SetPWM(2, Servo_Mirror((int16_t)c));
}
/* USER CODE END PM */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ps2Handle;
osThreadId classesHandle;
osThreadId armHandle;
osThreadId debugHandle;

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void);

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;

}
/* USER CODE END GET_IDLE_TASK_MEMORY */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(ps2, StartTask02, osPriorityHigh, 0, 256);
  ps2Handle = osThreadCreate(osThread(ps2), NULL);

  osThreadDef(classes, StartTask03, osPriorityAboveNormal, 0, 256);
  classesHandle = osThreadCreate(osThread(classes), NULL);

  osThreadDef(arm, StartTask04, osPriorityNormal, 0, 256);
  armHandle = osThreadCreate(osThread(arm), NULL);

  osThreadDef(debug, StartTask05, osPriorityLow, 0, 128);
  debugHandle = osThreadCreate(osThread(debug), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */

/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	PS2_Init(&htim1, &ps2_data);

  for(;;)
  {
    PS2_Update();
    osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */

/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

#if MECANUM_SELF_TEST
  Mecanum_SelfTest();
#endif
  for(;;)
  {
    int Vx, Vy, W;

    taskENTER_CRITICAL();

    Vx = 0; Vy = 0;
    if (ps2_data.UP)    Vx =  100;
    if (ps2_data.DOWN)  Vx = -100;
    if (ps2_data.LEFT)  Vy =  100;
    if (ps2_data.RIGHT) Vy = -100;

    W = ps2_data.RX - 128;
    taskEXIT_CRITICAL();

#define ROT_DEAD   25
#define ROT_GAIN   90
    if (W > -ROT_DEAD && W < ROT_DEAD) {
        W = 0;
    } else {

        W = W * ROT_GAIN / 128;
        if (W >  100) W =  100;
        if (W < -100) W = -100;
    }

    Mecanum_Drive(Vx, Vy, W);

    osDelay(20);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */

/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  osDelay(100);
  PCA9685_Init();
  osDelay(100);

#define ARM_STEP_DIV     12
#define ARM_STEP_MIN     20
#define ARM_STEP_MAX     10

#define ARM_REFRESH_MS   1000

  taskENTER_CRITICAL();
  arm_A = HOME_A;
  arm_B = HOME_B;
  arm_C = HOME_C;
  taskEXIT_CRITICAL();

  Arm_Output((float)HOME_A, (float)HOME_B, (float)HOME_C);

  osDelay(400);
	uint8_t last_r1 = 0;
  uint8_t claw_state = 0;
  uint8_t last_r2 = 0;
  uint8_t claw2_state = 0;
  uint32_t refresh_tick = 0;

  for(;;)
  {

    refresh_tick += 20;
    if (ARM_REFRESH_MS > 0 && refresh_tick >= (uint32_t)ARM_REFRESH_MS) {
      refresh_tick = 0;
      PCA9685_CheckAndRepair();
    }

    uint8_t js_LY, js_RY, js_R1, js_L1, js_L2, js_TRI, js_R2, js_SQUARE, js_CIRCLE;
    taskENTER_CRITICAL();
    js_LY = ps2_data.LY;
    js_RY = ps2_data.RY;
    js_R1 = ps2_data.R1;
    js_L1 = ps2_data.L1;
    js_L2 = ps2_data.L2;
    js_TRI = ps2_data.TRIANGLE;
    js_R2 = ps2_data.R2;
    js_SQUARE = ps2_data.SQUARE;
    js_CIRCLE = ps2_data.CIRCLE;
    taskEXIT_CRITICAL();

    uint8_t auto_running = home_task_auto(js_TRI);
    if (!auto_running) {
      auto_running = preset_task_auto(js_L1, js_L2);
    }
    if (auto_running) {
      arm_A = limit_angle(arm_A, LIM_A_MIN, LIM_A_MAX);
      arm_B = limit_angle(arm_B, LIM_B_MIN, LIM_B_MAX);
      arm_C = limit_angle(arm_C, LIM_C_MIN, LIM_C_MAX);
      Arm_Output(arm_A, arm_B, arm_C);
      osDelay(20);
      continue;
    }

    int LY = js_LY - 128;
    int RY = js_RY - 128;
    int step_LY = 0;
    int step_RY = 0;

    if (LY > ARM_STEP_MIN || LY < -ARM_STEP_MIN) {
      step_LY = LY / ARM_STEP_DIV;
      if (step_LY == 0) {
        step_LY = (LY > 0) ? 1 : -1;
      }
      if (step_LY >  ARM_STEP_MAX) step_LY =  ARM_STEP_MAX;
      if (step_LY < -ARM_STEP_MAX) step_LY = -ARM_STEP_MAX;
    }
    if (RY > ARM_STEP_MIN || RY < -ARM_STEP_MIN) {
      step_RY = RY / ARM_STEP_DIV;
      if (step_RY == 0) {
        step_RY = (RY > 0) ? 1 : -1;
      }
      if (step_RY >  ARM_STEP_MAX) step_RY =  ARM_STEP_MAX;
      if (step_RY < -ARM_STEP_MAX) step_RY = -ARM_STEP_MAX;
    }

    arm_B += step_LY;

    arm_C -= step_RY;

    if (js_R1 == 1 && last_r1 == 0) {
        claw_state = !claw_state;
        if (claw_state == 1) { arm_A = GRAB_CLAW_CLOSE;  }
        else                 { arm_A = GRAB2_CLAW_OPEN;  }
    }
    last_r1 = js_R1;

    if (js_R2 == 1 && last_r2 == 0) {
        claw2_state = !claw2_state;
        if (claw2_state == 1) { arm_A = GRAB_CLAW_CLOSE; }
        else                  { arm_A = GRAB_CLAW_OPEN;  }
    }
    last_r2 = js_R2;

    if (js_SQUARE) {
        arm_A -= CLAW_SLOW_STEP;
        if (arm_A < (float)GRAB_CLAW_CLOSE) {
            arm_A = (float)GRAB_CLAW_CLOSE;
        }
    } else if (js_CIRCLE) {
        arm_A += CLAW_SLOW_STEP;
        if (arm_A > (float)GRAB_CLAW_OPEN) {
            arm_A = (float)GRAB_CLAW_OPEN;
        }
    }

    arm_A = limit_angle(arm_A, LIM_A_MIN, LIM_A_MAX);
    arm_B = limit_angle(arm_B, LIM_B_MIN, LIM_B_MAX);
    arm_C = limit_angle(arm_C, LIM_C_MIN, LIM_C_MAX);

    Arm_Output(arm_A, arm_B, arm_C);

    osDelay(20);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */

/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */

  osDelay(1200);

  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay((pca9685_ok != 0u) ? 400 : 100);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
