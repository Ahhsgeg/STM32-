/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private define ------------------------------------------------------------*/
#define DIR_PIN GPIO_PIN_14   // 方向控制引脚 PF14
#define PUL_PIN GPIO_PIN_5    // 步进脉冲引脚 PI5 (TIM8_CH1)
#define EN_PIN GPIO_PIN_15    // 使能引脚     PF15
#define DIR_PORT GPIOF
#define PUL_PORT GPIOI
#define EN_PORT GPIOF

#define LED_PIN GPIO_PIN_0    // LED控制引脚 PE0
#define LED_PORT GPIOE

#define KEY0_PIN GPIO_PIN_2   // PE2
#define KEY1_PIN GPIO_PIN_3   // PE3
#define KEY2_PIN GPIO_PIN_4   // PE4
#define KEY_PORT GPIOE

/* Private variables ---------------------------------------------------------*/
int motorRunning = 0;    //电机运行状态标志 0表示停止，1表示运行
int steps = 0;        //移动的步数
const int steps_per_rev = 1600;  // 每转1600步，使用1/8细分(200 * 8 = 1600)
volatile int step_count = 0;  // 当前步数计数

int initial_speed = 200; // 初始速度(步/秒)
int max_speed = 800; // 最大速度(步/秒)
int acceleration_time = 1000; // 加速时间(毫秒)
int deceleration_time = 1000; // 减速时间(毫秒)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  //系统时钟配置函数
void controlStepMotor(int steps, int direction);  //步进电机控制函数
void handleKeys(void);   //处理按键输入函数
void setMotorSpeed(int speed);  //设置电机速度函数

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();            //初始化HAL库
  SystemClock_Config();  //配置系统时钟
  MX_GPIO_Init();        //初始化GPIO
  MX_TIM8_Init();        //初始化TIM8
  
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);            //启动TIM8的PWM通道1
  HAL_TIM_Base_Start_IT(&htim8);                       //启动TIM8定时器中断
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET);  //使能电机
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);//设置电机默认方向

  while (1)
  {
    handleKeys();   //处理按键输入
    
    if (motorRunning)   //如果motorRunning为真
    {
      if (steps > 0)
      {
        controlStepMotor(steps, 1); //正向移动steps步
        steps = 0; //清零步数
      }
      else if (steps < 0)
      {
        controlStepMotor(-steps, 0); //反向移动steps步
        steps = 0; //清零步数
      }
      motorRunning = 0; //停止电机运行
    }

    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); //切换LED的状态
    HAL_Delay(100);  //延时100ms
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();   //使能电源控制时钟
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);//配置电压调节器输出电压

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;    //配置RCC振荡器和时钟
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();   //如果时钟配置失败，调用Error_Handler()函数
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Control step motor
  * @param steps: number of steps to move
  * @param direction: direction of movement (1: forward, 0: backward)
  */
void controlStepMotor(int steps, int direction)
{
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);  //设置电机方向
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET);   //使能电机

  step_count = 0;  //重设步数计数器

  // 加速阶段
  for (int speed = initial_speed; speed <= max_speed; speed += (max_speed - initial_speed) / (acceleration_time / 10))
  {
    setMotorSpeed(speed);
    HAL_Delay(10);
    if (step_count >= steps / 2) //如果达到一半步数，退出加速
    {
      break;
    }
  }

  // 匀速阶段
  setMotorSpeed(max_speed);
  while (step_count < steps / 2)
  {
    //匀速运行
  }

  // 减速阶段
  for (int speed = max_speed; speed >= initial_speed; speed -= (max_speed - initial_speed) / (deceleration_time / 10))
  {
    setMotorSpeed(speed);
    HAL_Delay(10);
    if (step_count >= steps)
    {
      break;
    }
  }

  setMotorSpeed(0); //停止电机
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET);   //关闭电机使能
}

/**
  * @brief Set motor speed by changing PWM frequency
  * @param speed: motor speed (steps per second)
  */
void setMotorSpeed(int speed)
{
  if (speed == 0)
  {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);   //停止PWM
  }
  else
  {
    uint32_t period = (SystemCoreClock / 2) / speed; //计算PWM周期
    __HAL_TIM_SET_AUTORELOAD(&htim8, period);   //设置自动重装载值
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, period / 2); //50%占空比
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);   //启动PWM
  }
}

/**
  * @brief Handle key inputs
  */
void handleKeys(void)
{
  if (HAL_GPIO_ReadPin(KEY_PORT, KEY0_PIN) == GPIO_PIN_RESET)  //读取KEY0引脚状态
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); //点亮LED
    steps += steps_per_rev / 4;   //增加步数
    HAL_Delay(200);
  }

  if (HAL_GPIO_ReadPin(KEY_PORT, KEY1_PIN) == GPIO_PIN_RESET)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); //点亮LED
    steps -= steps_per_rev / 4;   //减少步数
    HAL_Delay(200);
  }

  if (HAL_GPIO_ReadPin(KEY_PORT, KEY2_PIN) == GPIO_PIN_RESET)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); //点亮LED
    motorRunning = 1;
    HAL_Delay(200);
  }
  
  //按键释放后熄灭LED
  if (HAL_GPIO_ReadPin(KEY_PORT, KEY0_PIN) == GPIO_PIN_SET &&
      HAL_GPIO_ReadPin(KEY_PORT, KEY1_PIN) == GPIO_PIN_SET &&
      HAL_GPIO_ReadPin(KEY_PORT, KEY2_PIN) == GPIO_PIN_SET)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); //熄灭LED
  }
}

/**
  * @brief This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq(); //禁用中断
  while (1)
  {
  }
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
  //用户自定义错误处理
}
#endif /* USE_FULL_ASSERT */
