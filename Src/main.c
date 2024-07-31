/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private define ------------------------------------------------------------*/
#define DIR_PIN GPIO_PIN_14   // ����������� PF14
#define PUL_PIN GPIO_PIN_5    // ������������ PI5 (TIM8_CH1)
#define EN_PIN GPIO_PIN_15    // ʹ������     PF15
#define DIR_PORT GPIOF
#define PUL_PORT GPIOI
#define EN_PORT GPIOF

#define LED_PIN GPIO_PIN_0    // LED�������� PE0
#define LED_PORT GPIOE

#define KEY0_PIN GPIO_PIN_2   // PE2
#define KEY1_PIN GPIO_PIN_3   // PE3
#define KEY2_PIN GPIO_PIN_4   // PE4
#define KEY_PORT GPIOE

/* Private variables ---------------------------------------------------------*/
int motorRunning = 0;    //�������״̬��־ 0��ʾֹͣ��1��ʾ����
int steps = 0;        //�ƶ��Ĳ���
const int steps_per_rev = 1600;  // ÿת1600����ʹ��1/8ϸ��(200 * 8 = 1600)
volatile int step_count = 0;  // ��ǰ��������

int initial_speed = 200; // ��ʼ�ٶ�(��/��)
int max_speed = 800; // ����ٶ�(��/��)
int acceleration_time = 1000; // ����ʱ��(����)
int deceleration_time = 1000; // ����ʱ��(����)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  //ϵͳʱ�����ú���
void controlStepMotor(int steps, int direction);  //����������ƺ���
void handleKeys(void);   //���������뺯��
void setMotorSpeed(int speed);  //���õ���ٶȺ���

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();            //��ʼ��HAL��
  SystemClock_Config();  //����ϵͳʱ��
  MX_GPIO_Init();        //��ʼ��GPIO
  MX_TIM8_Init();        //��ʼ��TIM8
  
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);            //����TIM8��PWMͨ��1
  HAL_TIM_Base_Start_IT(&htim8);                       //����TIM8��ʱ���ж�
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET);  //ʹ�ܵ��
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);//���õ��Ĭ�Ϸ���

  while (1)
  {
    handleKeys();   //����������
    
    if (motorRunning)   //���motorRunningΪ��
    {
      if (steps > 0)
      {
        controlStepMotor(steps, 1); //�����ƶ�steps��
        steps = 0; //���㲽��
      }
      else if (steps < 0)
      {
        controlStepMotor(-steps, 0); //�����ƶ�steps��
        steps = 0; //���㲽��
      }
      motorRunning = 0; //ֹͣ�������
    }

    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); //�л�LED��״̬
    HAL_Delay(100);  //��ʱ100ms
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
  __HAL_RCC_PWR_CLK_ENABLE();   //ʹ�ܵ�Դ����ʱ��
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);//���õ�ѹ�����������ѹ

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
  RCC_OscInitStruct.PLL.PLLQ = 4;    //����RCC������ʱ��
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();   //���ʱ������ʧ�ܣ�����Error_Handler()����
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
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);  //���õ������
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET);   //ʹ�ܵ��

  step_count = 0;  //���貽��������

  // ���ٽ׶�
  for (int speed = initial_speed; speed <= max_speed; speed += (max_speed - initial_speed) / (acceleration_time / 10))
  {
    setMotorSpeed(speed);
    HAL_Delay(10);
    if (step_count >= steps / 2) //����ﵽһ�벽�����˳�����
    {
      break;
    }
  }

  // ���ٽ׶�
  setMotorSpeed(max_speed);
  while (step_count < steps / 2)
  {
    //��������
  }

  // ���ٽ׶�
  for (int speed = max_speed; speed >= initial_speed; speed -= (max_speed - initial_speed) / (deceleration_time / 10))
  {
    setMotorSpeed(speed);
    HAL_Delay(10);
    if (step_count >= steps)
    {
      break;
    }
  }

  setMotorSpeed(0); //ֹͣ���
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET);   //�رյ��ʹ��
}

/**
  * @brief Set motor speed by changing PWM frequency
  * @param speed: motor speed (steps per second)
  */
void setMotorSpeed(int speed)
{
  if (speed == 0)
  {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);   //ֹͣPWM
  }
  else
  {
    uint32_t period = (SystemCoreClock / 2) / speed; //����PWM����
    __HAL_TIM_SET_AUTORELOAD(&htim8, period);   //�����Զ���װ��ֵ
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, period / 2); //50%ռ�ձ�
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);   //����PWM
  }
}

/**
  * @brief Handle key inputs
  */
void handleKeys(void)
{
  if (HAL_GPIO_ReadPin(KEY_PORT, KEY0_PIN) == GPIO_PIN_RESET)  //��ȡKEY0����״̬
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); //����LED
    steps += steps_per_rev / 4;   //���Ӳ���
    HAL_Delay(200);
  }

  if (HAL_GPIO_ReadPin(KEY_PORT, KEY1_PIN) == GPIO_PIN_RESET)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); //����LED
    steps -= steps_per_rev / 4;   //���ٲ���
    HAL_Delay(200);
  }

  if (HAL_GPIO_ReadPin(KEY_PORT, KEY2_PIN) == GPIO_PIN_RESET)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); //����LED
    motorRunning = 1;
    HAL_Delay(200);
  }
  
  //�����ͷź�Ϩ��LED
  if (HAL_GPIO_ReadPin(KEY_PORT, KEY0_PIN) == GPIO_PIN_SET &&
      HAL_GPIO_ReadPin(KEY_PORT, KEY1_PIN) == GPIO_PIN_SET &&
      HAL_GPIO_ReadPin(KEY_PORT, KEY2_PIN) == GPIO_PIN_SET)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); //Ϩ��LED
  }
}

/**
  * @brief This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq(); //�����ж�
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
  //�û��Զ��������
}
#endif /* USE_FULL_ASSERT */
