
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Calculateur A code
  ******************************************************************************
  * @attention
  * Copyright (c) STMicroelectronics and ENSA MARRAKECH 2022.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "string.h"
#include "stdlib.h"
#include "fonction_gps.h"

/*typedef -----------------------------------------------------------*/
	CAN_TxHeaderTypeDef TxHeader;
	CAN_TxHeaderTypeDef TxHeader_B;
	CAN_RxHeaderTypeDef RxHeader;
	CAN_FilterTypeDef filter1_config;

/* variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;

    // Car variables :
      uint8_t car_state_flag = 0;

	// RPM variabls :
    uint32_t cont = 0;
	uint16_t speed=0;

	//Moteurs variables:
    #define moteur_pwm_max 100
    #define moteur_pwm_demi_max 80
	uint8_t moteur_rx_data; // 3 bits from calc C

	// Servo variables :
	uint8_t servo_rx_data;
    #define servo_deg_max  35
    #define servo_deg_demi_max 55
    #define servo_deg_demi_min 125
    #define servo_deg_min  140

	//capteur MAX741 variabls:
	uint16_t adc_VI[3]; // courant  tension potontiometer
	float Tension_BAT=0.0;
	float Courant_BAT=0.0;


	// ultrasonic 1 values :
    #define distance_seuil 15   //15 cm
	uint32_t Capt_echo_fm = 0;
	uint32_t Capt_echo_fd = 0;
	uint32_t echo_diff = 0;
	uint8_t echo_first_capt_flag = 0;
	uint8_t ultra_distance_cm  = 0;
	uint8_t calc_distance1_flag = 1;
	uint8_t obstacle_avant_flag = 0;


	// ultrasonic 2 values :
		uint32_t Capt_echo_fm2 = 0;
		uint32_t Capt_echo_fd2 = 0;
		uint32_t echo_diff2 = 0;
		uint8_t echo_first_capt_flag2 = 0;
		uint8_t ultra_distance_cm2  = 25;
		uint8_t calc_distance2_flag = 1;
		uint8_t obstacle_arr_flag = 0;

    // CAN1 variabls:
    #define data_len 1
	uint8_t Tx_Data[data_len];
	uint8_t Tx_Data_B[data_len];
	uint8_t Rx_Data[data_len];
	uint32_t TxMailbox;
	uint32_t TxMailbox_B;


	// GPS variabls :

    #define taille 500 //419
    uint8_t flag = 0;
    uint8_t GPS_rx_complete_flag=0;
    uint8_t Data[2];
    // uint8_t trame_GPS[38] = "G*3139.58749,N,00801.41035,W,212330\r\n"; // trame GPS
    // uint8_t trame_VCT[23] = "V*235,C*6.86,T*14.65\r\n";  // trame vitesse/courant/tension
     char tmp[7];
     char cord[27];
     #define taille 500
     uint8_t tab[taille] ={0};
     /*
    		  "$GPGGA,211821.00,3139.58468,N,00801.40319,W,1,04,1.96,455.6,M,42.7,M,,*47\r\n"
    		  "$GPGSA,A,3,32,31,02,26,,,,,,,,,3.35,1.96,2.71*08\r\n"
    		  "$GPGSV,2,1,08,02,16,053,26,11,10,051,18,12,26,059,22,25,53,032,24*74\r\n"
    		  "$GPGSV,2,2,08,26,14,277,33,29,83,117,,31,35,318,35,32,47,246,37*7B\r\n"
    		  "$GPGLL,3139.58749,N,00801.41035,W,212330.00,A,A*7F\r\n"
    		  "$GPRMC,211822.00,A,3139.58477,N,00801.40308,W,0.282,,061121,,,A*62\r\n"
    		  "$GPVTG,,T,,M,0.282,N,0.523,K,A*2F\r\n";
      */


/* function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* Prototypes definition */
        // Car fonctions :
            void CAR_update(void){
            	car_state_flag = (Rx_Data[0]>>6) & 0x01;

            }
        // calcule courant tension:
           void Calcule_VI(void){
        	   float tension_image = adc_VI[0] * 3.3 / 4095.0 ;
        	   Courant_BAT =  (122.1 * tension_image / 0.119) * 0.001;

        	   tension_image = adc_VI[1] * 3.3 / 4095.0 ;
        	   Tension_BAT =  tension_image * 8 /1.64 ;

           }
        //Servo moto fonctions:
			void servo_write(uint8_t deg){
			   float ccr = 31.11  * deg + 1600.0;  // ccr ~ 1500 --- 7500
			   //htim12.Instance->CCR1 = (uint16_t)ccr;
			   TIM12->CCR1 = (uint16_t)ccr;

			}

			void servo_update(void){
				servo_rx_data = (Rx_Data[0]>>3) & 0x07 ;
				switch(servo_rx_data){
					case 0 : servo_write(90); break;
					case 1 : servo_write(servo_deg_demi_max); break;
					case 2 : servo_write(servo_deg_max); break;
					case 5 : servo_write(servo_deg_demi_min); break;
					case 6 : servo_write(servo_deg_min); break;
				}

			}
        //Motors fonctions:
		void set_pwm_motor(TIM_HandleTypeDef *htim,uint8_t duty){
		  uint32_t CCr = 55999 * duty/100.0;   // ARR= 55999
		  htim->Instance->CCR1 = CCr ;

		}

		void motor_update(void){
			moteur_rx_data = Rx_Data[0] & 0x07 ;
			switch(moteur_rx_data){
			case 0 : set_pwm_motor(&htim3,0); break;
			case 1 : set_pwm_motor(&htim3, moteur_pwm_demi_max);  // sense avant 1/2 max
			         HAL_GPIO_WritePin(moteur_p1_GPIO_Port, moteur_p1_Pin, 1);
				     HAL_GPIO_WritePin(moteur_p2_GPIO_Port, moteur_p2_Pin, 0);
				     //send not recule to B
				     if((Tx_Data_B[0] & 0x2) != 0){
				                Tx_Data_B[0] &= ~(1<<1); // clear bit 1
				                HAL_CAN_AddTxMessage(&hcan1, &TxHeader_B, Tx_Data_B, &TxMailbox_B);
				     }
				     break;
			case 2 : set_pwm_motor(&htim3, moteur_pwm_max); // sense avant max
	                 HAL_GPIO_WritePin(moteur_p1_GPIO_Port, moteur_p1_Pin, 1);
		             HAL_GPIO_WritePin(moteur_p2_GPIO_Port, moteur_p2_Pin, 0);
		             if((Tx_Data_B[0] & 0x2) != 0){
		             		 Tx_Data_B[0] &= ~(1<<1); // clear bit 1
		             		 HAL_CAN_AddTxMessage(&hcan1, &TxHeader_B, Tx_Data_B, &TxMailbox_B);
		             				     }
		             break;
			case 5 : set_pwm_motor(&htim3,moteur_pwm_demi_max);  //sense arr 1/2 max
            		 HAL_GPIO_WritePin(moteur_p1_GPIO_Port, moteur_p1_Pin, 0);
                     HAL_GPIO_WritePin(moteur_p2_GPIO_Port, moteur_p2_Pin, 1);
                     if(!(Tx_Data_B[0] & 0x2)){
						 Tx_Data_B[0] |= (1<<1); // set bit 1
						 HAL_CAN_AddTxMessage(&hcan1, &TxHeader_B, Tx_Data_B, &TxMailbox_B);
									 }
                     break;
			case 6 : set_pwm_motor(&htim3,moteur_pwm_max);  // sense arr max
			         HAL_GPIO_WritePin(moteur_p1_GPIO_Port, moteur_p1_Pin, 0);
			         HAL_GPIO_WritePin(moteur_p2_GPIO_Port, moteur_p2_Pin, 1);
			         if(!(Tx_Data_B[0] & 0x2)){
						 Tx_Data_B[0] |= (1<<1); // set bit 1
						 HAL_CAN_AddTxMessage(&hcan1, &TxHeader_B, Tx_Data_B, &TxMailbox_B);
			       	 }
			         break;
			}
		}

		//ultrasonic fontions :
		void delay_us (uint16_t time)
		{
		__HAL_TIM_SET_COUNTER(&htim9, 0);
		while (__HAL_TIM_GET_COUNTER (&htim9) < time);
		}

		void ultrason_start_read (uint8_t capt_number)
		{
		 if((capt_number==1) && calc_distance1_flag){
			 // capteur avant 1
		      calc_distance1_flag = 0;
		      HAL_GPIO_WritePin(trig_ultrason_1_GPIO_Port, trig_ultrason_1_Pin, GPIO_PIN_SET);  // trig to HIGH
		      delay_us(10);  // att 10 us
	       	  HAL_GPIO_WritePin(trig_ultrason_1_GPIO_Port, trig_ultrason_1_Pin, GPIO_PIN_RESET);  // TRIG to low
		      __HAL_TIM_ENABLE_IT(&htim9, TIM_IT_CC1);
		 }
		 else if((capt_number==2) && calc_distance2_flag){
			 // capteur arrière 2
			 calc_distance2_flag = 0;
			 HAL_GPIO_WritePin(trig_ultrason_2_GPIO_Port, trig_ultrason_2_Pin, GPIO_PIN_SET);  // trig to HIGH
			 delay_us(10);  // att 10 us
			 HAL_GPIO_WritePin(trig_ultrason_2_GPIO_Port, trig_ultrason_2_Pin, GPIO_PIN_RESET);  // TRIG to low
			 __HAL_TIM_ENABLE_IT(&htim9, TIM_IT_CC2);
		     }
		}

		void obstacles_check(void){
			ultrason_start_read (1);
			if(ultra_distance_cm > distance_seuil ) obstacle_avant_flag = 0;
			else obstacle_avant_flag = 1;

			ultrason_start_read (2);
			if(ultra_distance_cm2 > distance_seuil) obstacle_arr_flag = 0;
			else obstacle_arr_flag = 1;

		}
        // GPS PROTOTYPE
		void GPS_encode(uint8_t * , short  ,char*  , char * );
		void GPS_affichage_UART(UART_HandleTypeDef *,char*  ,char*  );

        // ESP8266 UART:
		void send_frame_VCT(UART_HandleTypeDef *huart ,uint16_t V,float C,float T){
			  char frame_tmp[23]; char V_str[5], C_str[5], T_str[6] ;

			  if(V<10) sprintf(V_str ,"00%d", V);
			  else if(V< 100) sprintf(V_str ,"0%d", V);
			  else sprintf(V_str ,"%d", V);

			  if(T < 10) sprintf(T_str ,"0%.2f",T);
			  else  sprintf(T_str ,"%.2f", T);
			  sprintf(C_str ,"%.2f", C);
			  strncpy(frame_tmp,"V*",3);

			  strcat(frame_tmp, V_str);
			  strcat(frame_tmp, ",C*");
			  strcat(frame_tmp, C_str);
			  strcat(frame_tmp, ",T*");
			  strcat(frame_tmp, T_str);
			  strcat(frame_tmp, "\r\n");
			 HAL_UART_Transmit(huart, (uint8_t*) frame_tmp, 22, 2000);
		 }

		 void send_frame_GPS(UART_HandleTypeDef *huart , char *cordonnes , char *time){
			    char frame_tmp[38]={0};
			    strncpy(frame_tmp,"G*", 3);
			    strcat(frame_tmp, cordonnes);
			    strcat(frame_tmp, ",");
			    strcat(frame_tmp, time);
			    strcat(frame_tmp, "\r\n");
			    HAL_UART_Transmit(huart, (uint8_t*) frame_tmp,37 , 2000);
		        // HAL_UART_Transmit(huart, (uint8_t*) "\r\n", 2, 80);
		 }

// main :

int main(void)
{

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_USART6_UART_Init();
  MX_TIM9_Init();
  MX_CAN1_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  /*  CODE inits */
    // capteur max471 init:
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adc_VI,3);

    //moteur init :
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 );
    HAL_GPIO_WritePin(moteur_p1_GPIO_Port, moteur_p1_Pin, 1);
    HAL_GPIO_WritePin(moteur_p2_GPIO_Port, moteur_p2_Pin, 0);

    //SERVO init :
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1 );
    //ultrasonic 1 init:

    //capt vitess init :
    HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim6 );


    // can init:
    TxHeader.DLC = data_len;
    TxHeader.RTR = CAN_RTR_DATA ;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = 0x400; // to calculateur C

    TxHeader_B.DLC = data_len;
    TxHeader_B.RTR = CAN_RTR_DATA;
    TxHeader_B.IDE = CAN_ID_STD;
    TxHeader_B.StdId = 0x407; // to calculateur B

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);

    //uart gps:
    HAL_UART_Receive_DMA(&huart3,tab , taille);

  /* Infinite loop */

  while (1)
  {
	        //Courant tension :
	  	  		Calcule_VI();
	        	send_frame_VCT(&huart1 , speed , Courant_BAT , Tension_BAT); // to ESP8266
		   //GPS:
	          if(GPS_rx_complete_flag){
	        	  GPS_rx_complete_flag=0;
		        GPS_encode(tab , taille ,cord  , tmp );
		        send_frame_GPS(&huart1 , cord , tmp  ); //// to ESP8266

		        // affichage sur uart6
		        GPS_affichage_UART(&huart6 ,cord ,tmp );
	          }

	        // ultrasonics (obstacles):
	           obstacles_check();
             if(obstacle_avant_flag || obstacle_arr_flag ){
            	if(obstacle_avant_flag ){
            		// test command:
            		if((Rx_Data[0] & 0x07) == 1 || (Rx_Data[0] & 0x07) == 2 ){
						Rx_Data[0] = Rx_Data[0] & 0xf8;
						motor_update();
					} else motor_update();

            		if(!(Tx_Data[0] & 0x01)){
            	        Tx_Data[0] |= (1<<0);

            	        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Tx_Data, &TxMailbox);

            		}
            	}
            	if(obstacle_arr_flag){
            		// test command:
            		if((Rx_Data[0] & 0x07) == 5 || (Rx_Data[0] & 0x07) == 6 ){
						Rx_Data[0] = Rx_Data[0] & 0xf8;
						motor_update();
					} else motor_update();
            		// send to calc c:
            		if(!(Tx_Data[0] & 0x02)){
				       Tx_Data[0] |= (1<<1);
				       HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Tx_Data, &TxMailbox);
            		}
            	 }
            	//send to B stop bit
            	if((Tx_Data_B[0] & 0x1) !=1){
            	Tx_Data_B[0] |= (1<<0);
            	HAL_CAN_AddTxMessage(&hcan1, &TxHeader_B, Tx_Data_B, &TxMailbox_B);
            	}
             }
             else{
            	 if(!Tx_Data[0]) {
            	 Tx_Data[0] = 0;
            	 HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Tx_Data, &TxMailbox);
                 }

            	 //send to B not stop:
            	 if((Tx_Data_B[0] & 0x1) != 0){
            		 Tx_Data_B[0] &= ~(1<<0);
            		 HAL_CAN_AddTxMessage(&hcan1, &TxHeader_B, Tx_Data_B, &TxMailbox_B);
            	 }
            	 //moteur :
            	  motor_update();
             }
  }

}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}
// ADC2 Initialization Function

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


 // CAN1 Initialization Function
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // filter config :

      filter1_config.FilterActivation = CAN_FILTER_ENABLE;
      filter1_config.FilterBank = 5;
      filter1_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
      filter1_config.FilterIdHigh = 0x407 << 5;
      filter1_config.FilterIdLow = 0;
      filter1_config.FilterMaskIdHigh =0 ;
      filter1_config.FilterMaskIdLow =0 ;
      filter1_config.FilterMode = CAN_FILTERMODE_IDLIST;
      filter1_config.FilterScale = CAN_FILTERSCALE_32BIT;
      filter1_config.SlaveStartFilterBank = 14;

      HAL_CAN_ConfigFilter(&hcan1, &filter1_config);

}


 // TIM3 Initialization Function

static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 55999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}


// TIM6 Initialization Function

static void MX_TIM6_Init(void)
{



  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 59999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


 // TIM9 Initialization Function

static void MX_TIM9_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 167;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}


// TIM12 Initialization Function

static void MX_TIM12_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 27;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 59999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim12);

}


 // USART1 Initialization Function

static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}


// USART3 Initialization Function

static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}


// USART6 Initialization Function

static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}


//  Enable DMA controller clock

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

}

// GPIO Initialization Function

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_CAN_test_rx_GPIO_Port, led_CAN_test_rx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, moteur_p2_Pin|moteur_p1_Pin|trig_ultrason_1_Pin|trig_ultrason_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_CAN_test_rx_Pin */
  GPIO_InitStruct.Pin = led_CAN_test_rx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_CAN_test_rx_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : moteur_p2_Pin moteur_p1_Pin trig_ultrason_1_Pin trig_ultrason_2_Pin */
  GPIO_InitStruct.Pin = moteur_p2_Pin|moteur_p1_Pin|trig_ultrason_1_Pin|trig_ultrason_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_in_Pin */
  GPIO_InitStruct.Pin = RPM_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_in_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

// definition des fonctions des interruptions :

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
		speed = (cont / 20.0) * 60.0;
		cont= 0 ;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == RPM_in_Pin)
    {
     cont++;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			if (echo_first_capt_flag==0)
			{
				Capt_echo_fm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				echo_first_capt_flag = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // change la polar
			}

			else if (echo_first_capt_flag==1)
			{
				Capt_echo_fd = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				__HAL_TIM_SET_COUNTER(htim, 0);   // cont reset

				if (Capt_echo_fd > Capt_echo_fm)
				{
					echo_diff = Capt_echo_fd-Capt_echo_fm;
				}

				else if (Capt_echo_fm > Capt_echo_fd)
				{
					echo_diff = (0xFFFF - Capt_echo_fm) + Capt_echo_fd;
				}

				ultra_distance_cm = echo_diff * 0.034/2;
				echo_first_capt_flag = 0;
				calc_distance1_flag = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

				__HAL_TIM_DISABLE_IT(&htim9, TIM_IT_CC1);
			}
		}


		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
					if (echo_first_capt_flag2==0)
					{
						Capt_echo_fm2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
						echo_first_capt_flag2 = 1;
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING); // change la polar
					}

					else if (echo_first_capt_flag2==1)
					{
						Capt_echo_fd2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
						__HAL_TIM_SET_COUNTER(htim, 0);   // cont reset

						if (Capt_echo_fd2 > Capt_echo_fm2)
						{
							echo_diff2 = Capt_echo_fd2 - Capt_echo_fm2;
						}

						else if (Capt_echo_fm2 > Capt_echo_fd2)
						{
							echo_diff2 = (0xFFFF - Capt_echo_fm2) + Capt_echo_fd2;
						}

						ultra_distance_cm2 = echo_diff2 * 0.034/2;
						echo_first_capt_flag2 = 0;
						calc_distance2_flag = 1;
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

						__HAL_TIM_DISABLE_IT(&htim9, TIM_IT_CC2);
					}
				}
}


__weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Rx_Data);
        HAL_GPIO_TogglePin(led_CAN_test_rx_GPIO_Port, led_CAN_test_rx_Pin);


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART3){
		HAL_UART_Receive_DMA(&huart1,tab, taille);
        GPS_rx_complete_flag = 1;
	}
}

//  This function is executed in case of error occurrence.

void Error_Handler(void)
{

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics 2022 & ENSA MARRAKECH *****END OF FILE****/
