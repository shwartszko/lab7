
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "Ethernet.h" 
#include "template.h" 

#define DEBUG debug()
#define CRC_ERROR 100
#define FRAME_LENGTH_ERROR 101
#define PAYLOAD_LENGTH_ERROR 102 //must be greater than 42 and lesser than 1500
#define PREAMBLE_ERROR 103
#define NO_ERROR 104
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t myMAC[] ={0xcc,0xcc,0xcc,0xcc,0xcc,0xcc}; 	//Our MAC address
uint32_t is_frame_ready = 0;
Ethernet_res frame_res;
uint32_t timeout = 0;
Ethernet_req* temp = NULL;
Ethernet_req* temp2;
uint8_t PSN = 0;
uint8_t ACK = 0;
uint32_t sub_LLC_struct_ready = 0;
uint32_t mac_tx_busy = 0;
uint32_t sub_ready_for_LLC_Rx = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//for debug
	uint8_t tx_data_wire = 0;
	uint8_t tx_vld_wire = 0;
	uint8_t phy_clk_wire = 0;
	uint8_t phy_busy_wire = 0;
	uint8_t phy_reset_wire = 0; 
	uint8_t rx_data_wire = 0;
	uint8_t rx_vld_wire = 0;
	
void debug(void)
{
	 tx_data_wire = (uint8_t)((Phy_tx_data_bus_pin0_GPIO_Port->IDR) & 0x0FF);
	 tx_vld_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_tx_valid_GPIO_Port, Phy_tx_valid_Pin);
	 phy_clk_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_clock_GPIO_Port, Phy_clock_Pin);
	 phy_busy_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_tx_busy_GPIO_Port, Phy_tx_busy_Pin);
	 phy_reset_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_reset_GPIO_Port, Phy_reset_Pin);
	 rx_data_wire = (uint8_t)((Phy_rx_data_bus_pin0_GPIO_Port->IDR)>>8);
	 rx_vld_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_rx_valid_GPIO_Port, Phy_rx_valid_Pin);
}
	//end debug



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void sub_LLC()
{
	//static uint32_t first = 1;
	int i;
	static uint32_t ACK_received = 1;
	static uint32_t new = 1; //should be zero just for yoffi
	if(is_frame_ready) //frame for Rx MAC ready
	{
		is_frame_ready = 0;
		//printf("\r\nframe ready");
		if(frame_res.typeLength[0] == 0x08 && frame_res.typeLength[1] == 0x88) //type - new
		{
			//printf(":type-new");
			if(frame_res.payload[0]) //ACK frame
			{
				ACK_received = 1;
				HAL_TIM_Base_Stop_IT(&htim2);
				printf("\r\nACK of frame %d was received.\r\n",1-frame_res.payload[1]);
				//sub_ready_for_LLC_Rx = 1;
			}
			else //data frame
			{
				//printf(":data frame received,sending ACK");
				ACK = 1;
				sub_LLC_struct_ready = 1;
				sub_ready_for_LLC_Rx = 1;
				for(i = 0; i < 8; i++)
				{
					temp2->destinationMac[i] = temp->sourceMac[i];
					temp2->sourceMac[i] = myMAC[i]; //looks suspicious but it's good trust					
				}
			}
		}
		else //type - old
		{
			//printf(":type-old");
			sub_ready_for_LLC_Rx = 1;
		}
	}
	if(ACK_received || !new) //first packet or packet after receiving ACK or old packets 
	{
		if(temp)
			{//in the end of using 'temp' you have to free memory:
				free((void*)temp->payload);
				free(temp);
				temp = NULL;
			}
		if(isNewTxRequest())
		{
			//printf(":new Tx req");
			temp = getTxRequeset();
			//new = RANDOM;
			if(new) 
			{
				temp->typeLength[0] = 0x08;
				temp->typeLength[1] = 0x88;
				ACK_received = 0;
				PSN = 1 - PSN;
				ACK = 0;
			}
			else //sending old packet 
			{
				ACK = 0;
				temp->typeLength[1] = temp->typeLength[0] = 0;
			}
			sub_LLC_struct_ready = 1;
		}
	}
	else if(!ACK_received && timeout)
	{
		sub_LLC_struct_ready = 1; //sending the lost packet 
	}
		
}

/* Dll Rx functions */
uint8_t frame_ended = 0;
extern uint8_t isRxByteReady(void);									//check if there is new byte received from phy.
extern uint8_t getByte(void); 											//get byte from phy, it's mandatory to use "isRxByteReady()" before calling this function.
void LLC_RX()
{
	uint32_t i;
	uint32_t is_new = 0;
	if(sub_ready_for_LLC_Rx)
	{
		is_new = frame_res.typeLength[0] == 0x08 && frame_res.typeLength[1] == 0x88;
		sub_ready_for_LLC_Rx = 0;
		printf("\r\nReceived a new frame\r\n");
		switch(frame_res.syndrom)
		{
			case NO_ERROR:
				printf("Destination Address is: ");
				for(i = 0; i < MAC_ADDRESS_LEN; i++)
				{
					printf("%x",frame_res.destinationMac[i]);
				}
				printf("\r\nSource Address is: ");
				for(i = 0; i< MAC_ADDRESS_LEN; i++)
				{
					printf("%x",frame_res.sourceMac[i]);
				}
				printf("\r\nPayload is: ");  
				for(i = 0; i < frame_res.payloadSize[0] + frame_res.payloadSize[1]*256+2*(is_new); i++)
				{
					if(is_new && i == 0) //type - new
					{	i=2;}
					printf("%c", frame_res.payload[i]);
				}
				printf("\r\n\r\n");
				break;
			case PREAMBLE_ERROR:
				printf("\r\nOh no, We can't beleive this. Bibi is still prime minister? also Preamble wasn't valid.\r\n\r\n");
				break;
			case PAYLOAD_LENGTH_ERROR:
				printf("\r\nOh no, We can't beleive this. Bibi is still prime minister? also Payload Length wasn't valid..\r\n\r\n");
				break;
			case FRAME_LENGTH_ERROR:
				printf("\r\nOh no, We can't beleive this. Bibi is still prime minister? also Total Frame Length wasn't valid..\r\n\r\n");
				break;
			case CRC_ERROR:
				printf("\nOh no, We can't beleive this. Bibi is still prime minister? also CRC wasn't valid..\r\n\r\n");
				break;	
		}
		free((void*)frame_res.payload);
	}
}

void MAC_RX()
{
	uint8_t byte = 0;
	static uint32_t allocate = 1;
	uint8_t Rx_CRC_res = 0;
	uint8_t Tx_CRC_res = 0;
	static uint32_t data_size = 0;
	int32_t i = 0;
	static uint32_t error = 0;
	static uint32_t rx_frame_counter = 0;
	static uint32_t timer_on = 0;
	static uint8_t rx_frame[1522];
	static uint32_t preamble_counter = 0;
	static uint32_t stat = 0;
	static uint32_t timer = 0;

	if(isRxByteReady())
	{
		if(timer_on)
		{
			//HAL_TIM_Base_Stop_IT(&htim2);
			timer = 0;
			timer_on = 0;		
		}
		if(preamble_counter < 7)
		{
			byte = getByte();
			if(byte == 0xAA)
				preamble_counter++;
			else
			{
				preamble_counter = 0;
			}
		}
		else if(preamble_counter == 7)
		{
			byte = getByte();
			if(byte == 0xAB)
			{
				rx_frame_counter = 0;
				preamble_counter++;
			}
			else
			{
				preamble_counter = 0;
			}
		}
		else
		{
			if(rx_frame_counter==0)
			{
				rx_frame[0] = getByte();
				rx_frame_counter++;
				//printf("\r\n0::%x, ",rx_frame[0]);
			}
			else
			{
				if(rx_frame_counter < 1522)
				{
					byte = getByte();
					rx_frame[rx_frame_counter] = byte;
					//printf("%d::%x, ",rx_frame_counter,rx_frame[rx_frame_counter]);
					rx_frame_counter++;
				}
				else
				{
					frame_res.syndrom = FRAME_LENGTH_ERROR;
					preamble_counter = 0;
					rx_frame_counter = 0;
					is_frame_ready = 1;
				}
				//printf("\r\n16:%x,17:%x",rx_frame[16+8],rx_frame[8+17]);
			}
			//HAL_TIM_Base_Start_IT(&htim2);
			timer_on = 1;
			int c = 1;
		}		
	}
	else
	{
		if(timer_on)
			timer++;
	}
	
	if(timer == 10000) //start buliding the struct
	{
		//printf("\r\nstart building the frame;");
		timer = 0;
		timer_on = 0;
		//HAL_TIM_Base_Stop_IT(&htim2);
		frame_res.syndrom = NO_ERROR;
		if(rx_frame_counter < 64) // frame length error
		{
			frame_res.syndrom = FRAME_LENGTH_ERROR;
		}
		else 
		{
			hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
			Rx_CRC_res = HAL_CRC_Calculate(&hcrc,(uint32_t*)&rx_frame[0],1);
			//printf("0;");
			frame_res.destinationMac[0] = rx_frame[0];
			//if(ACK)
				//printf("0:%x,",rx_frame[0]);
			for(i = 1; i<rx_frame_counter - 4; i++) //crc calc + constructing the frame
			{
				if(ACK)	
				{
					//printf("%d:%x,",i,rx_frame[i]);
					//printf("%d;",i);
				}
				Rx_CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&rx_frame[i],1);
				if(i >= 1 && i <= 5) //destination address
					frame_res.destinationMac[i] = rx_frame[i];
				else if(i >= 6 && i <= 11) //source address
				{
					frame_res.sourceMac[i-6] = rx_frame[i];
				}
				else if(i == 16 && rx_frame[i+1] == 0x08 && rx_frame[i] == 0x88) //type - new
				{
					frame_res.typeLength[1] = 0x88;
					frame_res.typeLength[0] = 0x08;
					frame_res.payloadSize[0] = rx_frame[21];
					frame_res.payloadSize[1] = rx_frame[20];
					data_size = rx_frame[21] + rx_frame[20]*256 + 2; //+2 for ACK and PSN
					frame_res.payload = (uint8_t*)malloc(data_size*sizeof(uint8_t));
					frame_res.payload[0] = rx_frame[18]; //ACK
					frame_res.payload[1] = rx_frame[19]; //PSN
					Rx_CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&rx_frame[17],1);
					Rx_CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&rx_frame[18],1);
					Rx_CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&rx_frame[19],1);
					i=19;
					if(ACK)
					{
						//printf("17;");
						//printf("18;");
						//printf("19;");
						//printf("17:%x,",rx_frame[17]);
						//printf("18:%x,",rx_frame[18]);
						//printf("19:%x,",rx_frame[19]);
					}
				}
				else if(i >= 16 && i <= 17) //payload size , type - old
				{
					frame_res.payloadSize[0] = rx_frame[17];
					frame_res.payloadSize[1] = rx_frame[16];
					data_size = rx_frame[17] + rx_frame[16]*256; 
					frame_res.payload = (uint8_t*)malloc(data_size*sizeof(uint8_t));
				}
				else if(i >= 18 && i < 18 + data_size) //data 
				{
					 frame_res.payload[i-18] = rx_frame[i];
				}	
			}
			Tx_CRC_res = rx_frame[rx_frame_counter-4] + rx_frame[rx_frame_counter-3]*256 + rx_frame[rx_frame_counter-2]*65536 + rx_frame[rx_frame_counter-1]*(2^24); 
			//if(ACK)
			//	printf("tx:%c, rx:%c", Tx_CRC_res, Rx_CRC_res);
			if(Tx_CRC_res != Rx_CRC_res) 
			{
				//printf(":crc error:");
				frame_ended = 0;
				frame_res.syndrom = CRC_ERROR;
			}
			else if(data_size > 1498) //ze mamash lo meshane
			{
				frame_res.syndrom = PAYLOAD_LENGTH_ERROR;
			}
		}
		i=0;
		preamble_counter = 0;
		rx_frame_counter = 0;
		is_frame_ready = 1;
	}
}

/* Dll Tx functions */
uint8_t gap_time_passed = 1;
extern uint8_t isPhyTxReady(void);									//check if the phy_Tx ready to get another byte to send.
extern uint8_t sendByte(uint8_t data); 							//sent byte to phy, it's mandatory to use "isPhyTxReady()" before call this function
extern uint8_t isNewTxRequest(void); 								//check if the upper_layer sent us new data to transmit.
extern Ethernet_req* getTxRequeset(void);						//get from upper_layer data to transmit, it's mandatory to use "isNewTxRequest()" before call this function.
//Important! - after finish using the Ethernet_req struct, you have to free it and also free the Ethernet_req.payload.

void MAC_TX() //payload size msb sent first
{
	static uint8_t* frame;
	static uint32_t timer_on = 0;
	static uint16_t data_size = 0;
	static uint32_t frame_size = 0;
	static uint32_t i = 0;
	static uint32_t CRC_res = 0;
	static uint32_t tx_ready_to_send = 0;
	static uint32_t kaki = 26;
	static uint32_t first_of_frame = 0;
	uint32_t j = 0;
	if (sub_LLC_struct_ready && gap_time_passed)
	{
		sub_LLC_struct_ready = 0;
		if(timer_on)
		{
			HAL_TIM_Base_Stop_IT(&htim3);
			timer_on = 0;
		}
		data_size = ACK == 0? (temp->payloadSize[0] + (temp->payloadSize[1]*256)+4) : 0;
		if(data_size < 42)
		{
			frame = (uint8_t *)malloc(72*sizeof(uint8_t));
			frame_size = 42;
		}
		else
		{
			frame = (uint8_t *)malloc((data_size+30)*sizeof(uint8_t));
			frame_size = data_size;
		}	
		for(i=0; i<frame_size+30; i++)
		{
			if(i < 7) //preamble 
			{
				frame[i] = 0xAA;
			}
			else if(i == 7) //last preamble
			{
				frame[i] = 0xAB;
			}
			else if(i >=8 && i <= 13) //destination Mac be hearot 
			{
				frame[i] = ACK == 0? (temp->destinationMac[i-8]) : (temp2->destinationMac[i-8]);
				if(i==8) 
					CRC_res = HAL_CRC_Calculate(&hcrc,(uint32_t*)&frame[i],1);
				else
					CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1);;
			}
			else if(i >=14 && i <= 19) //source Mac gam be hearot 
			{
				frame[i] = myMAC[i-14];
				CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1);
			}
			else if(i >=20 && i <= 23) //VLAN
			{
				frame[i] = 0;
				CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1);
			}
			else if(i == 24) //Length
			{
				if(temp->typeLength[0] == 0x08 && temp->typeLength[1] == 0x88) //type - new
				{
					frame[i] = 0x88;
				}
				else
				{
					frame[i+1] = temp->payloadSize[0]; 
				}
				CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1); //maybe frame[i+1]
			}
			else if(i == 25) //Length
			{
				if(temp->typeLength[0] == 0x08 && temp->typeLength[1] == 0x88) //type - new
				{
					frame[i] = 0x08;
				}
				else
				{
					frame[i-1] = temp->payloadSize[1];
				}
				CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1); //maybe frame[i-1]
			}
			else if(ACK && i == 26)
			{
				frame[i] = ACK;
				frame[i+1] = PSN;
				frame[i+2] = frame[i+3] = 0;
				i = 29;
			}
			else if(i >=26 && i < 26+data_size)
			{
				if(temp->typeLength[0] == 0x08 && temp->typeLength[1] == 0x88 && i == 26) //type - new
				{
					frame[i] = ACK;	
					frame[i+1] = PSN;
					frame[i+2] = (temp->payloadSize[1]);
					frame[i+3] = (temp->payloadSize[0]);
					CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1);
					CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i+1],1);
					CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i+2],1);
					kaki = 30;
					i=29;
				}
				else
				{
					frame[i] = temp->payload[i-kaki];
				}
				CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1);
			}
			else if(i >=26+data_size && i < 26+frame_size) //Data
			{
				frame[i] = 0;
				CRC_res = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&frame[i],1);
			}	
			else if(i >= 26+frame_size) //CRC
			{
				frame[26+frame_size] = CRC_res & 0xFF; //lsb first
				frame[27+frame_size] = CRC_res & 0xFF00;
				frame[28+frame_size] = CRC_res & 0xFF0000;
				frame[29+frame_size] = CRC_res & 0xFF000000; //msb last
				i=frame_size + 29;
			}
			
		}
		//frame was fully build
		tx_ready_to_send = 1;
		i=0;
		
	}
	
	if(tx_ready_to_send && isPhyTxReady() && i < frame_size+30)
	{
		if(first_of_frame && temp->typeLength[0] == 0x08 && temp->typeLength[1] == 0x88&&!ACK)
		{
			HAL_TIM_Base_Start_IT(&htim2);
			first_of_frame = 0;
			//printf("\r\n");
		}
		sendByte(frame[i]);
		//if(ACK)
		printf("%d:%x,",i,frame[i]);
		i++;
	}
	if(i == frame_size+30)
	{
		printf("---");
		free(frame);
		frame_size = 0;
		data_size = 0;
		CRC_res = 0;
		i = 0;
		kaki = 26;
		tx_ready_to_send = 0;
		gap_time_passed = 0;
		HAL_TIM_Base_Start_IT(&htim3);
		timer_on = 1;
		first_of_frame = 1;
	}
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	printf("------------------------------------------------------------\r\n");
	printf("DLL is On! DLL will print Rx data and Tx data \r\n(My MAC address is: %02X:%02X:%02X:%02X:%02X:%02X)\r\n",
				myMAC[0],myMAC[1],myMAC[2],myMAC[3],myMAC[4],myMAC[5]);
	printf("------------------------------------------------------------\r\n");
	printf("Press any key to start\r\n");
	DllAlive = 1; //DLL is on!
	__HAL_RCC_CRC_CLK_ENABLE();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	temp2 = (Ethernet_req*)malloc(sizeof(Ethernet_req));
	temp2->payloadSize[0] = temp2->payloadSize[1] = 0;
	temp2->typeLength[0] = 0x08; 
	temp2->typeLength[1] = 0x88; 
  while (1)
  {
		//DLL functions - DO NOT TOUCH
		HAL_UART_Receive_IT(&huart2,&recieved_value,1);
		printer();
		DEBUG;
		//End Of DLL functions
		sub_LLC();
		MAC_TX(); //MY FUNCTIONS - DO NO TOUCH, BELEIVE ME ITS WORKS 
		MAC_RX();
		LLC_RX();
		
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Phy_tx_valid_GPIO_Port, Phy_tx_valid_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Phy_tx_data_bus_pin0_Pin|Phy_tx_data_bus_pin1_Pin|Phy_tx_data_bus_pin2_Pin|Phy_tx_data_bus_pin3_Pin 
                          |Phy_tx_data_bus_pin4_Pin|Phy_tx_data_bus_pin5_Pin|Phy_tx_data_bus_pin6_Pin|Phy_tx_data_bus_pin7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Phy_tx_valid_Pin */
  GPIO_InitStruct.Pin = Phy_tx_valid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Phy_tx_valid_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Phy_tx_data_bus_pin0_Pin Phy_tx_data_bus_pin1_Pin Phy_tx_data_bus_pin2_Pin Phy_tx_data_bus_pin3_Pin 
                           Phy_tx_data_bus_pin4_Pin Phy_tx_data_bus_pin5_Pin Phy_tx_data_bus_pin6_Pin Phy_tx_data_bus_pin7_Pin */
  GPIO_InitStruct.Pin = Phy_tx_data_bus_pin0_Pin|Phy_tx_data_bus_pin1_Pin|Phy_tx_data_bus_pin2_Pin|Phy_tx_data_bus_pin3_Pin 
                          |Phy_tx_data_bus_pin4_Pin|Phy_tx_data_bus_pin5_Pin|Phy_tx_data_bus_pin6_Pin|Phy_tx_data_bus_pin7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Phy_rx_data_bus_pin2_Pin Phy_rx_data_bus_pin3_Pin Phy_rx_data_bus_pin4_Pin Phy_rx_data_bus_pin5_Pin 
                           Phy_rx_data_bus_pin6_Pin Phy_rx_data_bus_pin7_Pin Phy_rx_data_bus_pin0_Pin Phy_rx_data_bus_pin1_Pin */
  GPIO_InitStruct.Pin = Phy_rx_data_bus_pin2_Pin|Phy_rx_data_bus_pin3_Pin|Phy_rx_data_bus_pin4_Pin|Phy_rx_data_bus_pin5_Pin 
                          |Phy_rx_data_bus_pin6_Pin|Phy_rx_data_bus_pin7_Pin|Phy_rx_data_bus_pin0_Pin|Phy_rx_data_bus_pin1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Phy_rx_valid_Pin Phy_tx_busy_Pin Phy_reset_Pin */
  GPIO_InitStruct.Pin = Phy_rx_valid_Pin|Phy_tx_busy_Pin|Phy_reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Phy_clock_Pin */
  GPIO_InitStruct.Pin = Phy_clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Phy_clock_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
