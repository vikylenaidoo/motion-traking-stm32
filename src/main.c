/*motion tracking

 * MCU: STM32F051, tested on UCT dev Board
 * this code for controlling the stepper motor
 * @TODO: JETSON will send position asynchronously to be read on uart via polling
 * stepper motor controlled by tim14 and freq is changed according to position
 * @TODD: aNOTHER timer for keeping microsecond time
 * @TODO: TIM15 for 90HZ to send the current angle(current_steps) and timestamp(microseconds) to the dma to go to jetson to be logged
 *
 * */
// --------------------IMPORTS--------------------------------------------------------
#include <stdbool.h>
#include <stdlib.h>
//#include <math.h>


#include "stm32f0xx.h"
#include "diag/Trace.h"
#include "motion_tracking.h"

#include "stm32f0xx_usart.h"
#include "stm32f0xx_dma.h"

//----------------------GLOBAL VARS && CONSTANTS------------------------------------------------------------
#define width 320

volatile uint8_t DMA_BUFFER[6]; //6bytes of data to be sent via dma
volatile uint8_t SEND_BUFFER[6]; // 6 byte to be stored before sending to dma, updated every 90hz

volatile uint32_t microseconds;

volatile bool status = 0; //used to keep step status
volatile bool isCalibrated = 0;
volatile bool dir = 1;

volatile uint16_t current_steps = 1200;
volatile uint16_t target_steps = 300; //max: 1200 current_steps == 180*

//as received from the uart
volatile uint16_t x_center;
volatile bool new_x_center_flag;

//will be used to keep the incoming uart data
#define INPUT_BUFFER_MAX_LENGTH 2
volatile uint8_t INPUT_BUFFER[INPUT_BUFFER_MAX_LENGTH]; //max 2 bytes
volatile uint8_t input_buffer_index = 0;

volatile bool time_calibrate_flag = 0; //used if the time zero button is pressed to reset the time on the next 90Hz sending interrupt
volatile uint32_t last_trigger_exti0_1 = 0; //used for debouncing the btn
volatile uint32_t last_trigger_exti2_3 = 0; //used for debouncing the btn
// ------------------------MAIN--------------------------------------------


int main(){
//setup

	init_TIM14(); //used for motor stepping

	init_TIM16(); ///time keeping
	init_TIM17(); //buzzer

	init_GPIOA();
	init_GPIOB();

	init_UART1();
	init_TIM15();//90hz send dma to uart tx

	dir = 1;
	GPIOB->ODR |= GPIO_ODR_1; //SET DIRECTION 1

	isCalibrated = 0;
	calibrate();

	//delay for the motor to come to stops

	init_TIM3();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//interrupt for receive not empty

//main loop
	while(1){
		//float current_angle = current_steps*0.15;
		//uint16_t steps = current_steps;

	/*receive x_center from jetson here*/
		//update_target_steps(x_center);
		//update_motor_control(steps);



	}
}




//-----------------------------INITIALISING FUNCTIONS-------------------------------------

void init_GPIOB(void){//outputs port
	/* PB0 for step
	 * PB1 for direction
	 * PB2 for buzzer
	 * */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable port B
//	GPIOB->MODER  |= 0x00505555;
	//set PB0 and PB1 to output mode
	GPIOB->MODER |= (GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0|GPIO_MODER_MODER2_0);

	GPIOB->ODR     = 0b0000000000000000; //set all output pins low

}


void init_GPIOA(void){ //inputs port
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~GPIO_MODER_MODER0;  // set sw0 to input
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; //pullup for sw0

	GPIOA->MODER &= ~GPIO_MODER_MODER1;  // set sw1 to input
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0; //pullup for sw1

	GPIOA->MODER &= ~GPIO_MODER_MODER2;  // set sw2 to input
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; //pullup for sw2

	//configure interrupts
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; //enable clock for syscfg
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // map pa0 to exti0
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // map pa1 to exti1
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA;

	//SW0 (PA0) used for calibrating the angle
	EXTI->IMR |= EXTI_IMR_MR0; //UNMASK EXTI0
	EXTI->FTSR |= EXTI_FTSR_TR0; // trigger on falling edge

	//SW1 (PA1) used for setting the time to zero
	EXTI->IMR |= EXTI_IMR_MR1; //UNMASK EXTI1
	EXTI->FTSR |= EXTI_FTSR_TR1; // trigger on falling edge

	//SW2 (PA2) used for step testing
	EXTI->IMR |= EXTI_IMR_MR2; //UNMASK EXTI1
	EXTI->FTSR |= EXTI_FTSR_TR2; // trigger on falling edge


	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	//NVIC_SetPriority(EXTI2_3_IRQn, 0);

}


void init_TIM3(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//freq = 48000000/psc/arr hz
	TIM3->PSC = 999; //scale frequency
	TIM3->ARR = 1499; //set value to count up to

	TIM3->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM3->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_SetPriority(TIM3_IRQn, 1);
	NVIC_EnableIRQ(TIM3_IRQn);


}


void init_TIM14(void){ /*this timer used for motor control*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	//freq = 48000000/psc/arr hz
	TIM14->PSC = 999; //scale frequency
	TIM14->ARR = 99; //set value to count up to

	TIM14->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM14->CR1  |= TIM_CR1_ARPE;
	TIM14->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_SetPriority(TIM14_IRQn, 1);
	NVIC_EnableIRQ(TIM14_IRQn);

}

void init_TIM15(void){ /*this timer used at 90Hz to send angle & timestamp data back to the jetson*/
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

	//freq = 48000000/psc/arr = 90hz
	TIM15->PSC = 9;
	TIM15->ARR = 53332;

	TIM15->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM15->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_SetPriority(TIM15_IRQn, 0); //set to highest priority
	NVIC_EnableIRQ(TIM15_IRQn);


}

void init_TIM16(void){ //used for keeping time
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	//freq = 48000000/psc/arr = 1MHz (1 usecond period)
	TIM16->PSC = 0;
	TIM16->ARR = 95;//47;

	TIM16->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM16->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_SetPriority(TIM16_IRQn, 0);
	NVIC_EnableIRQ(TIM16_IRQn);


}

void init_TIM17(void){
	/* Timer used for the buzzer with period of 0.5s
	 * start timer will be done when the zero time has been calibrated
	 * and the timer will be diabled in its own interrupt (after 1 period)
	 * */

	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;

	//freq = 48000000/psc/arr
	TIM17->PSC = 999;
	TIM17->ARR = 95999; //0.5Hz

	TIM17->DIER |= TIM_DIER_UIE; // enable update interrupt event

	NVIC_EnableIRQ(TIM17_IRQn);


}



void init_UART1(void){
	/*initilaise UART1
	 * RX from the jetson will have an interrupt on RXNE
	 * TX is sent from the DMA on the 90Hz timer (polling mode), no DMA interrupts initialised
	 **/

	/*
	 *https://community.st.com/s/question/0D50X00009XkZ55SAF/uart-example-code-for-stm32f0
	 *https://www.st.com/content/ccc/resource/technical/document/user_manual/2f/77/25/0f/5c/38/48/80/DM00122015.pdf/files/DM00122015.pdf/jcr:content/translations/en.DM00122015.pdf
	 *https://community.st.com/s/question/0D50X00009XkaKtSAJ/recieve-more-then-1-byte-over-usart
	 */

// Configure PA9 (Tx) and PA10 (Rx)
	GPIOA->AFR[1]	|=  0b0001 << (1 * 4);      //AFRH, PA9, UART1
	GPIOA->AFR[1]	|=  0b0001 << (2 * 4);      //AFRH, PA10, UART1
	GPIOA->MODER	|=  GPIO_MODER_MODER9_1;  		// PA9 to AF mode
	GPIOA->MODER	|=  GPIO_MODER_MODER10_1;  		// PA10 to AF mode
	GPIOA->OTYPER  	&= ~GPIO_OTYPER_OT_9;       //Output Push/Pull
	GPIOA->OTYPER  	&= ~GPIO_OTYPER_OT_10;       //Output Push/Pull
	GPIOA->OSPEEDR 	|=  GPIO_OSPEEDR_OSPEEDR9;  //PA9 high speed
	GPIOA->OSPEEDR 	|=  GPIO_OSPEEDR_OSPEEDR10;  //PA10 high speed
	GPIOA->PUPDR   	|=  GPIO_PUPDR_PUPDR9_0;         //PA9 Pull up
	GPIOA->PUPDR   	|=  GPIO_PUPDR_PUPDR10_0;         //PA10 Pull up

//configure USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	//enable usart1
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//configure interrupt for uart
	//NVIC_EnableIRQ(USART1_IRQn);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_SetPriority(USART1_IRQn, 1);

//enable uart
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //interrupt for transmit empty
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//interrupt for receive not empty

//configure dma
	init_DMA();
	//NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	//DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel2, ENABLE);

}

void init_DMA(void){
	/*configure the dma
	 * https://community.st.com/s/question/0D50X00009XkZeLSAV/uart-with-dma-mode
	 * */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR; //send to transmit data register (TDR) of the uart line
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&DMA_BUFFER; //6 bytes global buffer
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; //peripheral is destination
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_BufferSize = sizeof(DMA_BUFFER)-1;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //increment the memory adress
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //dont increment the peripheral (uart) adress
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //not used in memory to memory
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;


	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

}



//-----------------------------------------AUX FUNCTIONS -----------------------------------------------
void calibrate(void){
	//set the freq slow in one direction till the button interrupt
	while(!isCalibrated){

	}
	int now = microseconds;
	TIM14->CR1 &= ~TIM_CR1_CEN;
	while(microseconds-now<100000){

	}
	TIM14->CR1 |= TIM_CR1_CEN;
}

void update_motor_control(uint16_t steps){
/*change freq and direction according to current position away from target_angle
 * will try and make current_steps==target_steps
 * target_steps is updated in the uart reading interrupt
 */

	//set frequency
	//note: freq = 48000000/psc/arr psc=999+1
	uint16_t target_local = target_steps;
	uint16_t error = abs(target_local-steps);
	uint32_t arr;
	if(error<=1){
		arr = 1999;//9599; //24 Hz
	}
	else{
		if(error>134){ //20 degrees: fastest
			arr = 29;//47;//479; //1600 hz = 4rad/s	//@TODO: tune eqn, INVESTIGATE FREQ 1KHZ?
		}
		else{
			arr = (uint16_t)(4000/error) - 1; //@TODO: tune eqn
		}
	}
	TIM14->ARR = arr; //set value to count up to

	//set direction
	if(steps<target_steps){
		dir = 0;
		GPIOB->ODR &= ~GPIO_ODR_1; //set direction pin
	}
	else{
		dir = 1;
		GPIOB->ODR |= GPIO_ODR_1; //set direction pin

	}

}

void update_target_steps(uint16_t x_center){
	/*will look at the global xcenter to update what our target angle(steps) should be
	 * update_motor_control() will use the new target_steps to move the motor into position
	 * */

	if(new_x_center_flag){ //check if the xcenter has been updated (from uart rx)
		int dead_center = width/2;
		uint16_t x_error = abs(x_center-dead_center);
		uint16_t target;
		unsigned int steps = current_steps;

		if(x_error<50){ //small error
			if(x_center>dead_center){
				target = steps + 1.5*x_error; //@TODO: tune eqn
			}
			else{
				target = steps - 1.5*x_error; //@TODO: tune eqn
			}
		}
		else{ //large error
			if(x_center>dead_center){
				target = steps + 2*x_error; //@TODO: tune eqn
			}
			else{
				target = steps - 2*x_error; //@TODO: tune eqn
			}
		}

		//check target is within bounds
		if(target<1200 && target>=0){
			target_steps = target;
		}//else invalid target @TODO: see if valid target
		else{
			if(target>32000){ //underflow
				target = 0;
				target_steps = target;
			}
			else{
				target = 1200;
				target_steps = target;
			}
		}

		new_x_center_flag = 0;
	}
}


void reset_time(void){
	TIM16->CR1 &= ~TIM_CR1_CEN;// disable the timekeeping timer
	microseconds = 0;
	TIM16->CR1 |= TIM_CR1_CEN; //restart the time
	//@TODO beep here
	GPIOB->ODR |= GPIO_ODR_2;//beep pin
	TIM17->CR1 |= TIM_CR1_CEN; // start tim17 which will beep for 1 period (0.5s) then disable itself and the beep

}


//----------------------------------------INTERRUPT HANDLERS--------------------------------------
void TIM14_IRQHandler(void){

	status = !status;

	if(status){
		if(dir){
			if(current_steps>0){
				current_steps--;
				GPIOB->ODR |= GPIO_ODR_0; //set PB0 high
			}
		}
		else{
			current_steps++;
			GPIOB->ODR |= GPIO_ODR_0; //set PB0 high
		}

	}
	else{
		GPIOB->ODR &= ~GPIO_ODR_0; //set PB0 low

	}
	//clear the interrupt flag
	TIM14->SR &= ~TIM_SR_UIF;

}

void TIM15_IRQHandler(){
/* data needs to be sent on this interrupt
 * angle(current_steps) as 2 bytes
 * timestamp(microseconds) as 4 bytes (should give about 70 mins run time before overflow)
 * we must create data packet of 6 bytes to be sent via uart to jetson using dma
 * current_steps(2) | microseconds(4)
 **/

	if(time_calibrate_flag){ //time reset has been requested by user pressing SW0
		reset_time();
		time_calibrate_flag = 0;
	}

	//create packet and load into dma buffer
	union{
		uint16_t s;
		uint8_t bytes[2];
	} steps;

	steps.s = (uint16_t)current_steps;
	DMA_BUFFER[0] = steps.bytes[0];
	DMA_BUFFER[1] = steps.bytes[1];


	union{
		uint32_t us;
		uint8_t bytes[4];
	} useconds;

	useconds.us = microseconds;
	DMA_BUFFER[2] = useconds.bytes[0];
	DMA_BUFFER[3] = useconds.bytes[1];
	DMA_BUFFER[4] = useconds.bytes[2];
	DMA_BUFFER[5] = useconds.bytes[3];

	//send command for dma to send
	if(DMA_GetFlagStatus(DMA1_FLAG_TC2)){
		DMA_Cmd(DMA1_Channel2, DISABLE);
		//USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
		DMA_ClearFlag(DMA1_FLAG_TC2);

		DMA_SetCurrDataCounter(DMA1_Channel2, 6);
		DMA_Cmd(DMA1_Channel2, ENABLE);
	}


	TIM15->SR &= ~TIM_SR_UIF; //clear theinterrupt flag

}

void TIM16_IRQHandler(){
	microseconds++;
	TIM16->SR &= ~TIM_SR_UIF; //clear theinterrupt flag
}


void TIM17_IRQHandler(){
	/* disables the beep after 0.5s*/
	GPIOB->ODR &= ~GPIO_ODR_2;
	TIM17->CR1 &= ~TIM_CR1_CEN; //disable itself
	TIM17->SR &= ~TIM_SR_UIF;//clear the interrupt flag
}

void TIM3_IRQHandler(){
	/* update the frequency of motor stepping*/

	uint16_t steps = current_steps;
	update_motor_control(steps);
	TIM3->SR &= ~TIM_SR_UIF;//clear the interrupt flag
}





void EXTI0_1_IRQHandler(void){
	uint32_t now = microseconds;
	if((now-last_trigger_exti0_1)>100000){
		if(EXTI->PR & EXTI_PR_PR0){ //SW0 has been pressed
			//calibrate the angle to zero
			if(!isCalibrated){
				TIM14->CR1 &= ~TIM_CR1_CEN;

				current_steps = 0;

				isCalibrated = 1;
				dir = 0;
				TIM14->CR1 |= TIM_CR1_CEN;
			}

			EXTI->PR |= EXTI_PR_PR0; //clear the interrupt
		}
		//else{

			if(EXTI->PR & EXTI_PR_PR1){ //SW1 has been pressed
				time_calibrate_flag = 1;
				EXTI->PR |= EXTI_PR_PR1; //clear the interrupt
			}
		//}

	}
	last_trigger_exti0_1 = now;
	EXTI->PR |= EXTI_PR_PR0 | EXTI_PR_PR1;//clear both interrrupts
}

void EXTI2_3_IRQHandler(void){
	uint32_t now = microseconds;
	if((now-last_trigger_exti2_3)>100000){
		uint16_t target = target_steps;
		target = target+100;
		target_steps = target;
	}
	last_trigger_exti2_3 = now;
	EXTI->PR |= EXTI_PR_PR2; //clear the interrupt
}

void USART1_IRQHandler(){
	if(USART_GetITStatus(USART1, USART_IT_RXNE)){//receive not empty
		//x_center = USART_ReceiveData(USART1);
		INPUT_BUFFER[input_buffer_index] = USART_ReceiveData(USART1);
		input_buffer_index++;
		if(input_buffer_index>=INPUT_BUFFER_MAX_LENGTH){
			input_buffer_index = 0;
			union{
				uint8_t b[2];
				unsigned int x;
			} bt;

			bt.b[1] = INPUT_BUFFER[0];
			bt.b[0] = INPUT_BUFFER[1];

			//x_center = bt.x;
			new_x_center_flag = 1;
			uint16_t xc = bt.x;

			//now update target current_steps calculated from xcenter
			update_target_steps(xc);
		}

	}

}






// ---------------------------END-------------------------------------------------
