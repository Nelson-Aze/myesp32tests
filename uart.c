
#include <Includes_Project.h>
#include <stm32f10x.h>
#include USART_H
#include GPIO_H
#include PLATFORM_TYPES_H
#include RGB_LED_H
#include SERIAL_BUFFER_H

extern BOOL SIMCOM_Data_Read(UBYTE Data);

/*	This Function is called to Initialise the USART Pins for Both COM & SIM Module	*/
void USART_Pin_Init()
{
	GPIO_Config PB10, PB11, PA9, PA10;

	/*	Enable the clock for PORTB	*/
	PORTB_CLOCK_ENABLE();
	PORTA_CLOCK_ENABLE();

	/*	Configure PB10 pin as Output_Alternate_PushPull	for TX operation*/
	PB10.CurrentPort = PB;
	PB10.CurrentPin = P10;
	PB10.PinMode = Speed_50MHz_Output;
	PB10.PinState = Output_Alternate_PushPull;

	/*	Configure PB11 pin as Input_Floating	for RX operation*/
	PB11.CurrentPort = PB;
	PB11.CurrentPin = P11;
	PB11.PinMode = Input;
	PB11.PinState = Input_Floating;

	/*	Configure PB6 pin as Output_Alternate_PushPull	for TX operation*/
	PA9.CurrentPort = PA;
	PA9.CurrentPin = P9;
	PA9.PinMode = Speed_50MHz_Output;
	PA9.PinState = Output_Alternate_PushPull;

	/*	Configure PB7 pin as Input_Floating for RX operation*/
	PA10.CurrentPort = PA;
	PA10.CurrentPin = P10;
	PA10.PinMode = Input;
	PA10.PinState = Input_Floating;

	GPIO_Config_Pin(PB10);
	GPIO_Config_Pin(PB11);
	GPIO_Config_Pin(PA9);
	GPIO_Config_Pin(PA10);
}

/*	This is the Interrupt Handler User For COM	*/
void USART3_IRQHandler(void)
{
	if((USART3->SR & USART_REG_SR_RXNE_FLAG))
	{
		
		//UBYTE Data;

		/*	Read the Data	from the Data Register	*/
		//Data = (UBYTE)(USART3->DR & 0xFF);

		// ComRxISR(Data);

		/*	Clear the Interrupt flag	*/
		USART3->SR &= ~(USART_REG_SR_RXNE_FLAG);
	}
}

/*	This is the Interrupt Handler User For SIM Module	*/
void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_REG_SR_RXNE_FLAG)
	{
		UBYTE Data;
		
		/*	Read the Data	from the Data Register	*/
		Data = (UBYTE)(USART1->DR & 0xFF);

		(void)SIMCOM_Data_Read(Data);

		/*	Clear the Interrupt flag	*/
		USART1->SR &= ~(USART_REG_SR_RXNE_FLAG);
	}
}

/*	This Function is called to Configure the USART for Both COM & SIM Module	*/
void USART_Init()
{
	USART_Pin_Init();

		/*	Enable Clock to USART3 for COM	*/
	RCC_USART3_CLK_ENABLE();

	/*	Configure the word length	*/
	USART3->CR1 &= ~(USART_REG_CR1_WL_1S8B);

	/*	Configure the number of stop bits	*/
	USART3->CR2 |= USART_REG_CR2_STOP;
	
	/*	Configure BaudRate	*/
	USART3->BRR = USART_REG_BRR_APB1_BAUDRATE_115200;
	
	/*	Enable	Tx and Rx	*/
	USART3->CR1 |= USART_REG_CR1_TX_ENABLE | USART_REG_CR1_RX_ENABLE;
	
	/*	Enable Interrupt	*/
	USART3->CR1 |= USART_REG_CR1_RXNE_ENABLE;
	
	/*	Assign ISR	*/
	NVIC_EnableIRQ(USART3_IRQn);

	/*	Set Priority to USART3	*/
	//NVIC_SetPriority(USART3_IRQn,2);

	/*	Enable	USART	*/
	USART3->CR1 |= USART_REG_CR1_USART_EN;

	/*	Enable Clock to USART1 for SIM Module	*/
	RCC_USART1_CLK_ENABLE();

	/*	Configure the word length	*/
	USART1->CR1 &= ~(USART_REG_CR1_WL_1S8B);

	/*	Configure the number of stop bits	*/
	USART1->CR2 |= USART_REG_CR2_STOP;
	
	/*	Configure BaudRate	*/
	USART1->BRR = USART_REG_BRR_APB2_BAUDRATE_57600;
	
	/*	Enable	Tx and Rx	*/
	USART1->CR1 |= USART_REG_CR1_TX_ENABLE | USART_REG_CR1_RX_ENABLE;
	
	/*	Enable Interrupt	*/
	USART1->CR1 |= USART_REG_CR1_RXNE_ENABLE;
	
	/*	Assign ISR	*/
	NVIC_EnableIRQ(USART1_IRQn);

	/*	Set Priority to USART3	*/
	//NVIC_SetPriority(USART1_IRQn,1);

	/*	Enable	USART	*/
	USART1->CR1 |= USART_REG_CR1_USART_EN;

}

/*	This Function Can be used to transmit a byte data */
void USART_Transmit(UBYTE Data)
{
	/*	Write the Data to the Data Register	*/
		USART3->DR = Data;
	
	/*	Wait for the Tansmission complete bit to set	*/
		while(!(USART3->SR & 0x40));
}

void USART_Send_String(const char *Data)
{
	while (*Data)
	{
		USART_Transmit(*Data);
		Data++;
	}

	USART_Transmit(0x0D);
	USART_Transmit('\n');
}

void USART_Send_Number(unsigned short int Number)
{
	unsigned short int temp = Number / 10000;
	Number %= 10000;
	USART_Transmit(temp + 48);
	
	temp = Number / 1000;
	Number %= 1000;
	USART_Transmit(temp + 48);
	
	temp = Number / 100;
	Number %= 100;
	USART_Transmit(temp + 48);
	
	temp = Number / 10;
	Number %= 10;
	USART_Transmit(temp + 48);
	
	USART_Transmit(Number + 48);
	
	USART_Transmit(0x0D);
	USART_Transmit('\n');
}

/*	This Function Can be used to receive a byte data */
UBYTE USART_Receive()
{
	UBYTE Data=0;
	
	if((USART3->SR)	&	USART_SR_RXNE)
	{
		/*	Read the Data	from the Data Register	*/
		Data = (UBYTE)(USART3->DR & 0xFF);

		USART3->SR &= ~(USART_SR_RXNE);	
	}
	
	return Data;
}

/*	This Function Can be used to know the availability of a data */
UBYTE ISDataAvailable()
{

	if((USART3->SR & USART_SR_RXNE))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*	This Function Can be used to know the availability of a data */
static UBYTE ISGSMDataAvailable()
{

	if((USART1->SR & USART_SR_RXNE))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/* This function can be used to Enable Receive of a data */
void UART_enable_receive(void)
{
    if((USART1->CR1 & USART_REG_CR1_RX_ENABLE)==(USART_REG_CR1_RX_ENABLE))
	{
		/*	Enable Receiving	*/
		USART1->CR1 |= USART_REG_CR1_RX_ENABLE;
	}
}

/*	This function can be used to transmit a data for SIM Module	*/
void SIM_Send_Data(UBYTE Data)
{

	/*	Write the Data to the Data Register	*/
	USART1->DR = Data;

	/*	Wait for the Tansmission complete bit to set	*/
	while (!(USART1->SR & 0x40));

}

/*	This function can be used to Receive a data for SIM Module	*/
UBYTE SIM_Receive_Data(void)
{
	UBYTE Data=0;
	
	if((USART1->SR)	&	USART_SR_RXNE)
	{
		/*	Read the Data	from the Data Register	*/
		Data = (UBYTE)(USART1->DR & 0xFF);

		USART1->SR &= ~(USART_SR_RXNE);	
	}
	
	return Data;
}

void Fetch_Serial_Data()
{
	if(ISDataAvailable())
	{
		//ComRxISR(USART_Receive());
	}
	if(ISGSMDataAvailable())
	{
		//SIM_Send_Data(SIM_Receive_Data());
		//Accumulate_Rx_Data(SIM_Receive_Data());
	}
}


