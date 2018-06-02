#include "sys.h"
#include "bluetooth.h"
#include "led.h"
	  
u8 BluetoothReceiver;
u8 BluetoothControlMode=0; // 0-stop,1-lane-tracking, 2,3,4,5: left, right, forward, back

//USART3 TxD=GPIOB10(PD8)   RxD=GPIOB11(PD9)
void Bluetooth_Init(u32 bound){
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // remap
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);

	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);         //PB10, PB11 remap tp PD8, PD9
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD , &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
	USART_InitStructure.USART_BaudRate = bound;                 // baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 8 bits per byte
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 1 stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;         // no parity check code
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // no hardware flow
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);                   // init usart 3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);

}

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		BluetoothReceiver = USART_ReceiveData(USART3);
		BluetoothControl();
     } 
} 

void BluetoothControl(void)
{
	if (BluetoothControlMode==0)
	{
		if (BluetoothReceiver=='M') BluetoothControlMode=1; //0x4D
		if (BluetoothReceiver=='L') BluetoothControlMode=2; //0x4c
		if (BluetoothReceiver=='R') BluetoothControlMode=3; //0x52
		if (BluetoothReceiver=='F') BluetoothControlMode=4; //0x46
		if (BluetoothReceiver=='B') BluetoothControlMode=5; //0X42
	}
	else if (BluetoothControlMode!=0)
	{	
		if (BluetoothReceiver=='M') BluetoothControlMode=0;
		if (BluetoothReceiver=='L') BluetoothControlMode=0;
		if (BluetoothReceiver=='R') BluetoothControlMode=0;
		if (BluetoothReceiver=='F') BluetoothControlMode=0;
		if (BluetoothReceiver=='B') BluetoothControlMode=0;
	}
}



