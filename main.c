#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"

int N;
int n;
int i=-1 ;
int j;
int count;
int ncount;
int ADC_VALUE[3];
char BUFFER_REC[2000];
char BUFFER_request[10000];
char SEND[100];
#define RCC_APB2Periph_ADC2              ((uint32_t)0x00000200)

//////////SYST_CLOCK////////////////////////////
///////////////////////////////////////////////
void SystemInit_1() /// CLK_TIMER=168MHz
{

  RCC->CFGR |= 0x00009400;        // AHB_presc=1  APB1_presc=4
  RCC->CR |= 0x00010000;          // HSE_ON
  while (!(RCC->CR & 0x00020000));// wait until HSE READY

  RCC->PLLCFGR = 0x07402A04;      // PLL  M=4, N=168, P=2  Q=7 yapalim   168 Mhz
  RCC->CR |= 0x01000000;          // PLL_ON
  while(!(RCC->CR & 0x02000000)); // wait Pll_ready

  FLASH->ACR = 0x00000605;        // Flash ROM icin 5 Wait state secelim ve ART yi aktif edelim (Rehber Sayfa 55)
  RCC->CFGR |= 0x00000002;        // Sistem Clk u PLL uzerinden besleyelim
  while ((RCC->CFGR & 0x0000000F) != 0x0000000A); // Besleninceye kadar bekle

}





/*********************************************************************************
/*USART2*/ //pour la config de l'esp
void Usart2_Init()
{
  // Enable clock for GPIOA
  RCC->AHB1ENR |= 0x00000001;
  // Enable clock for USART2
  RCC->APB1ENR|=0x00020000;
  //enable USART2_TX to PA2 and USART2_RX to PA3
  GPIOA->AFR[0]=0x00007700;
  // configuring the USART2 ALTERNATE function  to PA2 and PA3
  GPIOA->MODER|=0x2A0000A0;
  //  BaudRate:115200
  USART2->BRR= 0x1120;
  //USART3->BRR     = 0x0890;      // 19200 baud
  //USART4->BRR     = 0x0450;      // 38400 baud
  //USART4->BRR     = 0x02e0;      // 57600 baud
  //USART4->BRR     = 0x016d;      // 115200 baud
  // USART2 enable + RXNEIE & IDLEIE & RE & TE
  USART2->CR1|=0x0000203C;
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_SetPriority(USART2_IRQn,1);
}



/***********************************************************************
/*USART3*/ //pour l'affichage de la réponse de l'esp

void Usart3_Init()
{
  // Enable clock for GPIOB
  RCC->AHB1ENR |= 0x00000002;
  // Enable clock for USART3
  RCC->APB1ENR|=0x00040000;
  //enable USART3_TX to PB10 and USART3_RX to PB11
  GPIOB->AFR[1]=0x00007700;
  // configuring the USART3 ALTERNATE function  to PB10 and PB11
  GPIOB->MODER|=0x2AA00000;
  //  BaudRate:9600
  USART3->BRR = 0x1120;
  //USART3->BRR     = 0x0890;      // 19200 baud
  //USART4->BRR     = 0x0450;      // 38400 baud
  //USART4->BRR     = 0x02e0;      // 57600 baud
  //USART4->BRR     = 0x016d;      // 115200 baud

  // USART3 enable
  USART3->CR1|=0x0000202C;

}

/////////////CONFIG_ADC//////////////////////
////////////////////////////////////////////

void P_ADC2d_InitADC(void)
{

  //Clock Enable (PCLK2=42MHz)
  RCC->APB2ENR |= RCC_APB2Periph_ADC2;
  //ADC2 PRESCALER=8
  ADC->CCR|= 3<<16;
  //ADC2 MODE SCAN (MULTICHANNEL)
  ADC2->CR1|= 0x00000100;
  // enable mode DISCONT 3 channels à la fois aprés le start
  ADC2->CR1|= 0x00004800;
  //ADC2(disable mode CONT )
  ADC2->CR2|= 0x00000000;
   //ADC2 LENGTH CHANNEL=3
   ADC2->SQR1 = 2<<20;
  //PA3 CHANNEL3 ADC2
   ADC2->SMPR2=0x00000000;
   ADC2->SMPR1=0x00000000;
   ADC2->SQR3=0x00003dC4;
   //EOCS=1 PASSE à 1 à la fin de chaque conversiOn (EOCS=CR2[10])
   ADC2->CR2|= 1<<10;

   // Enable clock for GPIOA & GPIOC
   RCC->AHB1ENR |= 0x00000005;
   //PA4 are analog inputs
   GPIOA->MODER |= 0x00000300;
   //PC4 PC5 are analog inputs
   GPIOC->MODER |= 0x00000F00;


   // EOCIE=1 interrupt Enable (EOCIE=CR1[5])
   ADC2->CR1|=1<<5;
   //ENABLE REQUEST INTERRUPT
   NVIC_EnableIRQ(ADC_IRQn);
   NVIC_SetPriority(ADC_IRQn,2);
   /* Set the ADON bit to wake up the ADC from power down mode */
   ADC2->CR2 |= (uint32_t)ADC_CR2_ADON;
}


/////////////CONFIG_TIMER6////////////////////////////////
/////////////////////////////////////////////////////////
void config_TIMER6() //débordement chaque s
{
	 // Enable TIM6 clock (RCC_APB1ENR[4]=1)
	 RCC->APB1ENR |= 0x00000010;

	 // Set prescaler to PSC
	 TIM6->PSC = 41999;
	 // Set ARR to 999
	 TIM6->ARR =39999; // le compteur compte de 0 à ARR
	                   // -->puis UIF passe 1
	 //INTERRUPT ENABLE (TIM6: DIER[0]=UIE=1)
     TIM6->DIER |= 0x0001; // TIM6 passe en inteerupt qd UIF=1
     //ENABLE INTERRIUPT REQUEST
	 NVIC_EnableIRQ(TIM6_DAC_IRQn); //interrupt request = 1
	 //ENABLE COUNTER
	 TIM6->CR1|=0x0001;

}


/**********************************************************************************
/**********************************************************************************
    //TRANSMISION 1 seul caractère
 **********************************************************************************/

void SendChar2(char Tx)
{
	while((USART2->SR&0x80)==0);  // On attend à ce que le TDR soit dispo
	USART2->DR=Tx;
}

/**********************************************************************************
    //TRANSMISION d'une chaine de caractère: STRING
 **********************************************************************************/
void SendTxt2(char *Adr)
{
  while(*Adr)
  {
    SendChar2(*Adr);
    Adr++;
  }
}

////////////////////////////////////////////////////
void SendChar3(char Tx)
{                                  //TXE !=0
	while((USART3->SR&0x80)==0);  // On attend à ce que le TDR soit dispo
	USART3->DR=Tx;
}

/**********************************************************************************
    //TRANSMISION d'une chaine de caractère
 **********************************************************************************/
void SendTxt3(char *Adr)
{
  while(*Adr)
  {
    SendChar3(*Adr);
    Adr++;
  }
}
//////////////////////
void Delay(int count)
{
	while(count--);
}



//////////////////////////////////////////////////////////////
/***********-CONFIG_ESP-*************************************/

void config_GSM()
{
	SendTxt2("AT\r\n");
	Delay(42000000);
	Delay(42000000);

	SendTxt2("AT+CPIN?\r\n");
	Delay(42000000);
	Delay(42000000);

	SendTxt2("AT+CREG?\r\n");
	Delay(84000000);

	SendTxt2("AT+CGATT?\r\n");
	Delay(84000000);

	SendTxt2("AT+CIPSHUT\r\n");
	Delay(84000000);

	SendTxt2("AT+CIPSTATUS\r\n");
	Delay(84000000);
	Delay(42000000);

	SendTxt2("AT+CIPMUX=0\r\n");
	Delay(84000000);
}



int main(void)
{
 //System clock : HSExPLL=8x21=168MHz
 SystemInit_1();
 P_ADC2d_InitADC();
 Usart2_Init();
 Usart3_Init();


 config_GSM();
 config_TIMER6();
 RCC->AHB1ENR|=0x8;
 GPIOD->MODER=0x55<<24;
  while(1);
}
///////////////INTERRUPTIONS///////////////////////
//////////////////////////////////////////////////



void TIM6_DAC_IRQHandler()
{

	if((TIM6->SR & 0x0001) != 0) //TEST  if TIMER6 FLAG UIF is set
    {
        GPIOD->ODR^=0x8000;
		ADC2->CR2 |= (uint32_t)ADC_CR2_SWSTART;

		//Clear FLAG UIF (UIF=0)
		TIM6->SR &= 0x0000;
    }


}


void   USART2_IRQHandler()
{

	if((USART2->SR&0x0020)!=0) //TEST FLAG RXNE
    {
	  i++;
	  BUFFER_REC[i]=USART2->DR;
	}

	if((USART2->SR&0x0010)!=0)//TEST FLAG IDLE
	{
		 n=strlen(BUFFER_REC);

		 for(j=0;j<n;j++)
		 	   {
		 		 SendChar3(BUFFER_REC[j]);
		 	   }

		 for(j=0;j<n;j++)
		       {
		       		 BUFFER_REC[j]=0;
		       }
		  i=-1;
	      USART2->SR;//CLEAR IDLE
          USART2->DR;
    }
}




void ADC_IRQHandler(void)
{
      //TEST FLAG EOC
	 if(ADC2->SR&(1<<1))
	 {
	     ncount++;

		 if(ncount==1)
		 {
			 ADC_VALUE[0]=ADC2->DR; // EOC is cleared when DR is read

		 }
		 if(ncount==2)
		 {
			 ADC_VALUE[1]=ADC2->DR;

		 }
		 if(ncount==3)
		 {
			 ADC_VALUE[2]=ADC2->DR;

			sprintf(BUFFER_request,"GET /update?key=I5DA4OM2PS7CJNWK&field1=%d&field2=%d&field3=%d\r\n",ADC_VALUE[0],ADC_VALUE[1],ADC_VALUE[2]);

		 	SendTxt2("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n");

			Delay(84000000);


           /* sprintf(SEND,"AT+CIPSEND=%d\r\n",N);

		 	SendTxt2(SEND);

		 	Delay(84000000);
          */
			SendTxt2("AT+CIPSEND\r\n");
			Delay(84000000);

		 	SendTxt2(BUFFER_request);
		 	//SendTxt2("\r\n");
			Delay(84000000);
			Delay(42000000);

			//SendTxt2("AT+CSCS=\"HEX\"");
		 	SendTxt2("\x1A\r\n");
			Delay(84000000);


			SendTxt2("AT+CIPSHUT\r\n");
			Delay(84000000);
		 	ncount=0;

		 }

    }
}
//le système transmet DATA chaque s







