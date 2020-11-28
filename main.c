#include "stm32f7xx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#define BUFSIZE 8192  //Frame-Größe
#define N (BUFSIZE/2)

#define TRIGGER 0x7fff
#define MAGNITUDE_SCALING_FACTOR (BUFSIZE/64)

#define HANN_WINDOW  //auswahl der fensterfunktion
//#define HAMMING_WINDOW

float32_t window[BUFSIZE];

float32_t outvol = 0.5; //ausgangs-gain

uint32_t fftinstance;

float32_t cbuf[2*N];
float32_t *cbufptr;
//float32_t outbuffer[N];
float32_t outbuffer[2*N];

#define PING 1 //Bool: welcher Buffer?
#define PONG 0

#define ALTERNATE 0 //zum wechselnden muten von frames

uint16_t pingIN[BUFSIZE], pongIN[BUFSIZE];  //Eingangs-Doppelbuffer
uint16_t pingOUT[BUFSIZE], pongOUT[BUFSIZE];  //Ausgangs-Doppelbuffer

int rx_proc_buffer, tx_proc_buffer;
volatile int RX_buffer_full = 0;
volatile int TX_buffer_empty = 0;

uint16_t COEFF1 = 0;
uint16_t poti1_value;
uint16_t poti2_value;

float32_t pitch;

  uint32_t i;
  uint32_t j;

  uint16_t k;

void fftinit(){
if(BUFSIZE==32)
fftinstance = &arm_cfft_sR_f32_len16;
else if(BUFSIZE==64)
fftinstance = &arm_cfft_sR_f32_len32;
else if(BUFSIZE==128)
fftinstance = &arm_cfft_sR_f32_len64;
else if(BUFSIZE==256)
fftinstance = &arm_cfft_sR_f32_len128;
else if(BUFSIZE==512)
fftinstance = &arm_cfft_sR_f32_len256;
else if(BUFSIZE==1024)
fftinstance = &arm_cfft_sR_f32_len512;
else if(BUFSIZE==2048)
fftinstance = &arm_cfft_sR_f32_len1024;
else if(BUFSIZE==4096)
fftinstance = &arm_cfft_sR_f32_len2048;
else if(BUFSIZE==8192)
fftinstance = &arm_cfft_sR_f32_len4096;
else
while(1){};
}

void windowinit(){

for(i=0;i<BUFSIZE;i++){  //rechteck-fenster als default
    window[i] = 1;  
    }

#ifdef HAMMING_WINDOW
for(i=0;i<BUFSIZE;i++){
    window[i] = 0.54 - 0.46*cos((2*PI*i)/BUFSIZE);  
    }
#endif

#ifdef HANN_WINDOW
for(i=0;i<BUFSIZE;i++){
    window[i] = 0.5 - 0.5*cos((2*PI*i)/BUFSIZE);
    }
#endif
}

///////////////////////////////////////////
//        Interrupt-Handler              //
///////////////////////////////////////////


void DMA2_Stream0_IRQHandler()  //eingangs-buffer voll
{	
  if((DMA2->LISR) & DMA_LISR_TCIF0)  //Transaction complete interrupt flag
  {
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0; //Transaction complete Flag clearen
     if((DMA2_Stream0->CR) & DMA_SxCR_CT)  //welcher Buffer?
			rx_proc_buffer = PING;
    else
      rx_proc_buffer = PONG;
    RX_buffer_full = 1;
    
  }
  //GPIOC->ODR ++;
  }

void DMA1_Stream4_IRQHandler()  //ausgangs-buffer leer
{	
  if((DMA1->HISR) & DMA_HISR_TCIF4)  //Transaction complete interrupt flag
  {
    DMA1->HIFCR |= DMA_HIFCR_CTCIF4; //Transaction complete Flag clearen
     if((DMA1_Stream4->CR) & DMA_SxCR_CT){  //welcher Buffer?
			tx_proc_buffer = PING;
                         }
    else{
      tx_proc_buffer = PONG;
      }
    TX_buffer_empty = 1;

  }
  //GPIOC->ODR ++;
}

void adc_read(){
ADC1->CR2 |= ADC_CR2_SWSTART;   //adc1 messung starten
poti1_value = ADC1->DR;    

ADC2->CR2 |= ADC_CR2_SWSTART;   //adc2 messung starten
poti2_value = ADC2->DR;    

outvol = 0 + (float32_t)(poti1_value)/5000; 
}

///////////////////////////////////////////
//        Signalverarbeitung             //
///////////////////////////////////////////


void process_buffer()
{
  int i;
  uint16_t *rxbuf, *txbuf;
	  int16_t left_sample, right_sample;

  // ping oder pong buffer feststellung
	if (rx_proc_buffer == PING) rxbuf = pingIN; else rxbuf = pongIN;
  if (tx_proc_buffer == PING) txbuf = pingOUT; else txbuf = pongOUT;
  for (i = 0; i < (BUFSIZE/2) ; i++) 
  {
    left_sample = *rxbuf++;
    right_sample = *rxbuf++;
    cbuf[(i*2)] = (float32_t)left_sample;
    cbuf[(i*2)+1] = 0.0;
  } 

#if 0 //fensterfunktion aktivieren
for(i=0;i<BUFSIZE;i++){
cbuf[i]=cbuf[i]*window[i];
}
#endif
  
#if 1 //analyse
  arm_cfft_f32(fftinstance, (float32_t *)(cbuf), 0, 1); //länge: N, cbuf: 2N
  //arm_cmplx_mag_f32((float32_t *)(cbuf),outbuffer,N);
#endif
  memcpy(outbuffer,cbuf,(8*N));  //8x wegen float-wortlänge



////////////////////////////////////////
///////////////Filter etc///////////////
////////////////////////////////////////

if(GPIOA->IDR & GPIO_IDR_IDR_2){ //taster: filter bypassen

#if 0 //tiefpass-filter
COEFF1 = poti2_value >> 2;
for(i=0;i<BUFSIZE;i++){
  if(  ((i>COEFF1)&&(i<(BUFSIZE-COEFF1))) ){
outbuffer[i]=outbuffer[i]/32;
//outbuffer[BUFSIZE-i]=outbuffer[BUFSIZE-i]/8;
}
}
#endif

#if 0 //band-generator
COEFF1 = poti2_value/50;
for(i=0;i<BUFSIZE;i++){
outbuffer[i]=0;
}
for(i=0;i<3;i++){
outbuffer[i+500+COEFF1]=1000;
}
outvol=outvol*15;
#endif

#if 0 //speed-test
for(i=0;i<BUFSIZE;i++){
outbuffer[i]=outbuffer[i]*((float32_t)(poti2_value)/2048);
outbuffer[i]=outbuffer[i]-20;
outbuffer[i]=outbuffer[i]*((float32_t)(poti2_value)/2048);
outbuffer[i]=outbuffer[i]*((float32_t)(poti2_value)/2048);
}
#endif

#if 0 //pitch-shift 1
for(i=0;i<N;i++){
  outbuffer[2*i]=cbuf[2*((int)((poti1_value*i)/4096U))];
  outbuffer[(2*i)+1]=cbuf[2*((int)((poti1_value*i)/4096U))+1];
}
/*for(i=BUFSIZE;i>N/2;i--){
  outbuffer[2*i]=cbuf[2*((int)((poti1_value*i)/4096U))];
  outbuffer[(2*i)+1]=cbuf[2*((int)((poti1_value*i)/4096U))+1];
}*/
#endif

#if 1 //pitch-shift 2
pitch = 0.9+(float32_t)(poti2_value)/20000;
for(i=2;i<(N/2);i++){
  outbuffer[2*i]=cbuf[2*(int)(pitch*(float32_t)(i))];
  outbuffer[2*i+1]=cbuf[2*(int)(pitch*(float32_t)(i))+1];
  }
for(i=4;i<(N/2);i++){
  outbuffer[BUFSIZE-(2*i)]=cbuf[BUFSIZE-(2*(int)(pitch*(float32_t)(i)))];
  outbuffer[BUFSIZE-(2*i)-1]=cbuf[BUFSIZE-(2*(int)(pitch*(float32_t)(i)))-1]; 
  }
#endif



} //wegen bypass
////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////


#if 1  //re-synthese, 1=audio out, 0=fft out
  #define MAGNITUDE_SCALING_FACTOR 1
  arm_cfft_f32(fftinstance, outbuffer, 1, 1); //länge: N, cbuf: 2N
#endif

  for (i = 0; i < (2*N) ; i++) 
  {
    *txbuf++ = (int16_t)(outvol*outbuffer[i]/MAGNITUDE_SCALING_FACTOR);
  }
  //pingOUT[1]=TRIGGER;
  //pongOUT[1]=TRIGGER;
  TX_buffer_empty = 0;
  RX_buffer_full	= 0;
  
}

#if 0
void process_buffer()
{
  int i;
  uint16_t *rxbuf, *txbuf;
	
	if (rx_proc_buffer == PING) 
		rxbuf = pingIN; 
	else
	  rxbuf = pongIN;
  if (tx_proc_buffer == PING) 
		txbuf = pingOUT;
	else 
		txbuf = pongOUT;
	
  for (i=0 ; i<(BUFSIZE) ; i++)
  {
    *txbuf++ = (*rxbuf++)*2;
  }
  TX_buffer_empty = 0;
  RX_buffer_full = 0;
  //GPIOC->ODR++;
}
#endif


int main(void) {

/////////////statische buffer füllen/////////////


for(i=0;i<BUFSIZE;i++){
#if 0 //rechteck;
  pingOUT[i]=0x7fe0;
  pongOUT[i]=0x8002;
#endif //rechteck
  pingOUT[i]=1000;
  pongOUT[i]=0;
  }


///////////startup-delay/////////////

//for(i=0;i<0xfffff;i++){}

///////////Taktverteilung/////////////
  
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;


////////////GPIO init/////////////////
  
 GPIOA->MODER |= GPIO_MODER_MODER4_1; //LRCK  //GPIOS, 10: alternate function mode
 GPIOA->MODER |= GPIO_MODER_MODER5_1; //SCK
 GPIOA->MODER |= GPIO_MODER_MODER7_1; //DIN

 GPIOA->AFR[0] |= 0x5 << GPIO_AFRL_AFRL4_Pos;   //gpio mux für i2s1
 GPIOA->AFR[0] |= 0x5 << GPIO_AFRL_AFRL5_Pos;
 GPIOA->AFR[0] |= 0x5 << GPIO_AFRL_AFRL7_Pos;

 GPIOB->MODER |= GPIO_MODER_MODER10_1; //LRCK  //GPIOS, 10: alternate function mode
 GPIOB->MODER |= GPIO_MODER_MODER12_1; //SCK
 GPIOB->MODER |= GPIO_MODER_MODER15_1; //DIN

 GPIOB->AFR[1] |= 0x5 << GPIO_AFRH_AFRH2_Pos;;   //gpio mux für i2s2 = PB10,PB12,PB15
 GPIOB->AFR[1] |= 0x5 << GPIO_AFRH_AFRH4_Pos;
 GPIOB->AFR[1] |= 0x5 << GPIO_AFRH_AFRH7_Pos;

 GPIOC->MODER |= GPIO_MODER_MODER4_1;  //PC4 (mcko i2s1) als alternate function
 GPIOC->MODER |= GPIO_MODER_MODER9_1;  //PC9 (mckin i2s) alternate function
 
 GPIOC->AFR[0] |= 0x5 << GPIO_AFRL_AFRL4_Pos; //PC4 i2s mcko
 GPIOC->AFR[1] |= 0x5 << GPIO_AFRH_AFRH1_Pos; //PC9 i2s1 mckin

 //poti pa0 (adc1 in 0)
 GPIOA->MODER |= 0b11 << GPIO_MODER_MODER0_Pos;  //pa0 analog mode

 //poti pa1 (adc2 in 1) 
 GPIOA->MODER |= 0b11 << GPIO_MODER_MODER1_Pos;  //pa1 analog mode

 //taster pa2
 GPIOA->MODER |= 0b00 << GPIO_MODER_MODER2_Pos; //input
 GPIOA->PUPDR |= 0b01 << GPIO_PUPDR_PUPDR2_Pos; //interner pull-up

 /////////////////ADC Config/////////////////////

 ADC1->CR2 |= ADC_CR2_ADON;   //ADC1 an
 ADC2->CR2 |= ADC_CR2_ADON;   //ADC2 an
 ADC->CCR |= 0b11 << ADC_CCR_ADCPRE_Pos;

 ADC2->SQR3 |= 1U << ADC_SQR3_SQ1_Pos; //regular group 1. channel

//////////////////I2S pll Init///////////////// Details: Seite 1129ff im Reference Manual
 RCC->CFGR |= RCC_CFGR_I2SSRC;                //1:externer takt, 0:i2spll als source


#if 0 //pll konfiguration für internen i2s takt
 RCC->PLLI2SCFGR |= 192U << RCC_PLLI2SCFGR_PLLI2SN_Pos;
 RCC->PLLI2SCFGR |= 2U << RCC_PLLI2SCFGR_PLLI2SR_Pos;
 RCC->CR |= RCC_CR_PLLI2SON;
 while(!(RCC->CR & RCC_CR_PLLI2SRDY)){}
#endif


    /////////DMA2 Initialisierung für I2s1 - eingang/////////// f746:S.240 & 1102 im RM

DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR); //periph. adresse
DMA2_Stream0->M0AR = (uint32_t)&pingIN; //memory 0 adresse
DMA2_Stream0->M1AR =(uint32_t)&pongIN; //memory 1 adresse
DMA2_Stream0->NDTR = BUFSIZE;  //Anzahl Samples
DMA2_Stream0->CR |= 3U << DMA_SxCR_CHSEL_Pos; //channel select [2:0]
DMA2_Stream0->CR |= DMA_SxCR_DBM; //double buffer mode
DMA2_Stream0->CR |= DMA_SxCR_CIRC; //circular mode
DMA2_Stream0->CR |= DMA_SxCR_MINC; //increment Memory Adresse
DMA2_Stream0->CR |= (0b01 << DMA_SxCR_MSIZE_Pos); //Memory: Datenbreite pro Sample 01: 16 bit
DMA2_Stream0->CR |= (0b01 << DMA_SxCR_PSIZE_Pos); //Peripheral: Datenbreite pro Sample 01:16 bit
DMA2_Stream0->CR |= 0b00 << DMA_SxCR_DIR_Pos;  //Datenrichtung, 00: periphal to memory, 01:m2p


DMA2_Stream0->CR |= DMA_SxCR_TCIE; //transaction complete Interrupt enable

//DMA2_Stream0->CR |= DMA_SxCR_PFCTRL; //Flow Controller Bit 1: peripheral ist flow controller
//DMA2_Stream0->FCR &= ~DMA_SxFCR_DMDIS; //direct mode (ohne fifo)




 //////////////////I2S1 Init - eingang/////////////////////////////////////////

  SPI1->I2SCFGR |= SPI_I2SCFGR_I2SMOD; //1:i2s mode, 0:spi mode
  SPI1->I2SCFGR |= 0b11 << SPI_I2SCFGR_I2SCFG_Pos; //11: master mode receive 10:master tx 00:slave tx 01:slave rx
  SPI1->I2SCFGR |= SPI_I2SCFGR_CHLEN; //1:channel lenght 32 bit, 0: 16 bit
  SPI1->I2SPR = 1U << SPI_I2SPR_I2SDIV_Pos; //i2s freq. linear prescaler [7:0], faktor = (i2sdiv)*2
  //SPI1->I2SPR |= SPI_I2SPR_ODD; //1: prescaler faktor = (i2sdiv)*2+1
  SPI1->I2SPR |= SPI_I2SPR_MCKOE; //MCK output enable
  
  SPI1->CR2 |= SPI_CR2_RXDMAEN; //DMA rx aktivieren
  
  SPI1->I2SCFGR |= SPI_I2SCFGR_I2SE; //i2s enable
  SPI1->CR1 |= SPI_CR1_SPE; //spi1 enable

    //SPI1->I2SCFGR |= SPI_I2SCFGR_CKPOL; //0:L 1:H bei inaktivität 



    /////////DMA1 Initialisierung für I2s2 - ausgang/////////// f746:S.240 & 1102 im RM

DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR); //periph. adresse
DMA1_Stream4->M0AR = (uint32_t)&pingOUT; //memory 0 adresse
DMA1_Stream4->M1AR =(uint32_t)&pongOUT; //memory 1 adresse
DMA1_Stream4->NDTR = BUFSIZE;  //Anzahl Samples
DMA1_Stream4->CR |= DMA_SxCR_DBM; //double buffer mode
DMA1_Stream4->CR |= DMA_SxCR_CIRC; //circular mode
DMA1_Stream4->CR |= DMA_SxCR_MINC; //increment Memory Adresse
DMA1_Stream4->CR |= (0b01 << DMA_SxCR_MSIZE_Pos); //Memory: Datenbreite pro Sample 01: 16 bit
DMA1_Stream4->CR |= (0b01 << DMA_SxCR_PSIZE_Pos); //Peripheral: Datenbreite pro Sample 01:16 bit
DMA1_Stream4->CR |= 0b01 << DMA_SxCR_DIR_Pos;  //Datenrichtung, 00: periphal to memory, 01:m2p


DMA1_Stream4->CR |= DMA_SxCR_TCIE; //transaction complete Interrupt enable

//DMA1_Stream4->CR |= DMA_SxCR_PFCTRL; //Flow Controller Bit 1: peripheral ist flow controller
//DMA1_Stream4->CR |= 0b000 << DMA_SxCR_CHSEL; //channel select [2:0]
//DMA1_Stream4->FCR &= ~DMA_SxFCR_DMDIS; //direct mode (ohne fifo)



 //////////////////I2S2 Init - ausgang///////////////// Details: Seite 1129ff im Reference Manual
 //RCC->CFGR |= RCC_CFGR_I2SSRC;                //1:externer takt, 0:i2spll als source

  SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD; //1:i2s mode, 0:spi mode
  SPI2->I2SCFGR |= 0b10 << SPI_I2SCFGR_I2SCFG_Pos; //11: master mode receive 10:master tx 00:slave tx 01:slave rx
  SPI2->I2SCFGR |= SPI_I2SCFGR_CHLEN; //1:channel lenght 32 bit, 0: 16 bit
  SPI2->I2SPR = 4U << SPI_I2SPR_I2SDIV_Pos; //i2s freq. linear prescaler [7:0], faktor = (i2sdiv)*2
  //SPI2->I2SPR |= SPI_I2SPR_ODD; //1: prescaler faktor = (i2sdiv)*2+1
  //SPI2->I2SPR |= SPI_I2SPR_MCKOE; //MCK output enable 
  SPI2->CR2 |= SPI_CR2_TXDMAEN; //DMA tx aktivieren
  SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; //i2s enable
  SPI2->CR1 |= SPI_CR1_SPE; //spi2 enable

  //SPI1->I2SCFGR |= SPI_I2SCFGR_CKPOL; //0:L 1:H bei inaktivität


  /////////////////Interrupts/////////////////////////////

  NVIC_EnableIRQ(DMA2_Stream0_IRQn); //eingang
  NVIC_EnableIRQ(DMA1_Stream4_IRQn); //ausgang

  DMA1_Stream4->CR |= DMA_SxCR_EN; //stream enablen
  DMA2_Stream0->CR |= DMA_SxCR_EN; //stream enablen


  //GPIOC->MODER |= 0b0101010101010101;
  //GPIOC->ODR = 0b00000000;

  //for (i = 0; i < 1; i++) {
  //printf("%i \n",SystemCoreClock);
  //}

    //SPI1->DR = 0x0001;

 fftinit();  //instance passend zur bufferlänge wählen
 windowinit(); //fenster-funktion initialisieren

  while(1)          //Main-Schleife
	{
		while(!(RX_buffer_full & TX_buffer_empty)){}
        //GPIOC->ODR = 0b11111101;
                adc_read();  //poti einlesen
		process_buffer();
        //GPIOC->ODR = 0b11111110;
                }
        

}

