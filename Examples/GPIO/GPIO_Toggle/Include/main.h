

#ifndef __MAIN_H
#define __MAIN_H

/* Includes */
//#include "Board.h"
#include "apm32f00x_gpio.h"

#define SW_GPIO_PORT GPIOD
#define SWDIO_GPIO_PIN GPIO_PIN_1
#define SWCLK_GPIO_PIN GPIO_PIN_2

#define OUTPUT_PORT GPIOA
#define PUMP_GPIO_PIN GPIO_PIN_1
#define VALVE_GPIO_PIN GPIO_PIN_2
#define LCD_LIGHT_GPIO_PIN GPIO_PIN_3

#define BUTTONS_GPIO_PORT GPIOD
#define BUTTON1_GPIO_PIN GPIO_PIN_4
#define BUTTON2_GPIO_PIN GPIO_PIN_5
#define BUTTON3_GPIO_PIN GPIO_PIN_6

#define WR_GPIO_PIN GPIO_PIN_5
#define RD_GPIO_PIN GPIO_PIN_3
#define CS_GPIO_PIN GPIO_PIN_6
#define DATA_GPIO_PIN GPIO_PIN_4

#define PRESSURE_GPIO_PORT GPIOB
#define LOW_PRESSURE_SENS_GPIO_PIN GPIO_PIN_4    
#define HIGH_PRESSURE_SENS_GPIO_PIN GPIO_PIN_5

#define BuzzerLed_GPIO_PORT GPIOD
#define Buzzer_GPIO_PIN GPIO_PIN_1
#define Led_GPIO_PIN GPIO_PIN_2

#define CS_GPIO_PORT GPIOC
#define RD_GPIO_PORT GPIOC
#define WR_GPIO_PORT GPIOC
#define DATA_GPIO_PORT GPIOC

#define First_Filter_Add 		0x7800
#define Second_Filter_Add   0x7850
#define Third_Filter_Add	  0x7900
#define Fourth_Filter_Add   0x7950
#define Fifth_Filter_Add    0x7A00
#define GALON_Add           0x7A50
#define E5_Add							0x7B00
#define Cflow_Add						0x7B50

void ADCCalibration(void);
void ADCMeasure(void);
void Board_KeyInit(void);
void totalnumber(unsigned int order,unsigned int num);
void TMR4Init(void);
void Print_Display(uint8_t E,uint8_t Filter_Number,uint16_t Filter_Life);
void Screen_Water_Filling(void);



/***************  LCD KUTUPHANESINE AIT VERILER     *******************/

#define OSC_OFF    	0x00         				
#define OSC_ON    	0x01         				
#define DISP_OFF 		0x02								
#define DISP_ON   	0x03         				
#define COM_1_3__4  0x29  							// 1/3bias 4com


#define TIMER_DIS   0x04         				
#define WDT_DIS   	0x05         				
#define BUZZ_OFF  	0x08         				       				 
#define RC32K  			0X18         				// 
#define IRQ_DIS  		0X80        				// 


//#define 	VK1621_SEGNUM		32


#define VK1621_CS_H() 				GPIO_SetBit(CS_GPIO_PORT,CS_GPIO_PIN)       //VK1621_CS_IO = 1 	
#define VK1621_CS_L() 				GPIO_ClearBit(CS_GPIO_PORT,CS_GPIO_PIN)     //VK1621_CS_IO = 0 	

#define VK1621_RD_H() 				GPIO_SetBit(RD_GPIO_PORT,RD_GPIO_PIN)       //VK1621_RD_IO = 1 	
#define VK1621_RD_L() 				GPIO_ClearBit(RD_GPIO_PORT,RD_GPIO_PIN)     //VK1621_RD_IO = 0 	

#define VK1621_WR_H() 				GPIO_SetBit(WR_GPIO_PORT,WR_GPIO_PIN)       //VK1621_WR_IO = 1 	 
#define VK1621_WR_L() 				GPIO_ClearBit(WR_GPIO_PORT,WR_GPIO_PIN)     //VK1621_WR_IO = 0 	 

//#define VK1621_DATA_H()     GPIO_SetBit(DATA_GPIO_PORT,DATA_GPIO_PIN)   //VK1621_DAT_IO = 1 
//#define VK1621_DATA_L() 	  GPIO_SetBit(DATA_GPIO_PORT,DATA_GPIO_PIN)   //VK1621_DAT_IO = 0 

void VK1621_DATA_H(void);
void VK1621_DATA_L(void);

/* Exported functions ------------------------------------------------------- */

void Vk1621_Init(void);
void Vk1621_DisAll(unsigned char dat);
void WritenDataVk1621(unsigned char Addr,unsigned char *Databuf,unsigned char Cnt);
void Flash_Write2(void);
void Flow_Pulse(void);   //BUNLARI IMPLICIT OLDUKLARI ICIN YENI GUNCELLEMEDE TANITTIM
void TMR4Isr(void);			 //BUNLARI IMPLICIT OLDUKLARI ICIN YENI GUNCELLEMEDE TANITTIM


/***************  LCD KUTUPHANESINE AIT VERILER     *******************/

/** @addtogroup Examples
  @{
*/

/** @addtogroup GPIO_Toggle
  @{
*/
/**@} end of group GPIO_Toggle */
/**@} end of group Examples */

#endif /*__MAIN_H */
