#include "main.h"
#include "Board.h"
#include "apm32f00x_gpio.h"
#include "apm32f00x_rcm.h"
#include "apm32f00x_tmr4.h"
#include "apm32f00x_misc.h"
#include "apm32f00x_eint.h"
#include "apm32f00x_adc.h"
#include "apm32f00x_fmc.h"
#include <stdlib.h>
#include "math.h"
#include <stdbool.h>
#include "math.h"

uint8_t tick;
uint8_t tick2;  //dolum ekrani dng hizi iin eklendi.
uint32_t tick_wr;
#define VK1621_CLK 10 //delay_nus 50->10kHz 10->50kHz 5->100kHz
#define FLOWMETER_EXTI_LIN E 	 EINT_LINE0
uint32_t pulse;
uint8_t Mechanic_Valve_pulse;
uint8_t Water_Leak;
uint16_t adcData;
uint16_t voltage;
uint16_t binler_bas,yuzler_bas,onlar_bas,birler_bas;
uint16_t binler_kalan,yuzler_kalan,onlar_kalan;
uint16_t First_FLife,Second_FLife,Third_FLife,Fourth_FLife,Fifth_FLife;
uint16_t Const_Flow_Value;  //15.6.23
bool Tap,G,BellVib,Bell,Drop,Ripple,Ripple2,Dot,LPM,Wrench;                       //Tap=S1,G=S2,Bell_Vib=S4;Drop=S6,Ripple=S7,Ripple2=S8
bool Button1_state,Button2_state,Button3_state;
uint8_t Button1_Cnt,Button2_Cnt,Button3_Cnt;
bool Set_Mode=false;
bool SYSTEM_ON;
uint8_t screen_mode;
uint8_t Error;
uint8_t numbers_visibilty_timer;
bool filter_save;
bool water_filling;
uint8_t water_filling_cnt;
uint8_t screen_reset_cnt;
float water_filling_pulse;
float liter;
uint16_t Total_Water;
uint16_t GALON;
float FLOW;
uint8_t zeropagedigit,onedigit,twodigit,threedigit,fourdigit;
uint8_t onepagedigit,onedigit1,twodigit1,threedigit1,fourdigit1;
uint8_t onedigit2,twodigit2,threedigit2,fourdigit2;
uint8_t onedigit3,twodigit3,threedigit3,fourdigit3;
uint8_t onedigit4,twodigit4,threedigit4,fourdigit4;
uint8_t onedigit5,twodigit5,threedigit5,fourdigit5;
uint8_t onedigit6,twodigit6,threedigit6,fourdigit6;
uint8_t filling_symbol;
bool onetime_HighPress_set;
bool onetime_HighPress_clear;
bool onetime_waterLeak;
bool onetime_systemOff,onetime_systemOn;
uint8_t selenoid_check_time;
uint16_t selenoid_check_cnt; 
bool selenoid_error;
bool selenoid_buzzer;
uint8_t selenoid_buzzer_cnt;
uint32_t flowmeter_stability_pulse;
uint8_t error_visibility_timer;
uint8_t buzzer_error_timer;
bool E5_active;
bool onetime_E5_setting;
uint8_t E5_visibility_timer;
bool onetime_system_set;
bool read_button3;
bool onetime_eint;
bool E2_lock;
bool onetime_buzzer;
uint16_t selenoid_error_waiting_time;
bool selenoid_error_wait;
uint8_t button_2_3_cnt;
bool set_mode2_digit=false;
bool onetime_set_mode2_set;
uint8_t screen_mode_cnt;
bool screen_cnt_start;
uint32_t pulse_reg; 	
uint16_t GALON_REG;
bool screen_rolling;
bool screen_roll_start;
bool FLOW_start;  //acilirken ekran donmasini nledik.
bool E5_lock,E6_lock,E7_lock;
uint32_t E6_work_cnt;  // 3*60 = 180 3 saat siniri olacak
uint8_t mechanic_timecheck;
uint16_t mechanic_timecnt;
uint8_t E2_time_cnt;      //4.7.23
bool E2_Second_Control;		//4.7.23
bool E2_First_Control;   
bool E2_Time_Start;
uint8_t E6_Pulse;
uint8_t E6_pulse_reg;

bool E0_timer_start=false;
uint8_t E0_timer_cnt=0;
bool onetime_reset_after_E0;
bool onetime_E5_active;  //17.1.24

// YBS ve ABS switch'leri için 5 saniye kontrol değişkenleri - ayrı ayrı açılma/kapanma
uint8_t high_pressure_on_timer = 0;   // YBS açılma zamanlayıcısı (tetiklendiğinde)
uint8_t high_pressure_off_timer = 0;  // YBS kapanma zamanlayıcısı (tetiklenmediğinde) 
uint8_t low_pressure_on_timer = 0;    // ABS açılma zamanlayıcısı (tetiklendiğinde)
uint8_t low_pressure_off_timer = 0;   // ABS kapanma zamanlayıcısı (tetiklenmediğinde)
bool high_pressure_stable_on = false;  // YBS 5sn boyunca kesintisiz tetiklenmiş mi
bool high_pressure_stable_off = false; // YBS 5sn boyunca kesintisiz tetiklenmemiş mi
bool low_pressure_stable_on = false;   // ABS 5sn boyunca kesintisiz tetiklenmiş mi  
bool low_pressure_stable_off = false;  // ABS 5sn boyunca kesintisiz tetiklenmemiş mi
bool high_pressure_last_state = false; // Önceki YBS durumu
bool low_pressure_last_state = false;  // Önceki ABS durumu

/*unsigned char vk1621_dispram[VK1621_SEGNUM];

vk1621_dispram[VK1621_SEGNUM]=
{//com3   com2   com1     com0 
	1D,     1E,    1F,      1A,     //seg0   			vk1621_dispram[0]
	  ,     1C,    1G,      1B,     //seg1				vk1621_dispram[1]
	2D,     2E,    2F,      2A,     //seg2				vk1621_dispram[2]	
	  ,     2C,    2G,      2B,     //seg3				vk1621_dispram[3]
	3D,     3E,    3F,      3A,		  //seg4				vk1621_dispram[4]
	  ,     3C,    3G,      3B,			//seg5				vk1621_dispram[5]
}
*/
/*8led
     A
   F   B
     G
	 E   C
	   D
*/
unsigned char shuzi_zimo[15]= //
{
//0    1    2    3    4    5    6    7    8    9    -    L    o    H    i 
	0xF5,0x05,0xD3,0x97,0x27,0xB6,0xF6,0x15,0xF7,0xB7,0x02,0xE0,0xC6,0x67,0x05
};
unsigned char vk1621_addrbit=6;
unsigned char vk1621_segi,vk1621_comi;
unsigned char vk1621_maxcom;

unsigned char DATA[16];
unsigned char SEG[2];

void ADCInit(ADC_CHANNEL_T channel);
void ADC_GPIO_Init(ADC_CHANNEL_T channel);
void Delay(uint32_t count)
{
    volatile uint32_t delay = count;
    while(delay--);
}
int main(void)
{
	  /** Master clock frequency = 48 / 8 = 6MHZ */
    RCM_ConfigHIRCDiv(RCM_HIRC_DIV_8);
	  TMR4Init();
	
    GPIO_Config_T gpioConfig;
	
    gpioConfig.mode = GPIO_MODE_OUT_PP;
	  gpioConfig.pin =SWDIO_GPIO_PIN|SWCLK_GPIO_PIN;
    GPIO_Config(SW_GPIO_PORT, &gpioConfig);
	
	  gpioConfig.mode = GPIO_MODE_OUT_PP;
	  gpioConfig.pin =PUMP_GPIO_PIN|VALVE_GPIO_PIN|LCD_LIGHT_GPIO_PIN;
    GPIO_Config(OUTPUT_PORT, &gpioConfig);
	
		gpioConfig.mode = GPIO_MODE_IN_PU;
	  gpioConfig.pin =BUTTON1_GPIO_PIN|BUTTON2_GPIO_PIN|BUTTON3_GPIO_PIN;
    GPIO_Config(BUTTONS_GPIO_PORT, &gpioConfig);
	
	  gpioConfig.mode = GPIO_MODE_OUT_PP;
	  gpioConfig.pin =WR_GPIO_PIN|RD_GPIO_PIN|CS_GPIO_PIN|DATA_GPIO_PIN;
    GPIO_Config(GPIOC, &gpioConfig);
		
		gpioConfig.mode = GPIO_MODE_IN_PU;
	  gpioConfig.pin =LOW_PRESSURE_SENS_GPIO_PIN|HIGH_PRESSURE_SENS_GPIO_PIN;
    GPIO_Config(PRESSURE_GPIO_PORT, &gpioConfig);
		
		Board_KeyInit();		
		//NVIC_EnableIRQRequest(EINTC_IRQn, 0X02);

		GPIO_SetBit(OUTPUT_PORT,LCD_LIGHT_GPIO_PIN); 
				
		Vk1621_Init();
		
		DATA[0]=0x00;
		DATA[1]=0x01;
		DATA[2]=0x02;
		DATA[3]=0x03;
		DATA[4]=0x04;
		DATA[5]=0x05;
		DATA[6]=0x06;
		DATA[7]=0x07;
		DATA[8]=0x08;
		DATA[9]=0x09;
		DATA[10]=0x0A;
		DATA[11]=0x0B;
		DATA[12]=0x0C;
		DATA[13]=0x0D;
		DATA[14]=0x0E;
		DATA[15]=0x0F; 
		pulse=0;
		Water_Leak=0;
		adcData = 0;
    voltage = 0;
		ADCInit(ADC_CHANNEL_4);   		
    Tap=true;G=true;Drop=true;Ripple=true;Ripple2=true;BellVib=false;Bell=false,Dot=false,LPM=false,Wrench=false; //30.5.23
		
		if(*(__IO uint32_t *)First_Filter_Add>9999)First_FLife=1500;   
		else First_FLife=*(__IO uint32_t *)First_Filter_Add;   
		
	  if(*(__IO uint32_t *)Second_Filter_Add>9999)Second_FLife=3000;
		else Second_FLife=*(__IO uint32_t *)Second_Filter_Add;   
		
		if(*(__IO uint32_t *)Third_Filter_Add>9999)Third_FLife=3000;
		else Third_FLife=*(__IO uint32_t *)Third_Filter_Add;		
		
		if(*(__IO uint32_t *)Fourth_Filter_Add>9999)Fourth_FLife=6000;
		else Fourth_FLife=*(__IO uint32_t *)Fourth_Filter_Add;  
		
		if(*(__IO uint32_t *)Fifth_Filter_Add>9999)Fifth_FLife=3000;
		else Fifth_FLife=*(__IO uint32_t *)Fifth_Filter_Add;   
		
	  if(*(__IO uint32_t *)GALON_Add>9999){GALON=0;liter=0;}  //lenovo
		else {GALON=*(__IO uint32_t *)GALON_Add;liter=3.78*GALON;} //elle degistirilen galon tekrar artis yasarsa litre 1 e dnyordu. bu satiri ekledim. 3.5.23
		
		if(*(__IO uint32_t *)E5_Add>9999)E5_active=false;  //Ercan Bey talebi
		else E5_active=*(__IO uint32_t *)E5_Add;
		
		if(*(__IO uint32_t *)Cflow_Add>9999)Const_Flow_Value=2548;
		else Const_Flow_Value=*(__IO uint32_t *)Cflow_Add;

		
	  if(First_FLife<50||Second_FLife<50||Third_FLife<50||Fourth_FLife<50||Fifth_FLife<50)
		{
			Error=3;
			if(First_FLife<50)screen_mode=1;
			else if(Second_FLife<50)screen_mode=2;
			else if(Third_FLife<50)screen_mode=3;
 			else if(Fourth_FLife<50)screen_mode=4;
			else if(Fifth_FLife<50)screen_mode=5;
		}
		else {Error=4;screen_mode=0;buzzer_error_timer=0;onetime_buzzer=true;}   							//buzzer time 16.3.23 eklendi denenecek
		if(First_FLife==0||Second_FLife==0||Third_FLife==0||Fourth_FLife==0||Fifth_FLife==0)
		{
			Error=1;
			if(First_FLife==0)screen_mode=1;
			else if(Second_FLife==0)screen_mode=2;
			else if(Third_FLife==0)screen_mode=3;
			else if(Fourth_FLife==0)screen_mode=4;
			else if(Fifth_FLife==0)screen_mode=5;
		}
		else {if(Error!=3){Error=4;screen_mode=0;buzzer_error_timer=0;onetime_buzzer=true;}}   //buzzer time 16.3.23 eklendi denenecek
		
		FLOW=0;
		Total_Water=0;
		Button1_state=Button2_state=Button3_state=false;
		Button1_Cnt=Button2_Cnt=Button3_Cnt=0;
		Set_Mode=false;
		SYSTEM_ON=true;
		numbers_visibilty_timer=0;
		filter_save=false;
		water_filling=false;
		water_filling_cnt=0;
		water_filling_pulse=0;
		zeropagedigit=1;
		filling_symbol=0;
		
		////*******Total_Water_Digits*******////
		binler_kalan=GALON%1000;
		fourdigit=(GALON-binler_kalan)/1000;
		
		yuzler_kalan=binler_kalan%100;
		threedigit=(binler_kalan-yuzler_kalan)/100;
		
		onlar_kalan=yuzler_kalan%10;
		twodigit=(yuzler_kalan-onlar_kalan)/10;
		
		onedigit=GALON-1000*fourdigit-100*threedigit-10*twodigit;
		
				////*******Total_Filter1_Digits*******////
		binler_kalan=First_FLife%1000;
		fourdigit1=(First_FLife-binler_kalan)/1000;
		
		yuzler_kalan=binler_kalan%100;
		threedigit1=(binler_kalan-yuzler_kalan)/100;
		
		onlar_kalan=yuzler_kalan%10;
		twodigit1=(yuzler_kalan-onlar_kalan)/10;
		
		onedigit1=First_FLife-1000*fourdigit1-100*threedigit1-10*twodigit1;
		
						////*******Total_Filter2_Digits*******////
		binler_kalan=Second_FLife%1000;
		fourdigit2=(Second_FLife-binler_kalan)/1000;
		
		yuzler_kalan=binler_kalan%100;
		threedigit2=(binler_kalan-yuzler_kalan)/100;
		
		onlar_kalan=yuzler_kalan%10;
		twodigit2=(yuzler_kalan-onlar_kalan)/10;
		
		onedigit2=Second_FLife-1000*fourdigit2-100*threedigit2-10*twodigit2; 
		
								////*******Total_Filter3_Digits*******////
		binler_kalan=Third_FLife%1000;
		fourdigit3=(Third_FLife-binler_kalan)/1000;
		
		yuzler_kalan=binler_kalan%100;
		threedigit3=(binler_kalan-yuzler_kalan)/100;		
		onlar_kalan=yuzler_kalan%10;
		twodigit3=(yuzler_kalan-onlar_kalan)/10;
		onedigit3=Third_FLife-1000*fourdigit3-100*threedigit3-10*twodigit3; 
		
										////*******Total_Filter4_Digits*******////
		binler_kalan=Fourth_FLife%1000;
		fourdigit4=(Fourth_FLife-binler_kalan)/1000;
		
		yuzler_kalan=binler_kalan%100;
		threedigit4=(binler_kalan-yuzler_kalan)/100;
		
		onlar_kalan=yuzler_kalan%10;
		twodigit4=(yuzler_kalan-onlar_kalan)/10;
		
		onedigit4=Fourth_FLife-1000*fourdigit4-100*threedigit4-10*twodigit4; 
		
												////*******Total_Filter5_Digits*******////
		binler_kalan=Fifth_FLife%1000;
		fourdigit5=(Fifth_FLife-binler_kalan)/1000;
		
		yuzler_kalan=binler_kalan%100;
		threedigit5=(binler_kalan-yuzler_kalan)/100;
		
		onlar_kalan=yuzler_kalan%10;
		twodigit5=(yuzler_kalan-onlar_kalan)/10;
		
		onedigit5=Fifth_FLife-1000*fourdigit5-100*threedigit5-10*twodigit5;
				       
							       ////*******CONST_FLOW_Digits*******////
		
		//Const_Flow_Value=2548;
		binler_kalan=Const_Flow_Value%1000;
		fourdigit6=(Const_Flow_Value-binler_kalan)/1000;
		
		yuzler_kalan=binler_kalan%100;
		threedigit6=(binler_kalan-yuzler_kalan)/100;
		
		onlar_kalan=yuzler_kalan%10;
		twodigit6=(yuzler_kalan-onlar_kalan)/10;
		
		onedigit6=Const_Flow_Value-1000*fourdigit6-100*threedigit6-10*twodigit6;	
		
		tick2=0;
		onetime_HighPress_set=true;
		onetime_HighPress_clear=true;
		onetime_waterLeak=true;
		onetime_systemOff=true;   			 //TRUE idi. E2 hatasi su gitse bile sistem kapanmadan kaybolmamali diye false oldu.
		onetime_systemOn=false;
		selenoid_check_time=0;
		selenoid_check_cnt=0;
		selenoid_error=false;
		selenoid_buzzer=false;
		selenoid_buzzer_cnt=0;
		flowmeter_stability_pulse=0;
		error_visibility_timer=0;
		onetime_E5_setting=false;
		E5_visibility_timer=0;
		onetime_system_set=true;
		read_button3=true;
		onetime_eint=false;
		
	  read_button3=true;
		E2_lock=false;
		buzzer_error_timer=0;            //16.3.23
		onetime_buzzer=true;						 //16.3.23
		selenoid_error_waiting_time=0;   //16.3.23
		selenoid_error_wait=false;			 //16.3.23
		
		button_2_3_cnt=0;								 //20.3.23
		set_mode2_digit=false;
		onetime_set_mode2_set=true;
		onepagedigit=1;
		screen_reset_cnt=0;
		screen_mode_cnt=0;
		screen_cnt_start=false;
		pulse_reg=0;
		GALON_REG=0;  
		screen_rolling=false;
		screen_roll_start=false;
		FLOW_start=false;
		E5_lock=false;E6_lock=false;
		E6_work_cnt=0;  									  //15.6.23
		Mechanic_Valve_pulse=0;
		mechanic_timecheck=0;
		mechanic_timecnt=0;
		E7_lock=false;
		NVIC_DisableIRQRequest(EINTC_IRQn); //8.6.23
		EINT_ClearIntFlag(EINT_PORT_C); // Clear interrupt flag before disabling to prevent pending interrupts
		Vk1621_DisAll(0xFF);GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);Delay(200000);
		Vk1621_DisAll(0x00);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);
		
		E2_time_cnt=0;    //4.7.23
    E2_Second_Control=false;	 //4.7.23	
		E2_First_Control=false;
		E2_Time_Start=false;  //4.7.23
		
		E6_Pulse=0;
    E6_pulse_reg=0;
		
		E0_timer_start=false;
		E0_timer_cnt=0;
		onetime_reset_after_E0=true;   //7.12.23
		onetime_E5_active=true;  //17.1.24
		
		// YBS ve ABS switch kontrol değişkenlerini başlat - ayrı açılma/kapanma zamanlayıcıları
		high_pressure_on_timer = 0;
		high_pressure_off_timer = 0;
		low_pressure_on_timer = 0;
		low_pressure_off_timer = 0;
		high_pressure_stable_on = false;
		high_pressure_stable_off = false;
		low_pressure_stable_on = false;
		low_pressure_stable_off = false;
		high_pressure_last_state = GPIO_ReadInputBit(PRESSURE_GPIO_PORT, HIGH_PRESSURE_SENS_GPIO_PIN);
		low_pressure_last_state = GPIO_ReadInputBit(PRESSURE_GPIO_PORT, LOW_PRESSURE_SENS_GPIO_PIN);
		
   while(1)
   {		
		
		 
	//	 if(GPIO_ReadInputBit(PRESSURE_GPIO_PORT, LOW_PRESSURE_SENS_GPIO_PIN)){
   //      GPIO_ClearBit(OUTPUT_PORT, PUMP_GPIO_PIN);
    //   } 
		
		 if(!FLOW_start){Delay(5000);FLOW_start=true;EINT_ClearIntFlag(EINT_PORT_C);NVIC_EnableIRQRequest(EINTC_IRQn, 0X02);}
		 if(read_button3)
		 {
			 if(!GPIO_ReadInputBit(BUTTONS_GPIO_PORT,BUTTON3_GPIO_PIN))
			 {
					if(!Button3_state&&SYSTEM_ON)  //SYSTEM_ON
					{
						GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);Delay(5000);
						GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);
					}
					Button3_state=true;
			 }
			 else
			 {
				onetime_system_set=true;
				Button3_Cnt=0;
				if(SYSTEM_ON&&Button3_state)  
				{
					switch (screen_mode)
					{
						case 0:
						{ 
							if(Set_Mode||set_mode2_digit)  
							{
								switch(zeropagedigit)
								{
									case 1: onedigit++;if(onedigit>9)onedigit=0;
										break;
									case 2: twodigit++;if(twodigit>9)twodigit=0;
										break;
									case 3: threedigit++;if(threedigit>9)threedigit=0;
										break;
									case 4: fourdigit++;if(fourdigit>9)fourdigit=0;
										break;
								}
								GALON=fourdigit*1000+threedigit*100+twodigit*10+onedigit;
								filter_save=true;
							}
						}
							break;
						case 1:
						{ 
							if(Set_Mode)
							{
								//First_FLife=First_FLife+1500; 
								if((First_FLife>0||First_FLife==0)&&First_FLife<1500)First_FLife=1500; 
								else if(First_FLife>=1500&&First_FLife<3000)First_FLife=3000;
								else if(First_FLife>=3000&&First_FLife<4500)First_FLife=4500;
								else if(First_FLife>=4500&&First_FLife<6000)First_FLife=6000;
								else if(First_FLife>=6000){First_FLife=0;}
								if(First_FLife==0){Error=1;}
								else 
								{ Error=4;buzzer_error_timer=0;onetime_buzzer=true;Bell=false;BellVib=false;Wrench=false; 
									if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
								}
									binler_kalan=First_FLife%1000;
									fourdigit1=(First_FLife-binler_kalan)/1000;
									yuzler_kalan=binler_kalan%100;
									threedigit1=(binler_kalan-yuzler_kalan)/100;
									onlar_kalan=yuzler_kalan%10;
									twodigit1=(yuzler_kalan-onlar_kalan)/10;			
									onedigit1=First_FLife-1000*fourdigit1-100*threedigit1-10*twodigit1;
								filter_save=true;
							}
							// MANUAL ENTRY DISABLED
							//else if(set_mode2_digit)
							//{
							//	switch(onepagedigit)
							//	{
							//		case 1: onedigit1++;if(onedigit1>9)onedigit1=0;
							//			break;
							//		case 2: twodigit1++;if(twodigit1>9)twodigit1=0;
							//			break;
							//		case 3: threedigit1++;if(threedigit1>9)threedigit1=0;
							//			break;
							//		case 4: fourdigit1++;if(fourdigit1>9)fourdigit1=0;
							//			break;
							//	}
							//	First_FLife=fourdigit1*1000+threedigit1*100+twodigit1*10+onedigit1;
							//	if(First_FLife==0){Error=1;}
							//	else 
							//			 { Error=4;buzzer_error_timer=0;onetime_buzzer=true;    //13.6.23
							//				 if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//			   else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//				 else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//			 }
							//	filter_save=true;
							//}
						}
							break;
						case 2:
						{
							if(Set_Mode)
							{
								 //Second_FLife=Second_FLife+1500; 
								if((Second_FLife>0||Second_FLife==0)&&Second_FLife<1500)Second_FLife=1500;
								else if(Second_FLife>=1500&&Second_FLife<3000)Second_FLife=3000;
								else if(Second_FLife>=3000&&Second_FLife<4500)Second_FLife=4500;
								else if(Second_FLife>=4500&&Second_FLife<6000)Second_FLife=6000;
							  else if(Second_FLife>=6000){Second_FLife=0;}if(Second_FLife==0){Error=1;}
								else 
									{Error=4;buzzer_error_timer=0;onetime_buzzer=true;Bell=false;BellVib=false;Wrench=false; 
									 if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									 else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									 else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									}
								 	binler_kalan=Second_FLife%1000;
									fourdigit2=(Second_FLife-binler_kalan)/1000;
									yuzler_kalan=binler_kalan%100;
									threedigit2=(binler_kalan-yuzler_kalan)/100;
									onlar_kalan=yuzler_kalan%10;
									twodigit2=(yuzler_kalan-onlar_kalan)/10;
									onedigit2=Second_FLife-1000*fourdigit2-100*threedigit2-10*twodigit2; 
								filter_save=true;
							}
							// MANUAL ENTRY DISABLED
							//else if(set_mode2_digit)
							//{
							//	switch(onepagedigit)
							//	{
							//		case 1: onedigit2++;if(onedigit2>9)onedigit2=0;
							//			break;
							//		case 2: twodigit2++;if(twodigit2>9)twodigit2=0;
							//			break;
							//		case 3: threedigit2++;if(threedigit2>9)threedigit2=0;
							//			break;
							//		case 4: fourdigit2++;if(fourdigit2>9)fourdigit2=0;
							//			break;
							//	}
							//	Second_FLife=fourdigit2*1000+threedigit2*100+twodigit2*10+onedigit2;
							//	if(Second_FLife==0){Error=1;}
							//	else 
							//			{Error=4;buzzer_error_timer=0;onetime_buzzer=true;   //13.6.23
							//			 if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//		   else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//		   else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//			}
							//	filter_save=true;
							//}
						}
							break;
						case 3:
						{
							if(Set_Mode)
							{
								//Third_FLife=Third_FLife+1500; 
								if((Third_FLife>0||Third_FLife==0)&&Third_FLife<1500)Third_FLife=1500;
								else if(Third_FLife>=1500&&Third_FLife<3000)Third_FLife=3000;
								else if(Third_FLife>=3000&&Third_FLife<4500)Third_FLife=4500;
								else if(Third_FLife>=4500&&Third_FLife<6000)Third_FLife=6000;
						   	else if(Third_FLife>=6000){Third_FLife=0;}if(Third_FLife==0){Error=1;}
								else 
								{Error=4;buzzer_error_timer=0;onetime_buzzer=true;Bell=false;BellVib=false;Wrench=false; 
									if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
								}
								binler_kalan=Third_FLife%1000;
								fourdigit3=(Third_FLife-binler_kalan)/1000;		
								yuzler_kalan=binler_kalan%100;
								threedigit3=(binler_kalan-yuzler_kalan)/100;		
								onlar_kalan=yuzler_kalan%10;
								twodigit3=(yuzler_kalan-onlar_kalan)/10;
								onedigit3=Third_FLife-1000*fourdigit3-100*threedigit3-10*twodigit3; 
								filter_save=true;
							}
							// MANUAL ENTRY DISABLED
							//else if(set_mode2_digit)
							//{
							//	switch(onepagedigit)
							//	{
							//		case 1: onedigit3++;if(onedigit3>9)onedigit3=0;
							//			break;
							//		case 2: twodigit3++;if(twodigit3>9)twodigit3=0;
							//			break;
							//		case 3: threedigit3++;if(threedigit3>9)threedigit3=0;
							//			break;
							//		case 4: fourdigit3++;if(fourdigit3>9)fourdigit3=0;
							//			break;
							//	}
							//	Third_FLife=fourdigit3*1000+threedigit3*100+twodigit3*10+onedigit3;
							//	if(Third_FLife==0){Error=1;}
							//	else {Error=4;buzzer_error_timer=0;onetime_buzzer=true;   //13.6.23
							//			  	if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//			      else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//			      else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//			 }
							//	filter_save=true;
							//}
						}
							break;
						case 4:
						{
							if(Set_Mode)
							{
								//Fourth_FLife=Fourth_FLife+1500; 
								if((Fourth_FLife>0||Fourth_FLife==0)&&Fourth_FLife<1500)Fourth_FLife=1500;
								else if(Fourth_FLife>=1500&&Fourth_FLife<3000)Fourth_FLife=3000;
								else if(Fourth_FLife>=3000&&Fourth_FLife<4500)Fourth_FLife=4500;
								else if(Fourth_FLife>=4500&&Fourth_FLife<6000)Fourth_FLife=6000;
							  else if(Fourth_FLife>=6000){Fourth_FLife=0;}if(Fourth_FLife==0){Error=1;}
								else 
									{Error=4;buzzer_error_timer=0;onetime_buzzer=true;Bell=false;BellVib=false;Wrench=false; 
										if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									  else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									  else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									}	
								binler_kalan=Fourth_FLife%1000;
								fourdigit4=(Fourth_FLife-binler_kalan)/1000;
								yuzler_kalan=binler_kalan%100;
								threedigit4=(binler_kalan-yuzler_kalan)/100;
								onlar_kalan=yuzler_kalan%10;
								twodigit4=(yuzler_kalan-onlar_kalan)/10;
								onedigit4=Fourth_FLife-1000*fourdigit4-100*threedigit4-10*twodigit4; 
								filter_save=true;
							}
							// MANUAL ENTRY DISABLED
							//else if(set_mode2_digit)
							//{
							//	switch(onepagedigit)
							//	{
							//		case 1: onedigit4++;if(onedigit4>9)onedigit4=0;
							//			break;
							//		case 2: twodigit4++;if(twodigit4>9)twodigit4=0;
							//			break;
							//		case 3: threedigit4++;if(threedigit4>9)threedigit4=0;
							//			break;
							//		case 4: fourdigit4++;if(fourdigit4>9)fourdigit4=0;
							//			break;
							//	}
							//	Fourth_FLife=fourdigit4*1000+threedigit4*100+twodigit4*10+onedigit4;
							//	if(Fourth_FLife==0){Error=1;}
							//	else {Error=4;buzzer_error_timer=0;onetime_buzzer=true;  //13.6.23
							//				if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//		      else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//		      else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//			 }
							//	filter_save=true;
							//}
						}
							break;
						case 5:
						{
							if(Set_Mode)
							{
								//Fifth_FLife=Fifth_FLife+1500; 
								if((Fifth_FLife>0||Fifth_FLife==0)&&Fifth_FLife<1500)Fifth_FLife=1500;
								else if(Fifth_FLife>=1500&&Fifth_FLife<3000)Fifth_FLife=3000;
								else if(Fifth_FLife>=3000&&Fifth_FLife<4500)Fifth_FLife=4500;
								else if(Fifth_FLife>=4500&&Fifth_FLife<6000)Fifth_FLife=6000;
							  else if(Fifth_FLife>=6000){Fifth_FLife=0;}if(Fifth_FLife==0)Error=1;
								else 
									{Error=4;buzzer_error_timer=0;onetime_buzzer=true;Bell=false;BellVib=false;Wrench=false; 
										if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									  else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									  else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
									}
								binler_kalan=Fifth_FLife%1000;
								fourdigit5=(Fifth_FLife-binler_kalan)/1000;
								yuzler_kalan=binler_kalan%100;
								threedigit5=(binler_kalan-yuzler_kalan)/100;
								onlar_kalan=yuzler_kalan%10;
								twodigit5=(yuzler_kalan-onlar_kalan)/10;
								onedigit5=Fifth_FLife-1000*fourdigit5-100*threedigit5-10*twodigit5;
								filter_save=true;
							}
							// MANUAL ENTRY DISABLED
							//else if(set_mode2_digit)
							//{
							//	switch(onepagedigit)
							//	{
							//		case 1: onedigit5++;if(onedigit5>9)onedigit5=0;
							//			break;
							//		case 2: twodigit5++;if(twodigit5>9)twodigit5=0;
							//			break;
							//		case 3: threedigit5++;if(threedigit5>9)threedigit5=0;
							//			break;
							//		case 4: fourdigit5++;if(fourdigit5>9)fourdigit5=0;
							//			break;
							//	}
							//	Fifth_FLife=fourdigit5*1000+threedigit5*100+twodigit5*10+onedigit5;
							//	if(Fifth_FLife==0)Error=1;
							//	else 
							//		{Error=4;buzzer_error_timer=0;onetime_buzzer=true;  //13.6.23
							//			if(E5_lock){Error=5;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//		  else if(E6_lock){Error=6;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//		  else if(E7_lock){Error=7;selenoid_error=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; } //8.12.23
							//		}
							//	filter_save=true;
							//}
						}
							break;
						case 6:
						{
							// MANUAL ENTRY DISABLED
							//if(set_mode2_digit)
							//{
							//	switch(onepagedigit)
							//	{
							//		case 1: onedigit6++;if(onedigit6>9)onedigit6=0;
							//			break;
							//		case 2: twodigit6++;if(twodigit6>9)twodigit6=0;
							//			break;
							//		case 3: threedigit6++;if(threedigit6>9)threedigit6=0;
							//			break;
							//		case 4: fourdigit6++;if(fourdigit6>9)fourdigit6=0;
							//			break;
							//	}
							//	Const_Flow_Value=fourdigit6*1000+threedigit6*100+twodigit6*10+onedigit6;
							//	if(Const_Flow_Value==0){Const_Flow_Value=1;onedigit6=1;}
							//	filter_save=true;
							//}
						}
							break;
					}
				}  
				Button3_state=false;
			 }
		 }
		 if(SYSTEM_ON)
		 {
				 //50 galon kontrol set mode harici olarak atanabilir.
				 if(First_FLife<50||Second_FLife<50||Third_FLife<50||Fourth_FLife<50||Fifth_FLife<50)
				 {
					 if(First_FLife==0||Second_FLife==0||Third_FLife==0||Fourth_FLife==0||Fifth_FLife==0)
					 {
						 if(Error!=5&&Error!=0&&Error!=2)Error=1; 
					 }
					 else 
					 {
						 //buzzer_error_timer=0;
						 if(!E7_lock&&!E6_lock&&!E5_lock&&Error!=0&&Error!=2){Error=3;buzzer_error_timer=0;}  //8.12.23 if statement içerigi degisti
					 }
				 }
				 else {if(Error==3){Error=4;buzzer_error_timer=0;onetime_buzzer=true;}}  //27.4.23 //13.6.23
				 if(onetime_systemOn){GPIO_SetBit(OUTPUT_PORT,LCD_LIGHT_GPIO_PIN);Vk1621_DisAll(0xFF);GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);Delay(200000);
															Vk1621_DisAll(0x00);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);onetime_eint=true;onetime_systemOn=false;}
				 if(onetime_E5_setting)
				 {
					 Dot=false;  //31.5.23
					 totalnumber(1,16);totalnumber(2,5);totalnumber(3,11);totalnumber(4,0);
					 if(E5_active){totalnumber(5,14);totalnumber(6,10);Flash_Write2();}
					 else{Bell=false;BellVib=false;Wrench=false;totalnumber(5,15);totalnumber(6,15);if(Error==5)Error=4;Flash_Write2();} 
				 }
				 else 
				 {
					 switch (screen_mode)
					 {
						 case 0:   //toplam geen su miktari galon cinsinden
						 {
							 G=true;Dot=false;LPM=false;
							 if((Set_Mode||set_mode2_digit)&&numbers_visibilty_timer==1)  
							 {
								 switch(zeropagedigit)
								 {
									 case 1:totalnumber(6,10);break;
									 case 2:totalnumber(5,10);break;
									 case 3:totalnumber(4,10);break;
									 case 4:totalnumber(3,10);break;
								 }
							 }
							 if(numbers_visibilty_timer==0)Print_Display(Error,0,GALON);    //number visibility ile error visibility bakilcak
							 if(Error!=0&&screen_rolling)Screen_Water_Filling();																									//E0 SEBEKE SUYU KESIK, E1 FILTRE MR, E2 SU KAAGI,E5 SELENOID  
						 }break;
						 case 1: 
						 {
							 G=true;Dot=false;LPM=false;
							 if(numbers_visibilty_timer==1){
								 if(Set_Mode){ totalnumber(3,10);totalnumber(4,10);totalnumber(5,10);totalnumber(6,10);}
								 else if(set_mode2_digit)
								 {
									 switch(onepagedigit)
									 {
										 case 1:totalnumber(6,10);break;
										 case 2:totalnumber(5,10);break;
										 case 3:totalnumber(4,10);break;
										 case 4:totalnumber(3,10);break;
									 }
								 }
							 }
							 if(numbers_visibilty_timer==0)Print_Display(Error,1,First_FLife);
							 if(Error!=0&&screen_rolling)Screen_Water_Filling();		
						 }break;
						 case 2: 
						 {
							 G=true;Dot=false;LPM=false;
							 if(numbers_visibilty_timer==1)
							 {
								 if(Set_Mode){totalnumber(3,10);totalnumber(4,10);totalnumber(5,10);totalnumber(6,10);}
								 else if(set_mode2_digit)
								 {
									 switch(onepagedigit)
									 {
										 case 1:totalnumber(6,10);break;
										 case 2:totalnumber(5,10);break;
										 case 3:totalnumber(4,10);break;
										 case 4:totalnumber(3,10);break;
									 }
								 }
							 }							 
							 if(numbers_visibilty_timer==0)Print_Display(Error,2,Second_FLife);
							 if(Error!=0&&screen_rolling)Screen_Water_Filling();		
						 }break;
						 case 3:
						 {
							 G=true;Dot=false;LPM=false;
							 if(numbers_visibilty_timer==1)
							 {
								 if(Set_Mode){totalnumber(3,10);totalnumber(4,10);totalnumber(5,10);totalnumber(6,10);}	 
								 else if(set_mode2_digit)
								 {
									 switch(onepagedigit)
									 {
										 case 1:totalnumber(6,10);break;
										 case 2:totalnumber(5,10);break;
										 case 3:totalnumber(4,10);break;
										 case 4:totalnumber(3,10);break;
									 }
								 }
							 }
							 if(numbers_visibilty_timer==0)Print_Display(Error,3,Third_FLife);
							 if(Error!=0&&screen_rolling)Screen_Water_Filling();		
						 }break;
						 case 4:
						 {
							 G=true;Dot=false;LPM=false;
							 if(numbers_visibilty_timer==1)
							 {
								 if(Set_Mode){totalnumber(3,10);totalnumber(4,10);totalnumber(5,10);totalnumber(6,10);}
								 else if(set_mode2_digit)
								 {
									 switch(onepagedigit)
									 {
										 case 1:totalnumber(6,10);break;
										 case 2:totalnumber(5,10);break;
										 case 3:totalnumber(4,10);break;
										 case 4:totalnumber(3,10);break;
									 }
								 }
							 }
							 if(numbers_visibilty_timer==0)Print_Display(Error,4,Fourth_FLife);
							 if(Error!=0&&screen_rolling)Screen_Water_Filling();		
						 }break;
						 case 5:
						 {
							 G=true;Dot=false;LPM=false;
							 if(numbers_visibilty_timer==1)
							 {
								 if(Set_Mode){totalnumber(3,10);totalnumber(4,10);totalnumber(5,10);totalnumber(6,10);}
								 else if(set_mode2_digit)
								 {
									 switch(onepagedigit)
									 {
										 case 1:totalnumber(6,10);break;
										 case 2:totalnumber(5,10);break;
										 case 3:totalnumber(4,10);break;
										 case 4:totalnumber(3,10);break;
									 }
								 }
							 }
							 if(numbers_visibilty_timer==0)Print_Display(Error,5,Fifth_FLife);
							 if(Error!=0&&screen_rolling)Screen_Water_Filling();		
						 }break;
						 case 6:   //anlik litre/dk ile debi miktarini veren ekran.
						 {
							 G=false;LPM=true;
							 if(Set_Mode){Dot=false;Print_Display(Error,15,Const_Flow_Value);}
							 else if(set_mode2_digit)
							 {
								 Dot=false;
								 if(numbers_visibilty_timer==1)
								 {
									 switch(onepagedigit)
									 {
										 case 1:totalnumber(6,10);break;
										 case 2:totalnumber(5,10);break;
										 case 3:totalnumber(4,10);break;
										 case 4:totalnumber(3,10);break;
									 }			 
								 }
								 else if(numbers_visibilty_timer==0)Print_Display(Error,15,Const_Flow_Value);
							 }
							 else{Dot=true;Print_Display(Error,15,FLOW);}
							 if(Error!=0&&screen_rolling)Screen_Water_Filling();		
						 }break;
					 }
				 }
			 	 if(E5_active&&E5_lock&&selenoid_error&&!selenoid_error_wait)   //&&Error==5 
				 {
					 if(selenoid_buzzer_cnt>78) 
					 {
							GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);
							selenoid_check_time=0;
						  selenoid_buzzer=false;     
							//selenoid_buzzer_cnt=0;		
							selenoid_error_wait=true;   //16.3.23	    60 dk da bir kontrol icin eklendi.
						 selenoid_error_waiting_time=0;
					 }
					 else selenoid_buzzer=true;
				 }
				 else if(E5_active&&(E6_lock||E7_lock)&&!selenoid_error_wait)  //15.6.23   //&&Error==6  
				 {
					 if(selenoid_buzzer_cnt>78)  
					 {
							GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);
						 	//selenoid_check_time=0;
						  selenoid_buzzer=false;     
							//selenoid_buzzer_cnt=0;		
							selenoid_error_wait=true; 
							selenoid_error_waiting_time=0;						 
					 }
					 else selenoid_buzzer=true;
				 }
				 ADCMeasure();
				 if(!GPIO_ReadInputBit(BUTTONS_GPIO_PORT,BUTTON1_GPIO_PIN))
				 {
					 	if(!Button1_state)
						{
							GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);Delay(5000);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);
						}
						if(Button1_state==false&&!Set_Mode&&!set_mode2_digit)
						{
							if(!Set_Mode){screen_mode++;screen_mode_cnt=0;screen_cnt_start=true;}
							if(screen_mode==7)screen_mode=0;
						}
						else if(Button1_state==false)
						{
							if(Set_Mode)
							{
								zeropagedigit++;if(zeropagedigit>4)zeropagedigit=1;
							}
							// MANUAL ENTRY DISABLED
							//else if(set_mode2_digit)   //20.3.23
							//{ zeropagedigit++;if(zeropagedigit>4)zeropagedigit=1;
							//	onepagedigit++;if(onepagedigit>4)onepagedigit=1;;
							//} 
						}
						Button1_state=true;
				 }
				 else 
				 {
					  if(!onetime_E5_active)onetime_E5_active=true;
						Button1_Cnt=0;
						Button1_state=false;
				 }
				 if(!GPIO_ReadInputBit(BUTTONS_GPIO_PORT,BUTTON2_GPIO_PIN))
				 {
					 	if(!Button2_state)
						{
							GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);Delay(5000);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);
							if(Set_Mode){onepagedigit=1;numbers_visibilty_timer=0;Set_Mode=false;filter_save=true;onetime_waterLeak=true;} //onetime_waterLeak=true; eklendi 6.12.23
							// MANUAL ENTRY DISABLED
							//else if(set_mode2_digit){onepagedigit=1;numbers_visibilty_timer=0;set_mode2_digit=false;onetime_set_mode2_set=true;filter_save=true;onetime_waterLeak=true;}//onetime_waterLeak=true; eklendi 6.12.23
						}
						Button2_state=true;
				 }
				 else 
				 {
						Button2_Cnt=0;
						if(filter_save)
						{
							filter_save=false;
							Flash_Write2();
						}			
						Button2_state=false;
				 }
				 if(Water_Leak==1)
				 {
					 if(Error!=0)  //Error!=0&& eklendi  6.12.23
					 { 
						 if(Error!=1&&Error!=2)Error=2;  //8.12.23
						 if(onetime_waterLeak)   //6.12.23 iç brace alindi
						 {
						 		 onetime_waterLeak=false;
								 GPIO_ClearBit(OUTPUT_PORT,PUMP_GPIO_PIN);
								 GPIO_ClearBit(OUTPUT_PORT,VALVE_GPIO_PIN);
								 Bell=true;BellVib=true;							        //Su kacagi algilandi. //Error=2; yukari alindi
								 E6_work_cnt=0;           //15.6.23
								 water_filling=false;													//burada dnen ekran kalabilir.filling_symbol oldugu degerde kalir.
								 water_filling_pulse=0;
								 water_filling_cnt=0;
								 FLOW=0;
								 screen_mode=0;       //27-2-23
								 //onetime_buzzer=true;  //4.7.23
								 //buzzer_error_timer=0; //4.7.23
								 if(!E5_lock&&!E6_lock&&!E7_lock){onetime_buzzer=true;buzzer_error_timer=0;}  //8.12.23
								 selenoid_buzzer_cnt=0;
								 selenoid_error_wait=false;
								 selenoid_error_waiting_time=0;
								 ///onetime_HighPress_set=true;??
						 }
					 }
				 } 
				 // Yüksek basınç kontrolü - YBS 5 saniye boyunca kesintisiz tetiklenmeli
				 if(high_pressure_stable_on && Error!=0)  // YBS 5sn boyunca aktif ise kapat
				 {
					 if(onetime_HighPress_clear)
					 {
						 GPIO_ClearBit(OUTPUT_PORT,PUMP_GPIO_PIN);
						 GPIO_ClearBit(OUTPUT_PORT,VALVE_GPIO_PIN);
						 water_filling=false;
						 E6_work_cnt=0;           //15.6.23
						 water_filling_pulse=0;
						 water_filling_cnt=0;
						 pulse=0;
						 selenoid_check_time=0;
						 FLOW=0;
						 // Avoid forcing screen to 0 during configuration modes
						 if(!Set_Mode && !set_mode2_digit){
						 	screen_mode=0;   //5.6.23
						 	screen_cnt_start=true; // schedule normal return
						 }
						 onetime_HighPress_set=true;
						 onetime_HighPress_clear=false;
						 if(Error==0){Error=4;buzzer_error_timer=0;onetime_buzzer=true; }
						 flowmeter_stability_pulse=0;  //EGER DOLUM AKTIF ANCAK FLOW METRO DNMYORSA SELENOID ERRORA DSER. ANCAK BASTA DNP SONRA ARIZALANMIS ISE HATANIN GZDEN KAMAMASI
																					 //ICIN DOLUM SONRASI BU FLOWMETRE PULSE KONTROL DEGERI SIFIRLANIR. TEKRAR ARTMASI ANCAK FLOWMETRE INTERRUPTI ILE MUMKUNDUR. 
																					 //kapanis kisa periyot ile oluyor.// 
						 
						 // YBS ile kapama işlemi tamamlandı, zamanlayıcıları sıfırla
						 high_pressure_on_timer = 0;
						 high_pressure_stable_on = false;
					 }
				 }
				 else 
 				 {
						// Pompa açılma kontrolü - hem YBS hem ABS 5sn boyunca tetiklenmemeli 
						if(Error!=2 && Error!=0 && // E0 hatasi kontrolü
						   high_pressure_stable_off && // YBS 5sn boyunca tetiklenmemiş
						   low_pressure_stable_off && // ABS 5sn boyunca tetiklenmemiş
						   !E7_lock && !E6_lock && !E5_lock && Water_Leak==0 && onetime_HighPress_set) 
						{
							onetime_HighPress_clear=true;
							Ripple=true;
							Ripple2=true;
							GPIO_SetBit(OUTPUT_PORT,PUMP_GPIO_PIN);    // Burada pompa çalistiriliyor
							GPIO_SetBit(OUTPUT_PORT,VALVE_GPIO_PIN);   // Burada valf açiliyor
							water_filling=true;
							//screen_mode=6;   // commented here for testing  							  
							onetime_HighPress_set=false;
							
							// Açılma işlemi tamamlandı, zamanlayıcıları sıfırla
							high_pressure_off_timer = 0;
							low_pressure_off_timer = 0;
							high_pressure_stable_off = false;
							low_pressure_stable_off = false;
						}				
						if(selenoid_check_time==0){if(flowmeter_stability_pulse==0)selenoid_check_time=1;}
						else if(selenoid_check_time==2) //&&!selenoid_error_wait
									{if(Error!=0&&!E2_lock&&!E7_lock&&!E6_lock&&!E5_lock){selenoid_error_wait=false;selenoid_buzzer_cnt=0;Error=5;selenoid_error=true;E5_lock=true;GPIO_ClearBit(OUTPUT_PORT,PUMP_GPIO_PIN);GPIO_ClearBit(OUTPUT_PORT,VALVE_GPIO_PIN);}selenoid_check_time=0;}		//selenoid_check_time=0;  ek. Error!=0 7.12.23 
				 }
				// ...
				 // E0 hatası kontrolü - ABS 5 saniye boyunca kesintisiz tetiklenmeli
				 if(low_pressure_stable_on) // ABS 5sn boyunca aktif ise E0 hatası
				 {
						 if(Error!=2&&Error!=0) //commented here for testing//&&!E5_lock&&!E6_lock&&!E7_lock - added check to prevent reset during configuration
						 {
						 	 // 5 saniye boyunca alçak basınç varsa E0 hatası
								E0_timer_start=false;Error=0;Bell=true;BellVib=true;
								GPIO_ClearBit(OUTPUT_PORT,PUMP_GPIO_PIN);
								GPIO_ClearBit(OUTPUT_PORT,VALVE_GPIO_PIN);
								E6_work_cnt=0;
								//Set_Mode=false; // commented here for testing
								//numbers_visibilty_timer=0; // commented here for testing
								//screen_mode=0;        // E0 hatası ekranda görünsün - removed immediate jump to screen 0
								screen_mode_cnt=0;screen_cnt_start=true;  // Start 6-second countdown before returning to screen 0
								onetime_buzzer=true;
								buzzer_error_timer=0;
								E0_timer_cnt=0; // Bunu sifirlamak yine de iyi bir aliskanlik
								onetime_reset_after_E0=true;
								
								// ABS ile E0 hatası verildi, zamanlayıcıyı sıfırla
								low_pressure_on_timer = 0;
								low_pressure_stable_on = false;
						 }
				 }
				 else
				 {
					 E0_timer_start=false;E0_timer_cnt=0;
					 // E0 hatası sadece ABS 5sn boyunca tetiklenmediğinde temizlensin
					 if(Error==0&&low_pressure_stable_off)
					 {
						 Error=4;buzzer_error_timer=0;onetime_buzzer=true;Bell=false;BellVib=false;
						 if(!onetime_HighPress_set)onetime_HighPress_set=true;
						 //screen_mode=0;  // Removed immediate jump - use 6-second countdown
						 screen_mode_cnt=0;screen_cnt_start=true;  // Start 6-second countdown before returning to screen 0
						 // E0 temizlendiğinde ABS off zamanlayıcısını sıfırla
						 low_pressure_off_timer = 0;
						 low_pressure_stable_off = false;
					 }
				  //7.12.23 buradan brace sonuna kadar eklendi çünkü E0 e5-e6-e7 ye göre önceliklendirildi. e0 gidince tekrar olusabilmeleri için parametreler sifirlandi.
					if(onetime_reset_after_E0)
					{
						 selenoid_check_cnt=0;  
						 E6_Pulse=0;
						 E6_pulse_reg=0;
						 Mechanic_Valve_pulse=0;  
						 mechanic_timecheck=0;
						 mechanic_timecnt=0;
						 selenoid_check_time=0;flowmeter_stability_pulse=0;selenoid_error_wait=false;selenoid_error=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; 
						 if(E5_lock){Error=5;selenoid_error=true;}       //E5_lock=false; 8.12.23
						 else if(E6_lock){Error=6;selenoid_error=true;}  //E6_lock=false; 8.12.23
						 else if(E7_lock){Error=7;selenoid_error=true;}  //E7_lock=false;  8.12.23
						 E6_work_cnt=0;
						 onetime_reset_after_E0=false;
					}
				 }
		 }
		 else 
		 {
			 //if(onetime_systemOff)
			 //{
				 read_button3=false;
				 GPIO_ClearBit(OUTPUT_PORT,LCD_LIGHT_GPIO_PIN); 
				 //Vk1621_DisAll(0x00);
				 WritenDataVk1621(16,&DATA[0],1);WritenDataVk1621(17,&DATA[0],1);WritenDataVk1621(18,&DATA[0],1);Drop=false;Ripple=false;Ripple2=false;Tap=false;G=false;Bell=false;BellVib=false;
				 totalnumber(1,10);totalnumber(2,11);totalnumber(3,10);totalnumber(4,10);totalnumber(5,10);totalnumber(6,10);totalnumber(7,0);totalnumber(8,3);
				 WritenDataVk1621(19,&DATA[0],1); // WRENCH KAPAMA 31.5.23
				 LPM=false;    										// 31.5.23
				 GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);
				 GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);
				 onetime_E5_active=true;  //17.1.24
				 onetime_HighPress_set=true;
				 GPIO_ClearBit(OUTPUT_PORT,VALVE_GPIO_PIN);
				 GPIO_ClearBit(OUTPUT_PORT,PUMP_GPIO_PIN);
				 water_filling=false;
				 onetime_waterLeak=true;
				 Water_Leak=0;//28.3.23
				 tick2=0;Error=4;
				 buzzer_error_timer=0;
				 Set_Mode=false;
				 Wrench=false;    // E7 sonrasi eklendi 21.6.23
				 E2_lock=false;
				 E2_Second_Control=false; // 4.7.23
				 E2_time_cnt=0;  //4.7.23
				 E2_First_Control=false;
		     E2_Time_Start=false;  //4.7.23
				 onetime_systemOff=false;
				 onetime_buzzer=true;
				 selenoid_check_cnt=0;  //23.6.23
				 E6_Pulse=0;
				 E6_pulse_reg=0;
				 Mechanic_Valve_pulse=0;  //21.6.23
				 mechanic_timecheck=0;
				 mechanic_timecnt=0;
				 selenoid_check_time=0;flowmeter_stability_pulse=0;selenoid_error_wait=false;selenoid_error=false;selenoid_buzzer_cnt=0;selenoid_buzzer=false; 
				 E5_lock=false; 
				 E6_work_cnt=0;    				 //15.6.23
				 E6_lock=false;
				 E7_lock=false;
				 E6_work_cnt=0;
				 E0_timer_start=false;
				 E0_timer_cnt=0;
				 onetime_reset_after_E0=true;   //7.12.23
				 
				 // Sistem kapalı iken basınç switch zamanlayıcılarını sıfırla - ayrı açılma/kapanma
				 high_pressure_on_timer = 0;
				 high_pressure_off_timer = 0;
				 low_pressure_on_timer = 0;
				 low_pressure_off_timer = 0;
				 high_pressure_stable_on = false;
				 high_pressure_stable_off = false;
				 low_pressure_stable_on = false;
				 low_pressure_stable_off = false;
				 
			   Delay(5000);read_button3=true;
			 //}
		 }
		 
	}
}
void Screen_Water_Filling()
{
	switch(filling_symbol)
	 {
		 case 1:{WritenDataVk1621(16,&DATA[2],1);WritenDataVk1621(17,&DATA[6],1);WritenDataVk1621(18,&DATA[5],1);Drop=true;Ripple=false;Ripple2=false;}break;
		 case 2:{WritenDataVk1621(16,&DATA[4],1);WritenDataVk1621(17,&DATA[12],1);WritenDataVk1621(18,&DATA[9],1);Drop=true;Ripple=true;Ripple2=false;}break;
		 case 3:{WritenDataVk1621(16,&DATA[8],1);WritenDataVk1621(17,&DATA[5],1);WritenDataVk1621(18,&DATA[3],1);Drop=true;Ripple=true;Ripple2=true;}break;
			 
		 case 4:{WritenDataVk1621(16,&DATA[2],1);WritenDataVk1621(17,&DATA[6],1);WritenDataVk1621(18,&DATA[5],1);Drop=false;Ripple=false;Ripple2=false;}break;
		 case 5:{WritenDataVk1621(16,&DATA[4],1);WritenDataVk1621(17,&DATA[12],1);WritenDataVk1621(18,&DATA[9],1);Drop=true;Ripple=false;Ripple2=false;}break;
		 case 6:{WritenDataVk1621(16,&DATA[8],1);WritenDataVk1621(17,&DATA[5],1);WritenDataVk1621(18,&DATA[3],1);Drop=true;Ripple=true;Ripple2=false;}break;
			 
		 case 7:{WritenDataVk1621(16,&DATA[2],1);WritenDataVk1621(17,&DATA[6],1);WritenDataVk1621(18,&DATA[5],1);Drop=true;Ripple=true;Ripple2=true;}break;
		 case 8:{WritenDataVk1621(16,&DATA[4],1);WritenDataVk1621(17,&DATA[12],1);WritenDataVk1621(18,&DATA[9],1);Drop=false;Ripple=false;Ripple2=false;}break;
		 case 9:{WritenDataVk1621(16,&DATA[8],1);WritenDataVk1621(17,&DATA[5],1);WritenDataVk1621(18,&DATA[3],1);Drop=true;Ripple=false;Ripple2=false;}break;
			 
		 case 10:{WritenDataVk1621(16,&DATA[2],1);WritenDataVk1621(17,&DATA[6],1);WritenDataVk1621(18,&DATA[5],1);Drop=true;Ripple=true;Ripple2=false;}break;
		 case 11:{WritenDataVk1621(16,&DATA[4],1);WritenDataVk1621(17,&DATA[12],1);WritenDataVk1621(18,&DATA[9],1);Drop=true;Ripple=true;Ripple2=true;}break;
		 case 12:{WritenDataVk1621(16,&DATA[8],1);WritenDataVk1621(17,&DATA[5],1);WritenDataVk1621(18,&DATA[3],1);Drop=false;Ripple=false;Ripple2=false;}break;
	 }
}
void Print_Display(uint8_t E,uint8_t Filter_Number,uint16_t Filter_Life) 
{
	if(E==0)
	{
		Wrench=false;  //3.7.23 
		totalnumber(7,14);
		totalnumber(8,0);
		if(!screen_rolling){Ripple=true;Ripple2=true;Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);}// the drop and ripple symbols are activated
		//if(!Set_Mode)//commenting out for testing
		//{
			if(error_visibility_timer==0){Tap=false;Bell=false;BellVib=false;GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN)  ;}
			else if(error_visibility_timer==1){Tap=true;Bell=true;BellVib=true;GPIO_SetBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(buzzer_error_timer!=0)GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);}
		//}
	}
	else if(E==1)
	{
		Tap=true;
		totalnumber(7,14);
		totalnumber(8,1);
		if(!screen_rolling){Ripple=true;Ripple2=true;Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);}
		if(!Set_Mode)
		{
		  if(error_visibility_timer==0){Bell=false;BellVib=false;Wrench=false;GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);}
		  else if(error_visibility_timer==1){Bell=true;BellVib=true;Wrench=true;GPIO_SetBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(buzzer_error_timer!=0&&!Button3_state)GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);}		
		}
	}
	else if(E==2)
	{
		Tap=true;
		totalnumber(7,14);
		//totalnumber(8,2);
		if(!screen_rolling){Ripple=Ripple2=Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);}
		if(!Set_Mode)
		{
			if(error_visibility_timer==0){totalnumber(8,2);Bell=false;BellVib=false;Wrench=false;GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);}
			else if(error_visibility_timer==1){if(E5_lock)totalnumber(8,5);else if(E6_lock)totalnumber(8,6);else if(E7_lock)totalnumber(8,7);
																					Bell=true;BellVib=true;Wrench=true;GPIO_SetBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(!Button3_state&&((buzzer_error_timer!=0)||(selenoid_buzzer_cnt<78&&(E5_lock||E6_lock||E7_lock))))GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);}	
		}
	}
	else if(E==5)
	{
		Tap=true;
		totalnumber(7,14);
		totalnumber(8,5);
		if(!screen_rolling){Ripple=Ripple2=Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);}
		if(!Set_Mode)   //15.6.23
		{
		  if(error_visibility_timer==0){Bell=false;BellVib=false;Wrench=false;GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(selenoid_buzzer_cnt<78)GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN); } 
		  else if(error_visibility_timer==1){Bell=true;BellVib=true;Wrench=true;GPIO_SetBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(selenoid_buzzer_cnt<78)GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN); }		
		}
	}
	else if(E==3)  //2. ve 3. segmenti tekrar yakmak gerekebilir.  //Filtre mr sadece zil uyarisi.
	{
  	Tap=true;
		Wrench=false;
		GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);
		if(error_visibility_timer==1){Bell=false;BellVib=false;}
		else if(error_visibility_timer==0){Bell=true;BellVib=true;}
		if(!screen_rolling)
		{Ripple=true;Ripple2=true;Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);}
		totalnumber(7,0);
		totalnumber(8,3);
	} 
	else if(E==6)   //15.6.23
	{
		Tap=true;
		totalnumber(7,14);
		totalnumber(8,6);
		if(!screen_rolling){Ripple=Ripple2=Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);}
		if(!Set_Mode)   //15.6.23
		{
		  if(error_visibility_timer==0){Bell=false;BellVib=false;Wrench=false;GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(selenoid_buzzer_cnt<78)GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);}  
		  else if(error_visibility_timer==1){Bell=true;BellVib=true;Wrench=true;GPIO_SetBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(selenoid_buzzer_cnt<78)GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);}		
		}
	}
	else if(E==7)   //15.6.23
	{
		Tap=true;
		totalnumber(7,14);
		totalnumber(8,7);
		if(!screen_rolling){Ripple=Ripple2=Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);}
		if(!Set_Mode)   //15.6.23
		{
		  if(error_visibility_timer==0){Bell=false;BellVib=false;Wrench=false;GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(selenoid_buzzer_cnt<78)GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);} 
		  else if(error_visibility_timer==1){Bell=true;BellVib=true;Wrench=true;GPIO_SetBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);if(selenoid_buzzer_cnt<78)GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);}	
		}
	}
	else          						 //error yok normal alisma senaryosu
	{
		Tap=true;
		GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Led_GPIO_PIN);
		error_visibility_timer=0;
		if(!screen_rolling)
		{
			Bell=false;BellVib=false;Wrench=false;
		  Ripple=true;Ripple2=true;Drop=true;WritenDataVk1621(16,&DATA[15],1);WritenDataVk1621(17,&DATA[15],1);WritenDataVk1621(18,&DATA[15],1);
		}
			totalnumber(7,0);
		  totalnumber(8,3);
	}
	switch(Filter_Number)
	{
		case 0:totalnumber(1,0);totalnumber(2,10);break;     //toplam geen su miktari ekran.
		case 1:totalnumber(1,1);totalnumber(2,10);break;
		case 2:totalnumber(1,2);totalnumber(2,10);break;
		case 3:totalnumber(1,3);totalnumber(2,10);break;
		case 4:totalnumber(1,4);totalnumber(2,10);break;
		case 5:totalnumber(1,5);totalnumber(2,10);break;
		case 15:totalnumber(1,15);totalnumber(2,10);break;   // debi miktarini litre/dk cinsinden veren ekran.
	}
	binler_kalan=Filter_Life%1000;
	binler_bas=(Filter_Life-binler_kalan)/1000;
	
	yuzler_kalan=binler_kalan%100;
	yuzler_bas=(binler_kalan-yuzler_kalan)/100;
	
	onlar_kalan=yuzler_kalan%10;
	onlar_bas=(yuzler_kalan-onlar_kalan)/10;
	
	birler_bas=Filter_Life-1000*binler_bas-100*yuzler_bas-10*onlar_bas;
	
	totalnumber(3,binler_bas);
	totalnumber(4,yuzler_bas);
	totalnumber(5,onlar_bas);
	totalnumber(6,birler_bas);
}
void Flash_Write2(void)
{
    FMC_Unlock();
    FMC_ErasePage(0x7800);//FMC_ADDR
    FMC_ProgramWord(First_Filter_Add,First_FLife);  
	  FMC_ProgramWord(Second_Filter_Add,Second_FLife);  
	  FMC_ProgramWord(Third_Filter_Add,Third_FLife);  
	  FMC_ProgramWord(Fourth_Filter_Add,Fourth_FLife);  
	  FMC_ProgramWord(Fifth_Filter_Add,Fifth_FLife);  
	  FMC_ProgramWord(GALON_Add,GALON);  
		FMC_ProgramWord(E5_Add,E5_active);
	  FMC_ProgramWord(Cflow_Add,Const_Flow_Value);
    FMC_Lock();
}
void TMR4Init(void)
{
    /**  Divider = 7, counter = 0xff  */
    TMR4_ConfigTimerBase(7, 0XFF); 
    /**  Enable update interrupt  */
    TMR4_EnableInterrupt(TMR4_INT_UPDATE);
    /**  Enable TMR4  */
    TMR4_Enable();
    
    NVIC_EnableIRQRequest(TMR4_IRQn, 0X01);
}
void TMR4Isr(void)
 {// E5 ve Sistem on off kismi.
    if(TMR4_ReadIntFlag(TMR4_INT_UPDATE) == SET)
    {
        TMR4_ClearIntFlag(TMR4_INT_UPDATE);
        tick++;	
				if(SYSTEM_ON&&!Button3_state&&screen_rolling)  //
				{
					tick2++;        //screen_reset_cnt++;
					if(tick2>45)    //sistem aktif iken olmali
					{
						tick2=0;
						filling_symbol++;if(filling_symbol>12)filling_symbol=1;
					}
				}
				else tick2=0;
				if(tick==150)     //sn olarak 1-1 eslesmesinden dolayi 150 yaptim.
				{
					tick=0;
					if(SYSTEM_ON&&!Button3_state)
					{
						// YBS ve ABS switch 5 saniye kontrol mantığı - ayrı açılma/kapanma zamanlayıcıları
						bool current_high_pressure = GPIO_ReadInputBit(PRESSURE_GPIO_PORT, HIGH_PRESSURE_SENS_GPIO_PIN);
						bool current_low_pressure = GPIO_ReadInputBit(PRESSURE_GPIO_PORT, LOW_PRESSURE_SENS_GPIO_PIN);
						
						// YBS (Yüksek Basınç Switch) kontrolü
						if(current_high_pressure) // YBS tetiklendi
						{
							if(high_pressure_on_timer < 5) high_pressure_on_timer++;
							if(high_pressure_on_timer >= 5) high_pressure_stable_on = true;
							// YBS tetiklendiğinde off zamanlayıcısını sıfırla
							high_pressure_off_timer = 0;
							high_pressure_stable_off = false;
						}
						else // YBS tetiklenmedi
						{
							if(high_pressure_off_timer < 5) high_pressure_off_timer++;
							if(high_pressure_off_timer >= 5) high_pressure_stable_off = true;
							// YBS tetiklenmediğinde on zamanlayıcısını sıfırla
							high_pressure_on_timer = 0;
							high_pressure_stable_on = false;
						}
						
						// ABS (Alçak Basınç Switch) kontrolü
						if(current_low_pressure) // ABS tetiklendi
						{
							if(low_pressure_on_timer < 5) low_pressure_on_timer++;
							if(low_pressure_on_timer >= 5) low_pressure_stable_on = true;
							// ABS tetiklendiğinde off zamanlayıcısını sıfırla
							low_pressure_off_timer = 0;
							low_pressure_stable_off = false;
						}
						else // ABS tetiklenmedi
						{
							if(low_pressure_off_timer < 5) low_pressure_off_timer++;
							if(low_pressure_off_timer >= 5) low_pressure_stable_off = true;
							// ABS tetiklenmediğinde on zamanlayıcısını sıfırla
							low_pressure_on_timer = 0;
							low_pressure_stable_on = false;
						}
						
						high_pressure_last_state = current_high_pressure;
						low_pressure_last_state = current_low_pressure;
						
						if(onetime_eint){EINT_ClearIntFlag(EINT_PORT_C);NVIC_EnableIRQRequest(EINTC_IRQn, 0X02);onetime_eint=false;}
						if(Button1_state)Button1_Cnt++;
						// E5 ayar menüsü geçişi iptal edildi - Set+Menü butonları birlikte basıldığında E5'e geçmesin
						/*
						if(onetime_E5_active&&Error!=0&&Error!=2&&Button1_Cnt>2&&Button2_Cnt>2) //onetime_E5_active 17.1.24
						{
							 if(E5_active)E5_active=false;
							 else E5_active=true;Button1_Cnt=0;
							 onetime_E5_setting=true;selenoid_check_cnt=0;selenoid_check_time=0;
							 flowmeter_stability_pulse=0;  //??
							 selenoid_error_wait=false;
							 selenoid_error=false;
							 selenoid_buzzer_cnt=0; 
							 selenoid_buzzer=false; 
							 E5_lock=false; //17.1.24
							 E6_work_cnt=0;    				 
							 E6_lock=false;
							 E7_lock=false;
							 E6_work_cnt=0;
							onetime_E5_active=false;  //17.1.24 el çekmeyince ayar tekrar degisiyordu.
						}
						*/
						if(Button2_state)Button2_Cnt++;
						if(!onetime_E5_setting&&Button2_Cnt>3&&Button1_Cnt==0)  //Error!=0&& Error!=2&&6.12.23 Error!=5&&Error!=6&&Error!=7&& 8.12.23
						{
							GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);Delay(15000);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);  // 4.7.23
							Set_Mode=true;set_mode2_digit=false;
							screen_mode_cnt=0;  //5.6.23
						}
						if(Set_Mode||set_mode2_digit){numbers_visibilty_timer++;if(numbers_visibilty_timer==2)numbers_visibilty_timer=0;}
						if(Error!=4){if(onetime_buzzer)buzzer_error_timer++;error_visibility_timer++;if(error_visibility_timer==2)error_visibility_timer=0;if(buzzer_error_timer>=25){buzzer_error_timer=0;onetime_buzzer=false;}}  
						if(screen_rolling){water_filling_cnt++;} //Anlik debi için gerekli  //water_filling
						if(onetime_E5_setting){E5_visibility_timer++;if(E5_visibility_timer>3){onetime_E5_setting=false;E5_visibility_timer=0;}}
						if(E5_active&&!E5_lock&&selenoid_check_time==1&&!selenoid_error_wait)   
						{
							selenoid_check_cnt++;
							if(selenoid_check_cnt>64000&&selenoid_check_cnt<64010)    //110000 süresini 360-370
							{
								flowmeter_stability_pulse=0;
							}
							else if (selenoid_check_cnt>64010)   //370
							{
								if(flowmeter_stability_pulse>0)
								{
									if(!water_filling&&Error!=0&&!E2_lock&&!E7_lock&&!E6_lock){selenoid_error_wait=false;selenoid_buzzer_cnt=0;Error=5;selenoid_error=true;E5_lock=true;GPIO_ClearBit(OUTPUT_PORT,PUMP_GPIO_PIN);GPIO_ClearBit(OUTPUT_PORT,VALVE_GPIO_PIN);}
									selenoid_check_time=0;  //bu sartta check time 2 kaldirildigindan ve yb sartinda pulse varsa 0 a döndügünden direkt 0 yazildi.
								}
								else if(flowmeter_stability_pulse<1)
								{
									if(!water_filling){selenoid_check_time=0;}
									else selenoid_check_time=2;
								}
								selenoid_check_cnt=0;
							}
						}
						if(E5_active&&!E6_lock&&Error!=2&&Error!=0&&!E7_lock) // Error!=0&&  7.12.23 de eklendi.
						{
							//selenoid_check_time=0;flowmeter_stability_pulse=0;
							if(E6_pulse_reg<E6_Pulse)
								{
									  E6_work_cnt++;E6_pulse_reg=0;E6_Pulse=0;
										if(E6_work_cnt>110000) //110000
										{E5_lock=false;E6_lock=true;Error=6;selenoid_error_wait=false;selenoid_buzzer_cnt=0;GPIO_ClearBit(OUTPUT_PORT,PUMP_GPIO_PIN);GPIO_ClearBit(OUTPUT_PORT,VALVE_GPIO_PIN);}
								}  
							else
								{
									E6_work_cnt=0;E6_Pulse=0;E6_pulse_reg=0;E6_work_cnt=0;}   
						}
						if(selenoid_error_wait){selenoid_error_waiting_time++;if(selenoid_error_waiting_time>4390){selenoid_error_waiting_time=0;selenoid_error_wait=false;selenoid_buzzer_cnt=0;}} //selenoid_buzzer_cnt=0; 23.6.23 srekli buzzer yerine t sn.
						if(E5_active&&selenoid_buzzer&&(E5_lock||E6_lock||E7_lock))
						{
							selenoid_buzzer_cnt++;if(selenoid_buzzer_cnt>78){selenoid_buzzer_cnt=80;}  
						}
						if(!Set_Mode&&!set_mode2_digit&&screen_cnt_start)  
						{	 
							screen_mode_cnt++;
							if(screen_mode_cnt>6)
							{
								screen_mode_cnt=0;screen_cnt_start=false;
								if(screen_rolling)screen_mode=6;
								else {screen_mode=0;FLOW=0;}
							}
						}
						if(screen_roll_start){screen_rolling=true;screen_roll_start=false;screen_cnt_start=true; }
						else {screen_rolling=false;screen_cnt_start=true;flowmeter_stability_pulse=0;}
						
						if(mechanic_timecheck==1){mechanic_timecnt++;if(mechanic_timecnt>288){mechanic_timecnt=0;mechanic_timecheck=2;Mechanic_Valve_pulse=0;}}  
					  if(E2_Time_Start){E2_time_cnt++;if(E2_time_cnt>5){E2_Time_Start=false;E2_time_cnt=0;E2_Second_Control=true;}} //E2_time_cnt=0; eklendi
						if(E0_timer_start){E0_timer_cnt++;if(E0_timer_cnt>15)E0_timer_cnt=0;E0_timer_start=false;}
					}
					else if(Button3_state)
					{
						if(!Button2_state)
						{
							Button3_Cnt++;
							if(Button3_Cnt>2&&onetime_system_set) 
							{
								onetime_system_set=false;
								if(SYSTEM_ON==true){onetime_systemOn=true;NVIC_DisableIRQRequest(EINTC_IRQn);EINT_ClearIntFlag(EINT_PORT_C);SYSTEM_ON=false;ADC_ClearStatusFlag(ADC_FLAG_CC);}
								else {onetime_systemOff=true;SYSTEM_ON=true;}
								Button3_Cnt=0;
							}
						}
						else
						{
							// MANUAL ENTRY FEATURE DISABLED
							//button_2_3_cnt++;
							//if(button_2_3_cnt>3&&onetime_set_mode2_set)  //normalde 5 sn //!E5_lock&&!E7_lock&&!E6_lock&& 8.12.23 //Error!=0 && Error!=2&& 6.12.23
							//{
							//	GPIO_SetBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN);Delay(15000);GPIO_ClearBit(BuzzerLed_GPIO_PORT,Buzzer_GPIO_PIN); //delay 7000
							//  set_mode2_digit=true;button_2_3_cnt=0;Set_Mode=false;onetime_set_mode2_set=false;
							//	screen_mode_cnt=0;  //5.6.23
							//}  //filtre mr degisimi digit ile yapilacak.
						}
					}
				}
    }
}
void totalnumber(unsigned int order,unsigned int num)
{
	switch (order)
	{
		case 1:
			
			switch (num)
			{
				case 0:
						if(Tap)WritenDataVk1621(0,&DATA[11],1);else WritenDataVk1621(0,&DATA[10],1);																										// Delay(100);  //1101 = 11 bu sebeple 1-G yanmadi com3 kapali
						WritenDataVk1621(1,&DATA[15],1); 																													//Delay(100);  //1111 hepsi ayndi 	
				break;
				case 1:
						if(Tap)WritenDataVk1621(0,&DATA[1],1);else WritenDataVk1621(0,&DATA[0],1);   //ILK SAYI 1 DEGERI 
						WritenDataVk1621(1,&DATA[6],1);	
				break;
				case 2:
						if(Tap)WritenDataVk1621(0,&DATA[7],1);else WritenDataVk1621(0,&DATA[6],1);   //ILK SAYI 2 DEGERI 
						WritenDataVk1621(1,&DATA[13],1);	
				break;
				case 3:
			     if(Tap)WritenDataVk1621(0,&DATA[5],1);else WritenDataVk1621(0,&DATA[4],1);   //ILK SAYI 3 DEGERI 
			      WritenDataVk1621(1,&DATA[15],1); 
				break;
				case 4:
						if(Tap)WritenDataVk1621(0,&DATA[13],1);else WritenDataVk1621(0,&DATA[12],1);   //ILK SAYI 4 DEGERI 
			      WritenDataVk1621(1,&DATA[6],1);
				break;
				case 5:
					 if(Tap)WritenDataVk1621(0,&DATA[13],1);else WritenDataVk1621(0,&DATA[12],1);   //ILK SAYI 5 DEGERI 
  		      WritenDataVk1621(1,&DATA[11],1); 
				break;
				case 6:
					 if(Tap)WritenDataVk1621(0,&DATA[15],1);else WritenDataVk1621(0,&DATA[14],1);   //ILK SAYI 6 DEGERI 
						WritenDataVk1621(1,&DATA[11],1); 
					break;
				case 7:
						if(Tap)WritenDataVk1621(0,&DATA[1],1);else WritenDataVk1621(0,&DATA[0],1);    //ILK SAYI 7 DEGERI 
						WritenDataVk1621(1,&DATA[14],1); 
					break;
				case 8:
						if(Tap)WritenDataVk1621(0,&DATA[15],1);else WritenDataVk1621(0,&DATA[14],1);    //ILK SAYI 8 DEGERI 
						WritenDataVk1621(1,&DATA[15],1);
					break;
				case 10:  //silme islemi
						if(Tap)WritenDataVk1621(0,&DATA[1],1);else WritenDataVk1621(0,&DATA[0],1);   
						WritenDataVk1621(1,&DATA[0],1);
					break;
				case 15:  //F sayisi 
						if(Tap)WritenDataVk1621(0,&DATA[15],1);else WritenDataVk1621(0,&DATA[14],1);   
						WritenDataVk1621(1,&DATA[8],1);
					break;
				case 16:
						if(Tap)WritenDataVk1621(0,&DATA[15],1);else WritenDataVk1621(0,&DATA[14],1);   
						WritenDataVk1621(1,&DATA[9],1);
					break;
			}
			break;
		case 2:
			switch (num)
			{
				case 0:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[11],1);else WritenDataVk1621(2,&DATA[10],1);  
			     WritenDataVk1621(3,&DATA[15],1);
				}
				break;
				case 1:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[1],1);else WritenDataVk1621(2,&DATA[0],1);  
			     WritenDataVk1621(3,&DATA[6],1);
				}
				break;
				case 2:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[7],1);else WritenDataVk1621(2,&DATA[6],1);  
			     WritenDataVk1621(3,&DATA[13],1);
				}
				break;
				case 3:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[5],1);else WritenDataVk1621(2,&DATA[4],1);  
			     WritenDataVk1621(3,&DATA[15],1);
				}
				break;
				case 4:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[13],1);else WritenDataVk1621(2,&DATA[12],1);  
			     WritenDataVk1621(3,&DATA[6],1);
				}
				break;
				case 5:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[13],1);else WritenDataVk1621(2,&DATA[12],1);  
			     WritenDataVk1621(3,&DATA[11],1);
				}
				break;
				case 6:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[15],1);else WritenDataVk1621(2,&DATA[14],1);  
			     WritenDataVk1621(3,&DATA[11],1);
				}
				break;
				case 7:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[1],1);else WritenDataVk1621(2,&DATA[0],1); 
			     WritenDataVk1621(3,&DATA[14],1);
				}
				break;
				case 8:
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[15],1);else WritenDataVk1621(2,&DATA[14],1);  
			     WritenDataVk1621(3,&DATA[15],1);
				}
				case 10: // - ISARETI
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[5],1);else WritenDataVk1621(2,&DATA[4],1);  
			     WritenDataVk1621(3,&DATA[0],1);
				}
				break;
				case 11: // silme islemi
				{
					 if(BellVib)WritenDataVk1621(2,&DATA[1],1);else WritenDataVk1621(2,&DATA[0],1);  
			     WritenDataVk1621(3,&DATA[0],1);
				}
				break;
		}
		break;
		case 3:
			switch (num)
			{
				case 0:
					 if(Bell)WritenDataVk1621(4,&DATA[11],1);else WritenDataVk1621(4,&DATA[10],1);  
			     WritenDataVk1621(5,&DATA[15],1);
					break;
				case 1:
					 if(Bell)WritenDataVk1621(4,&DATA[1],1);else WritenDataVk1621(4,&DATA[0],1);  
			     WritenDataVk1621(5,&DATA[6],1);
					break;
				case 2:
					 if(Bell)WritenDataVk1621(4,&DATA[7],1);else WritenDataVk1621(4,&DATA[6],1);  
			     WritenDataVk1621(5,&DATA[13],1);
					break;
				case 3:
					 if(Bell)WritenDataVk1621(4,&DATA[5],1);else WritenDataVk1621(4,&DATA[4],1);  
			     WritenDataVk1621(5,&DATA[15],1);
					break;
				case 4:
					 if(Bell)WritenDataVk1621(4,&DATA[13],1);else WritenDataVk1621(4,&DATA[12],1);  
			     WritenDataVk1621(5,&DATA[6],1);
					break;
				case 5:
					 if(Bell)WritenDataVk1621(4,&DATA[13],1);else WritenDataVk1621(4,&DATA[12],1);  
			     WritenDataVk1621(5,&DATA[11],1);
					break;
				case 6:
					 if(Bell)WritenDataVk1621(4,&DATA[15],1);else WritenDataVk1621(4,&DATA[14],1);  
			     WritenDataVk1621(5,&DATA[11],1);
					break;
				case 7:
					if(Bell)WritenDataVk1621(4,&DATA[1],1);else WritenDataVk1621(4,&DATA[0],1);  
			     WritenDataVk1621(5,&DATA[14],1);
					break;
				case 8:
					 if(Bell)WritenDataVk1621(4,&DATA[15],1);else WritenDataVk1621(4,&DATA[14],1);  
			     WritenDataVk1621(5,&DATA[15],1);
					break;
				case 9:  
					 if(Bell)WritenDataVk1621(4,&DATA[13],1);else WritenDataVk1621(4,&DATA[12],1);  
			     WritenDataVk1621(5,&DATA[15],1);
					break;
				case 10:  //silme islemi
					 if(Bell)WritenDataVk1621(4,&DATA[1],1);else WritenDataVk1621(4,&DATA[0],1);  
			     WritenDataVk1621(5,&DATA[0],1);
					break;
				case 11:  // - isareti
					 if(Bell)WritenDataVk1621(4,&DATA[5],1);else WritenDataVk1621(4,&DATA[4],1);  
			     WritenDataVk1621(5,&DATA[0],1);
					break;
			}
			break;	
		case 4:
			switch (num)
			{
				case 0:
					 if(Drop)WritenDataVk1621(6,&DATA[11],1);else WritenDataVk1621(6,&DATA[10],1);
			     WritenDataVk1621(7,&DATA[15],1);
					break;
						
				case 1:
					 if(Drop)WritenDataVk1621(6,&DATA[1],1);else WritenDataVk1621(6,&DATA[0],1);  
			     WritenDataVk1621(7,&DATA[6],1);
					break;
				
				case 2:
					 if(Drop)WritenDataVk1621(6,&DATA[7],1);else WritenDataVk1621(6,&DATA[6],1);  
			     WritenDataVk1621(7,&DATA[13],1);
					break;
				
				case 3:
					 if(Drop)WritenDataVk1621(6,&DATA[5],1);else WritenDataVk1621(6,&DATA[4],1);  
			     WritenDataVk1621(7,&DATA[15],1);
					break;
				
				case 4:
					 if(Drop)WritenDataVk1621(6,&DATA[13],1);else WritenDataVk1621(6,&DATA[12],1);  
			     WritenDataVk1621(7,&DATA[6],1);
					break;
				
				case 5:
					 if(Drop)WritenDataVk1621(6,&DATA[13],1);else WritenDataVk1621(6,&DATA[12],1);  
			     WritenDataVk1621(7,&DATA[11],1);
					break;
				
				case 6:
					 if(Drop)WritenDataVk1621(6,&DATA[15],1);else WritenDataVk1621(6,&DATA[14],1);  
			     WritenDataVk1621(7,&DATA[11],1);
					break;
				
				case 7:
					 if(Drop)WritenDataVk1621(6,&DATA[1],1);else WritenDataVk1621(6,&DATA[0],1);  
			     WritenDataVk1621(7,&DATA[14],1);
					break;
				
				case 8:
					 if(Drop)WritenDataVk1621(6,&DATA[15],1);else WritenDataVk1621(6,&DATA[14],1);  
			     WritenDataVk1621(7,&DATA[15],1);
					break;
				case 9:  
					 if(Drop)WritenDataVk1621(6,&DATA[13],1);else WritenDataVk1621(6,&DATA[12],1);  
			     WritenDataVk1621(7,&DATA[15],1);
					break;
				case 10:  //silme islemi
					 if(Drop)WritenDataVk1621(6,&DATA[1],1);else WritenDataVk1621(6,&DATA[0],1);  
			     WritenDataVk1621(7,&DATA[0],1);
					break;
			}
			break;
		case 5:   //5th Seg has no symbol
			switch (num)
			{
				case 0:
					if(Dot)WritenDataVk1621(8,&DATA[11],1);else WritenDataVk1621(8,&DATA[10],1);
			     WritenDataVk1621(9,&DATA[15],1);
					break;
						
				case 1:
					 if(Dot)WritenDataVk1621(8,&DATA[1],1); else WritenDataVk1621(8,&DATA[0],1);  
			     WritenDataVk1621(9,&DATA[6],1);
					break;
				
				case 2:
					 if(Dot)WritenDataVk1621(8,&DATA[7],1); else WritenDataVk1621(8,&DATA[6],1);  
			     WritenDataVk1621(9,&DATA[13],1);
					break;
				
				case 3:
					 if(Dot)WritenDataVk1621(8,&DATA[5],1); else WritenDataVk1621(8,&DATA[4],1);  
			     WritenDataVk1621(9,&DATA[15],1);
					break;
				
				case 4:
					 if(Dot)WritenDataVk1621(8,&DATA[13],1); else WritenDataVk1621(8,&DATA[12],1);   
			     WritenDataVk1621(9,&DATA[6],1);
					break;
				
				case 5:
					  if(Dot)WritenDataVk1621(8,&DATA[13],1); else WritenDataVk1621(8,&DATA[12],1);  
			     WritenDataVk1621(9,&DATA[11],1);
					break;
				
				case 6:
					  if(Dot)WritenDataVk1621(8,&DATA[15],1); else WritenDataVk1621(8,&DATA[14],1);   
			     WritenDataVk1621(9,&DATA[11],1);
					break;
				
				case 7:
					 if(Dot)WritenDataVk1621(8,&DATA[1],1); else  WritenDataVk1621(8,&DATA[0],1); 
			     WritenDataVk1621(9,&DATA[14],1);
					break;
		
				case 8:
					 if(Dot)WritenDataVk1621(8,&DATA[15],1); else WritenDataVk1621(8,&DATA[14],1);   
			     WritenDataVk1621(9,&DATA[15],1);
					break;
				case 9:
					  if(Dot)WritenDataVk1621(8,&DATA[13],1); else WritenDataVk1621(8,&DATA[12],1);  
			     WritenDataVk1621(9,&DATA[15],1);
					break;
				case 10:  //SILME ISLEMI
					 WritenDataVk1621(8,&DATA[0],1);  
			     WritenDataVk1621(9,&DATA[0],1);
					break;
				case 14:  //N isareti  E5 AYARI IIN
					 if(Dot)WritenDataVk1621(8,&DATA[11],1); else  WritenDataVk1621(8,&DATA[10],1);
			     WritenDataVk1621(9,&DATA[14],1);
					break;
				case 15:  //F isareti
					  if(Dot)WritenDataVk1621(8,&DATA[15],1);else  WritenDataVk1621(8,&DATA[14],1); 
			     WritenDataVk1621(9,&DATA[8],1);	 
					break;
			}
			break;
		  case 6:
				switch(num)
				{
				case 0:
					 if(G)WritenDataVk1621(10,&DATA[11],1);else WritenDataVk1621(10,&DATA[10],1);  
			     WritenDataVk1621(11,&DATA[15],1);
					break;
						
				case 1:
					 if(G)WritenDataVk1621(10,&DATA[1],1);else WritenDataVk1621(10,&DATA[0],1);  
			     WritenDataVk1621(11,&DATA[6],1);
					break;
				
				case 2:
					 if(G)WritenDataVk1621(10,&DATA[7],1);else WritenDataVk1621(10,&DATA[6],1);  
			     WritenDataVk1621(11,&DATA[13],1);
					break;
				
				case 3:
					 if(G)WritenDataVk1621(10,&DATA[5],1);else WritenDataVk1621(10,&DATA[4],1);  
			     WritenDataVk1621(11,&DATA[15],1);
					break;
				
				case 4:
					 if(G)WritenDataVk1621(10,&DATA[13],1);else WritenDataVk1621(10,&DATA[12],1);  
			     WritenDataVk1621(11,&DATA[6],1);
					break;
				
				case 5:
					 if(G)WritenDataVk1621(10,&DATA[13],1);else WritenDataVk1621(10,&DATA[12],1);  
			     WritenDataVk1621(11,&DATA[11],1);
					break;
				
				case 6:
					 if(G)WritenDataVk1621(10,&DATA[15],1);else WritenDataVk1621(10,&DATA[14],1);  
			     WritenDataVk1621(11,&DATA[11],1);
					break;
				
				case 7:
					 if(G)WritenDataVk1621(10,&DATA[1],1);else WritenDataVk1621(10,&DATA[0],1);  
			     WritenDataVk1621(11,&DATA[14],1);
					break;
				
				case 8:
					 if(G)WritenDataVk1621(10,&DATA[15],1);else WritenDataVk1621(10,&DATA[14],1);  
			     WritenDataVk1621(11,&DATA[15],1);
					break;
				case 9:
					 if(G)WritenDataVk1621(10,&DATA[13],1);else WritenDataVk1621(10,&DATA[12],1);  
			     WritenDataVk1621(11,&DATA[15],1);
					break;
				case 10:  //silme islemi
					 if(G)WritenDataVk1621(10,&DATA[1],1);else WritenDataVk1621(10,&DATA[0],1);  
			     WritenDataVk1621(11,&DATA[0],1);
					break;
				case 15: //F isareti
					 if(G)WritenDataVk1621(10,&DATA[15],1);else WritenDataVk1621(10,&DATA[14],1);  
			     WritenDataVk1621(11,&DATA[8],1);
					break;
					}
			break;
			case 7:   //silme islemi olmali
				switch(num)
				{
					case 0:
					 if(Ripple)WritenDataVk1621(12,&DATA[1],1); else WritenDataVk1621(12,&DATA[0],1);  
			     WritenDataVk1621(13,&DATA[0],1);
					break;
					case 14:
					 if(Ripple)WritenDataVk1621(12,&DATA[15],1); else WritenDataVk1621(12,&DATA[14],1);  
			     WritenDataVk1621(13,&DATA[9],1);
					break;
				}
			break;
			case 8:
				switch(num)
				{
					case 0:
					 if(Ripple2)WritenDataVk1621(14,&DATA[11],1);else WritenDataVk1621(14,&DATA[10],1);  
			     WritenDataVk1621(15,&DATA[15],1);
					  break;
					case 1:
					 if(Ripple2)WritenDataVk1621(14,&DATA[1],1);else WritenDataVk1621(14,&DATA[0],1);  
			     WritenDataVk1621(15,&DATA[6],1);
					  break;
					case 2:
					 if(Ripple2)WritenDataVk1621(14,&DATA[7],1);else WritenDataVk1621(14,&DATA[6],1);  
			     WritenDataVk1621(15,&DATA[13],1);
					  break;
					case 3: //SILME ISLEMI
					 if(Ripple2)WritenDataVk1621(14,&DATA[1],1);else WritenDataVk1621(14,&DATA[0],1);  
			     WritenDataVk1621(15,&DATA[0],1);
					  break;
					case 5:
					 if(Ripple2)WritenDataVk1621(14,&DATA[13],1);else WritenDataVk1621(14,&DATA[12],1);  
			     WritenDataVk1621(15,&DATA[11],1);
					  break;
					case 6:
					 if(Ripple2)WritenDataVk1621(14,&DATA[14],1);else WritenDataVk1621(14,&DATA[15],1);  
			     WritenDataVk1621(15,&DATA[11],1);
					  break;
					case 7:
					 if(Ripple2)WritenDataVk1621(14,&DATA[1],1);else WritenDataVk1621(14,&DATA[0],1);  
			     WritenDataVk1621(15,&DATA[14],1);
					  break;
				}
			break;
	} 
	if(!LPM&&Wrench)WritenDataVk1621(19,&DATA[1],1);   			 //31.05.23
	else if(LPM&&!Wrench)WritenDataVk1621(19,&DATA[2],1);
	else if(LPM&&Wrench)WritenDataVk1621(19,&DATA[3],1);
	else if(!LPM&&!Wrench)WritenDataVk1621(19,&DATA[0],1);
}
void delay_nus(unsigned int n)	   
{
	unsigned char i;
	while(n--)
	{
		i=10;
		while(i--)  
		{
			__nop();
		}
	}
}
void delay_nms(unsigned long int n)
{
	while(n--)
	{
		delay_nus(1000);
	}
}
void WriteClockVk1621(void)
{
	VK1621_WR_L(); 
	delay_nus(VK1621_CLK);
	VK1621_WR_H();	
	delay_nus(VK1621_CLK);	
}
void WriteCommandVk1621(unsigned char FunctonCode)
{
	unsigned char Shift = 0x80; 
	unsigned char i;
	
	VK1621_CS_L();     
	delay_nus(VK1621_CLK/2);
	VK1621_DATA_H();  
	WriteClockVk1621();
	VK1621_DATA_L();  
	WriteClockVk1621();
	VK1621_DATA_L();  
	WriteClockVk1621();

	for(i = 0; i < 8; i++) 
	{
	 if(Shift & FunctonCode) 	
		 VK1621_DATA_H(); 
	 else VK1621_DATA_L();	  

	 WriteClockVk1621();
	 Shift = Shift >> 1;
	}
  VK1621_DATA_L(); 
	WriteClockVk1621();	
  VK1621_CS_H(); 		
	delay_nus(VK1621_CLK/2);
  VK1621_DATA_H(); 
}
void WritenDataVk1621(unsigned char Addr,unsigned char *Databuf,unsigned char Cnt)
{
	unsigned char i,j; 
	unsigned char Shift;
	unsigned char dataval; 
	
	VK1621_CS_L();   
	delay_nus(VK1621_CLK/2);

	VK1621_DATA_H();  
	WriteClockVk1621();  //101
	VK1621_DATA_L();  
	WriteClockVk1621();
	VK1621_DATA_H();  
	WriteClockVk1621();

	Shift = 0x20;
	for( i = 0; i < vk1621_addrbit; i++) 
	{   		 
		if (Addr & Shift) 
			VK1621_DATA_H(); 		
		else  
			VK1621_DATA_L();
		WriteClockVk1621();		
		Shift = Shift >> 1; 
	}
	for (j = 0; j < Cnt; j++) 
	{
		Shift = 0x01;
		dataval=*Databuf++;
		for (i = 0; i < 4; i++) 
		{
			if( dataval & Shift) 
				VK1621_DATA_H();		 
			else  
				VK1621_DATA_L();
			WriteClockVk1621();
			Shift = Shift << 1;
		}   
	}
  VK1621_CS_H();   
	delay_nus(VK1621_CLK/2);	 
  VK1621_DATA_H(); 
}
void VK1621_DATA_H(){GPIO_SetBit(DATA_GPIO_PORT,DATA_GPIO_PIN);	}
void VK1621_DATA_L(){GPIO_ClearBit(DATA_GPIO_PORT,DATA_GPIO_PIN);}
void Vk1621_DisAll(unsigned char dat)
{
	unsigned char segi;
	unsigned char dispram[32];
	
	for(segi=0;segi<32;segi++) 
	{
		dispram[segi]=dat;
	}
	WritenDataVk1621(1,dispram,32);
}
void Vk1621_Lowlevel_Init(void)
{
	GPIO_SetBit(CS_GPIO_PORT,CS_GPIO_PIN);
	GPIO_SetBit(RD_GPIO_PORT,RD_GPIO_PIN);
	GPIO_SetBit(WR_GPIO_PORT,WR_GPIO_PIN);
	GPIO_SetBit(DATA_GPIO_PORT,DATA_GPIO_PIN);
//	GPIO_SetMode(VK1621_DAT_PORT, VK1621_DAT_PIN, GPIO_MODE_QUASI); 
		
	VK1621_CS_H();      
	VK1621_RD_H();                    
	VK1621_WR_H();  
	VK1621_DATA_H(); 	
}
void Vk1621_Init(void)
{	
	Vk1621_Lowlevel_Init();

	WriteCommandVk1621(OSC_ON);
	WriteCommandVk1621(DISP_ON);
	WriteCommandVk1621(COM_1_3__4);vk1621_maxcom=4;	//1/3bias 4com
	
	WriteCommandVk1621(BUZZ_OFF);
	WriteCommandVk1621(IRQ_DIS);
	WriteCommandVk1621(TIMER_DIS);
	WriteCommandVk1621(WDT_DIS);
}
void Flow_Pulse(void)   //E5 FLOW SAYMAMA DURUMU BURASI SAYMADIGINDA ISLEMEYECEGI ICIN BURADA KONTROL EDILMEZ.
 {
	if(SYSTEM_ON)//&&!Button3_state
	{
		flowmeter_stability_pulse++;
		pulse++;
		if(!screen_roll_start){screen_roll_start=true;}
		water_filling_pulse++;
		if((pulse>Const_Flow_Value)&&screen_rolling)  			 //&&water_filling F DEGERINDEKI 4 BAS SAYI 2548
		{
			liter++;
			GALON_REG=liter/3.78;  	  				 //0-5 ARASI FILTRE AZALIMI
			if(GALON_REG>0)
			{
				GALON++;
				liter=0;
				if(First_FLife!=0)First_FLife--;
				if(Second_FLife!=0)Second_FLife--;
				if(Third_FLife!=0)Third_FLife--;
				if(Fourth_FLife!=0)Fourth_FLife--;
				if(Fifth_FLife!=0)Fifth_FLife--;
									
									binler_kalan=GALON%1000;
									fourdigit=(GALON-binler_kalan)/1000;
									yuzler_kalan=binler_kalan%100;
									threedigit=(binler_kalan-yuzler_kalan)/100;
									onlar_kalan=yuzler_kalan%10;
									twodigit=(yuzler_kalan-onlar_kalan)/10;
									onedigit=GALON-1000*fourdigit-100*threedigit-10*twodigit;
				
									binler_kalan=First_FLife%1000;
									fourdigit1=(First_FLife-binler_kalan)/1000;
									yuzler_kalan=binler_kalan%100;
									threedigit1=(binler_kalan-yuzler_kalan)/100;
									onlar_kalan=yuzler_kalan%10;
									twodigit1=(yuzler_kalan-onlar_kalan)/10;			
									onedigit1=First_FLife-1000*fourdigit1-100*threedigit1-10*twodigit1;
				
									binler_kalan=Second_FLife%1000;
									fourdigit2=(Second_FLife-binler_kalan)/1000;
									yuzler_kalan=binler_kalan%100;
									threedigit2=(binler_kalan-yuzler_kalan)/100;
									onlar_kalan=yuzler_kalan%10;
									twodigit2=(yuzler_kalan-onlar_kalan)/10;
									onedigit2=Second_FLife-1000*fourdigit2-100*threedigit2-10*twodigit2;
				
									binler_kalan=Third_FLife%1000;
									fourdigit3=(Third_FLife-binler_kalan)/1000;		
									yuzler_kalan=binler_kalan%100;
									threedigit3=(binler_kalan-yuzler_kalan)/100;		
									onlar_kalan=yuzler_kalan%10;
									twodigit3=(yuzler_kalan-onlar_kalan)/10;
									onedigit3=Third_FLife-1000*fourdigit3-100*threedigit3-10*twodigit3; 
									
									binler_kalan=Fourth_FLife%1000;
									fourdigit4=(Fourth_FLife-binler_kalan)/1000;
									yuzler_kalan=binler_kalan%100;
									threedigit4=(binler_kalan-yuzler_kalan)/100;
									onlar_kalan=yuzler_kalan%10;
									twodigit4=(yuzler_kalan-onlar_kalan)/10;
									onedigit4=Fourth_FLife-1000*fourdigit4-100*threedigit4-10*twodigit4; 
									
									binler_kalan=Fifth_FLife%1000;
									fourdigit5=(Fifth_FLife-binler_kalan)/1000;
									yuzler_kalan=binler_kalan%100;
									threedigit5=(binler_kalan-yuzler_kalan)/100;
									onlar_kalan=yuzler_kalan%10;
									twodigit5=(yuzler_kalan-onlar_kalan)/10;
									onedigit5=Fifth_FLife-1000*fourdigit5-100*threedigit5-10*twodigit5;
								
				Flash_Write2();
				GALON_REG=0;
			}
			Total_Water=liter;  	    //DEBI ICIN VERI
			pulse=0;
		}
		if(E5_active&&!E6_lock&&!E7_lock)   //&&!water_filling&&!selenoid_error_wait
		{
			if(!E5_lock&&!water_filling&&!selenoid_error_wait)
			{if(selenoid_check_time==0)selenoid_check_time=1;}
      E6_Pulse++; 			
		}
		if(water_filling_cnt>2)    												//4
		{
			FLOW=2000*water_filling_pulse/Const_Flow_Value; //1200*(3sn guncellendi)   2548;           //60 sn / 5 sn de = 12 oldugundan 12 ile carptim.2548 pulse=litre. 100 ise 00.54 0054 gzkmesi iin.
			water_filling_pulse=0;water_filling_cnt=0; 
		} 
		if(E6_lock&&Error!=2&&Error!=0){Mechanic_Valve_pulse++;if(mechanic_timecheck==0&&Mechanic_Valve_pulse>10)mechanic_timecheck=1;  //&&Error!=0  7.12.23 de eklendi.
																			 else if(mechanic_timecheck==2&&Mechanic_Valve_pulse>10){Error=7;E6_lock=false;E7_lock=true;selenoid_error_wait=false;selenoid_buzzer_cnt=0;}}
		 EINT_ClearIntFlag(EINT_PORT_C);
	} 
}
void Board_KeyInit(void)
{
    GPIO_Config_T gpioConfig;
//    EINT_PORT_T eintPortTab[] = {EINT_PORT_C};  //8.6.23 warning iin
    
    gpioConfig.mode = GPIO_MODE_IN_PU;
    gpioConfig.pin = GPIO_PIN_7;
    gpioConfig.intEn = GPIO_EINT_ENABLE;
    EINT_Config(EINT_PORT_C, EINT_TRIGGER_FALLING);

    GPIO_Config(GPIOC,&gpioConfig);
}
void ADCMeasure(void)
{
    //uint8_t i;

////    for (i = 0; i < 10; i++)
////    {
        //while (ADC_ReadStatusFlag(ADC_FLAG_CC) == RESET);
				if(ADC_ReadStatusFlag(ADC_FLAG_CC) == RESET)

        ADC_ClearStatusFlag(ADC_FLAG_CC);

        adcData = ADC_ReadData();

        /**  voltage(mV) =  adcData * (3300mV / 4095) */
        voltage = (adcData * 3300) / 4095;

        /**  voltage greater than 3100mv */
        if (voltage > 300)  //70 ler 125 idi (5000 ADET TESLIM DEGERI 150 IDI)
        {
					//Delay(5000);
					//adcData = ADC_ReadData();
					//voltage = (adcData * 3300) / 4095;

					//if (voltage > 150) 
					//{
						if(!E2_First_Control){E2_First_Control=true;E2_Time_Start=true;}
						else if(E2_Second_Control){Water_Leak=1;E2_lock=true;} 
					//}
					//else if(voltage<=150&&!E2_lock) {Water_Leak=0; if(Error==2){Error=4;buzzer_error_timer=0;onetime_buzzer=true;onetime_waterLeak=true;}}  
        }
				else if(voltage<=300&&!E2_lock){Water_Leak=0; if(Error==2){Error=4;buzzer_error_timer=0;onetime_buzzer=true;onetime_waterLeak=true;}E2_time_cnt=0;}  //E2_time_cnt=0; eklendi
////    }
}
void ADCInit(ADC_CHANNEL_T channel)
{
    ADC_Config_T adcConfig;

    /** ADC GPIO configuration */
    ADC_GPIO_Init(channel);

    ADC_SetMode(ADC_MODE_SINGLE_END);

    /** ADC configuration */
    ADC_ConfigStructInit(&adcConfig);
    adcConfig.channel = channel;
    adcConfig.convMode = ADC_CONV_MODE_CONTINUOUS;
    adcConfig.interrupt = ADC_INT_NONE;
    ADC_Config(&adcConfig);

    /** ADC Calibration */
    ADCCalibration();

    ADC_Enable();

    ADC_StartConversion();
}
void ADC_GPIO_Init(ADC_CHANNEL_T channel)
{
    GPIO_Config_T gpioConfig;

    /** ADC GPIO configuration */
    gpioConfig.intEn = GPIO_EINT_DISABLE;
    gpioConfig.mode = GPIO_MODE_IN_FLOATING;

    if (channel == ADC_CHANNEL_4)
    {
        gpioConfig.pin = GPIO_PIN_3;
        GPIO_Config(GPIOD, &gpioConfig);
    }
    else if (channel == ADC_CHANNEL_5)
    {
        gpioConfig.pin = GPIO_PIN_5;
        GPIO_Config(GPIOD, &gpioConfig);
    }
}
 void ADCCalibration(void)
{
	
    uint8_t i;
    int8_t offset = 0;
    uint16_t adcData = 0;

    ADC_SetOffset(0);
    ADC_EnableCompensation();
    ADC_ClearStatusFlag(ADC_FLAG_CC);
    ADC_Enable();

    for (i = 0; i < 10; i++)
    {
        ADC_StartConversion();

        while (ADC_ReadStatusFlag(ADC_FLAG_CC) == RESET);

        ADC_ClearStatusFlag(ADC_FLAG_CC);
    }

    ADC_Disable();

    adcData = ADC_ReadData();
    offset = (int8_t)(0x800 - adcData);
    ADC_SetOffset(offset);

    ADC_DisableCompensation();
    ADC_ClearStatusFlag(ADC_FLAG_CC);
}
