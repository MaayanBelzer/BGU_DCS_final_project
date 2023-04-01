#include <stdint.h>

//UART----------
#define SDA_SERIAL_BAUD 9600
#define CORE_CLOCK 48000000
//--------------
#define SW_POS 0x80  //PTD7

#define MUDULO_REGISTER  0x2EE0

#ifndef TFC_bspGPIO_H_
#define TFC_bspGPIO_H_


#define TFC_HBRIDGE_EN_LOC			(uint32_t)(1<<21)
#define TFC_HBRIDGE_FAULT_LOC     	(uint32_t)(1<<20)

#define TFC_HBRIDGE_ENABLE			GPIOE_PSOR = TFC_HBRIDGE_EN_LOC	
#define TFC_HBRIDGE_DISABLE			GPIOE_PCOR = TFC_HBRIDGE_EN_LOC	
//-------------- Switches = SWs ------------------------------------------- 
#define TFC_DIP_SWITCH0_LOC			((uint32_t)(1<<2))
#define TFC_DIP_SWITCH1_LOC			((uint32_t)(1<<3))
#define TFC_DIP_SWITCH2_LOC			((uint32_t)(1<<4))
#define TFC_DIP_SWITCH3_LOC			((uint32_t)(1<<5))
//-------------- Push Buttons = PBs ------------------------------------------- 
#define PUSH_BUTT0N0_LOC		((uint32_t)(1<<7))
#define PUSH_BUTT0N1_LOC		((uint32_t)(1<<6))	

#define PBs_PORT_INPUT          GPIOD_PDIR
#define PBs_INT_PENDING        	PORTD_ISFR

#define PUSH_BUTTON0_RELEASED	((PBs_PORT_INPUT & PUSH_BUTT0N0_LOC)>0)
#define PUSH_BUTTON1_RELEASED	((PBs_PORT_INPUT & PUSH_BUTT0N1_LOC)>0)
#define PUSH_BUTTON0_PRESSED	(!(PBs_PORT_INPUT & PUSH_BUTT0N0_LOC)>0)
#define PUSH_BUTTON1_PRESSED	(!(PBs_PORT_INPUT & PUSH_BUTT0N1_LOC)>0)
//-------------- Battery_LEDs ------------------------------------------- 
#define TFC_BAT_LED0_LOC			((uint32_t)(1<<8))
#define TFC_BAT_LED1_LOC			((uint32_t)(1<<9))
#define TFC_BAT_LED2_LOC			((uint32_t)(1<<10))
#define TFC_BAT_LED3_LOC			((uint32_t)(1<<11))
//-------------  RGB LEDs ---------------------------------------------
#define RED_LED_LOC	         		((uint32_t)(1<<18))
#define GREEN_LED_LOC	         	((uint32_t)(1<<19))
#define BLUE_LED_LOC	         	((uint32_t)(1<<1))
#define PORT_LOC(x)        	        ((uint32_t)(1<<x))

#define RED_LED_OFF          		GPIOB_PSOR |= RED_LED_LOC
#define GREEN_LED_OFF	         	GPIOB_PSOR |= GREEN_LED_LOC
#define BLUE_LED_OFF	            GPIOD_PSOR |= BLUE_LED_LOC
#define RGB_LED_OFF	                RED_LED_OFF,GREEN_LED_OFF,BLUE_LED_OFF

#define RED_LED_TOGGLE          	GPIOB_PTOR |= RED_LED_LOC
#define GREEN_LED_TOGGLE	        GPIOB_PTOR |= GREEN_LED_LOC
#define BLUE_LED_TOGGLE	            GPIOD_PTOR |= BLUE_LED_LOC
#define RGB_LED_TOGGLE	            RED_LED_TOGGLE,GREEN_LED_TOGGLE,BLUE_LED_TOGGLE

#define RED_LED_ON          		GPIOB_PCOR |= RED_LED_LOC
#define GREEN_LED_ON	         	GPIOB_PCOR |= GREEN_LED_LOC
#define BLUE_LED_ON	                GPIOD_PCOR |= BLUE_LED_LOC
#define RGB_LED_ON	                RED_LED_ON,GREEN_LED_ON,BLUE_LED_ON

//-----------------------------------------------------------------------------
#define TFC_BAT_LED0_ON				GPIOB_PSOR = TFC_BAT_LED0_LOC
#define TFC_BAT_LED1_ON				GPIOB_PSOR = TFC_BAT_LED1_LOC
#define TFC_BAT_LED2_ON				GPIOB_PSOR = TFC_BAT_LED2_LOC
#define TFC_BAT_LED3_ON				GPIOB_PSOR = TFC_BAT_LED3_LOC

#define TFC_BAT_LED0_OFF			GPIOB_PCOR = TFC_BAT_LED0_LOC
#define TFC_BAT_LED1_OFF			GPIOB_PCOR = TFC_BAT_LED1_LOC
#define TFC_BAT_LED2_OFF			GPIOB_PCOR = TFC_BAT_LED2_LOC
#define TFC_BAT_LED3_OFF			GPIOB_PCOR = TFC_BAT_LED3_LOC

#define TFC_BAT_LED0_TOGGLE			GPIOB_PTOR = TFC_BAT_LED0_LOC
#define TFC_BAT_LED1_TOGGLE			GPIOB_PTOR = TFC_BAT_LED1_LOC
#define TFC_BAT_LED2_TOGGLE			GPIOB_PTOR = TFC_BAT_LED2_LOC
#define TFC_BAT_LED3_TOGGLE			GPIOB_PTOR = TFC_BAT_LED3_LOC

//-----------------------------------------------------------------------------
//LCD
/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)    (!a ? (GPIOE_PDOR&=~0X20) : (GPIOE_PDOR|=0X20)) 
#define LCD_EN_DIR(a)    (!a ? (GPIOE_PDDR&=~0X20) : (GPIOE_PDDR|=0X20))
#define LCD_RS(a)        (!a ? (GPIOE_PDOR&=~0X08) : (GPIOE_PDOR|=0X08)) 
#define LCD_RS_DIR(a)    (!a ? (GPIOE_PDDR &=~0X08) : (GPIOE_PDDR|=0X08))
#define LCD_RW(a)        (!a ? (GPIOE_PDOR&=~0X10) : (GPIOE_PDOR|=0X10)) 
#define LCD_RW_DIR(a)    (!a ? (GPIOE_PDDR &=~0X10) : (GPIOE_PDDR|=0X10)) 
#define LCD_DATA_OFFSET 0x00
#define LCD_DATA_WRITE    GPIOB_PDOR
#define LCD_DATA_DIR    GPIOB_PDDR
/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#ifdef CHECKBUSY
	#define	LCD_WAIT lcd_check_busy()
#else
	#define LCD_WAIT DelayMs(5)
#endif


#define FOURBIT_MODE	0x0
#define EIGHTBIT_MODE	0x1
#define LCD_MODE        FOURBIT_MODE
   
#define OUTPUT_PIN      1	
#define INPUT_PIN       0	
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00	

#define LCD_STROBE_READ(value)	LCD_EN(1), \
				asm("nop"), asm("nop"), \
				value=LCD_DATA_READ, \
				LCD_EN(0) 

#define	lcd_cursor(x)		lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()		lcd_cmd(0x01)
#define lcd_putchar(x)		lcd_data(x)
#define lcd_goto(x)		lcd_cmd(0x80+(x))
#define lcd_cursor_right()	lcd_cmd(0x14)
#define lcd_cursor_left()	lcd_cmd(0x10)
#define lcd_display_shift()	lcd_cmd(0x1C)
#define lcd_home()		lcd_cmd(0x02)
#define cursor_off              lcd_cmd(0x0C)
#define cursor_on               lcd_cmd(0x0F) 
#define lcd_function_set        lcd_cmd(0x3C) // 8bit,two lines,5x10 dots 
#define lcd_new_line            lcd_cmd(0xC0)     


//-----------------------------------------------------------------------------
//ENGINE
//----------------------------------------------------------

#define ENGINE_PH_A(a)    (!a ? (GPIOC_PDOR&=~PORT_LOC(12)) : (GPIOC_PDOR|=PORT_LOC(12))) 
#define ENGINE_PH_A_DIR(a)    (!a ? (GPIOC_PDDR&=~PORT_LOC(12)) : (GPIOC_PDDR|=PORT_LOC(12)))
#define ENGINE_PH_B(a)    (!a ? (GPIOC_PDOR&=~PORT_LOC(13)) : (GPIOC_PDOR|=PORT_LOC(13))) 
#define ENGINE_PH_B_DIR(a)    (!a ? (GPIOC_PDDR&=~PORT_LOC(13)) : (GPIOC_PDDR|=PORT_LOC(13)))
#define ENGINE_PH_C(a)    (!a ? (GPIOC_PDOR&=~PORT_LOC(16)) : (GPIOC_PDOR|=PORT_LOC(16))) 
#define ENGINE_PH_C_DIR(a)    (!a ? (GPIOC_PDDR&=~PORT_LOC(16)) : (GPIOC_PDDR|=PORT_LOC(16)))
#define ENGINE_PH_D(a)    (!a ? (GPIOC_PDOR&=~PORT_LOC(17)) : (GPIOC_PDOR|=PORT_LOC(17))) 
#define ENGINE_PH_D_DIR(a)    (!a ? (GPIOC_PDDR&=~PORT_LOC(17)) : (GPIOC_PDDR|=PORT_LOC(17)))

//-----------------------------------------------------------------------------
//ADC
//----------------------------------------------------------
#define ADC_MAX_CODE    (4095)

#define ADC0_irq_no   57
#define ADC1_irq_no   58

#define ADC0_CHANA    19                                    // set to desired ADC0 channel trigger A    
#define ADC0_CHANB    20                                    // set to desired ADC0 channel trigger B    

#define ADC1_CHANA    20                                    // set to desired ADC1 channel trigger A  20 defaults to potentiometer in TWRK60     
#define ADC1_CHANB    20                                    // set to desired ADC1 channel trigger B

#define ADC0_DLYA     0x2000                                // ADC0 trigger A delay 
#define ADC0_DLYB     0x4000                                // ADC0 trigger B delay 
#define ADC1_DLYA     0x6000                                // ADC1 trigger A delay
#define ADC1_DLYB     0x7fff                                // ADC1 trigger B delay 


#define ADC0A_DONE   0x01       
#define ADC0B_DONE   0x02       
#define ADC1A_DONE   0x04       
#define ADC1B_DONE   0x08       


// Bit shifting of bitfiled is already taken into account so 
// bitfiled values are always represented as relative to their position.
  
/************************* #Defines ******************************************/


#define A                 0x0
#define B                 0x1


/////// NOTE: the following defines relate to the ADC register definitions
/////// and the content follows the reference manual, using the same symbols.


//// ADCSC1 (register)

// Conversion Complete (COCO) mask
#define COCO_COMPLETE     ADC_SC1_COCO_MASK
#define COCO_NOT          0x00

// ADC interrupts: enabled, or disabled.
#define AIEN_ON           ADC_SC1_AIEN_MASK
#define AIEN_OFF          0x00

// Differential or Single ended ADC input
#define DIFF_SINGLE       0x00
#define DIFF_DIFFERENTIAL ADC_SC1_DIFF_MASK

//// ADCCFG1

// Power setting of ADC
#define ADLPC_LOW         ADC_CFG1_ADLPC_MASK
#define ADLPC_NORMAL      0x00

// Clock divisor
#define ADIV_1            0x00
#define ADIV_2            0x01
#define ADIV_4            0x02
#define ADIV_8            0x03

// Long samle time, or Short sample time
#define ADLSMP_LONG       ADC_CFG1_ADLSMP_MASK
#define ADLSMP_SHORT      0x00

// How many bits for the conversion?  8, 12, 10, or 16 (single ended).
#define MODE_8            0x00
#define MODE_12           0x01
#define MODE_10           0x02
#define MODE_16           0x03



// ADC Input Clock Source choice? Bus clock, Bus clock/2, "altclk", or the 
//                                ADC's own asynchronous clock for less noise
#define ADICLK_BUS        0x00
#define ADICLK_BUS_2      0x01
#define ADICLK_ALTCLK     0x02
#define ADICLK_ADACK      0x03

//// ADCCFG2

// Select between B or A channels
#define MUXSEL_ADCB       ADC_CFG2_MUXSEL_MASK
#define MUXSEL_ADCA       0x00

// Ansync clock output enable: enable, or disable the output of it
#define ADACKEN_ENABLED   ADC_CFG2_ADACKEN_MASK
#define ADACKEN_DISABLED  0x00

// High speed or low speed conversion mode
#define ADHSC_HISPEED     ADC_CFG2_ADHSC_MASK
#define ADHSC_NORMAL      0x00

// Long Sample Time selector: 20, 12, 6, or 2 extra clocks for a longer sample time
#define ADLSTS_20          0x00
#define ADLSTS_12          0x01
#define ADLSTS_6           0x02
#define ADLSTS_2           0x03

////ADCSC2

// Read-only status bit indicating conversion status
#define ADACT_ACTIVE       ADC_SC2_ADACT_MASK
#define ADACT_INACTIVE     0x00

// Trigger for starting conversion: Hardware trigger, or software trigger.
// For using PDB, the Hardware trigger option is selected.
#define ADTRG_HW           ADC_SC2_ADTRG_MASK
#define ADTRG_SW           0x00

// ADC Compare Function Enable: Disabled, or Enabled.
#define ACFE_DISABLED      0x00
#define ACFE_ENABLED       ADC_SC2_ACFE_MASK

// Compare Function Greater Than Enable: Greater, or Less.
#define ACFGT_GREATER      ADC_SC2_ACFGT_MASK
#define ACFGT_LESS         0x00

// Compare Function Range Enable: Enabled or Disabled.
#define ACREN_ENABLED      ADC_SC2_ACREN_MASK
#define ACREN_DISABLED     0x00

// DMA enable: enabled or disabled.
#define DMAEN_ENABLED      ADC_SC2_DMAEN_MASK
#define DMAEN_DISABLED     0x00

// Voltage Reference selection for the ADC conversions
// (***not*** the PGA which uses VREFO only).
// VREFH and VREFL (0) , or VREFO (1).

#define REFSEL_EXT         0x00
#define REFSEL_ALT         0x01
#define REFSEL_RES         0x02     /* reserved */
#define REFSEL_RES_EXT     0x03     /* reserved but defaults to Vref */

////ADCSC3

// Calibration begin or off
#define CAL_BEGIN          ADC_SC3_CAL_MASK
#define CAL_OFF            0x00

// Status indicating Calibration failed, or normal success
#define CALF_FAIL          ADC_SC3_CALF_MASK
#define CALF_NORMAL        0x00

// ADC to continously convert, or do a sinle conversion
#define ADCO_CONTINUOUS    ADC_SC3_ADCO_MASK
#define ADCO_SINGLE        0x00

// Averaging enabled in the ADC, or not.
#define AVGE_ENABLED       ADC_SC3_AVGE_MASK
#define AVGE_DISABLED      0x00

// How many to average prior to "interrupting" the MCU?  4, 8, 16, or 32
#define AVGS_4             0x00
#define AVGS_8             0x01
#define AVGS_16            0x02
#define AVGS_32            0x03

////PGA

// PGA enabled or not?
#define PGAEN_ENABLED      ADC_PGA_PGAEN_MASK
#define PGAEN_DISABLED     0x00 

// Chopper stabilization of the amplifier, or not.
#define PGACHP_CHOP        ADC_PGA_PGACHP_MASK 
#define PGACHP_NOCHOP      0x00

// PGA in low power mode, or normal mode.
#define PGALP_LOW          ADC_PGA_PGALP_MASK
#define PGALP_NORMAL       0x00

// Gain of PGA.  Selectable from 1 to 64.
#define PGAG_1             0x00
#define PGAG_2             0x01
#define PGAG_4             0x02
#define PGAG_8             0x03
#define PGAG_16            0x04
#define PGAG_32            0x05
#define PGAG_64            0x06

#define POT_ADC_CHANNEL		 0

#define  FILESMAX 1024


typedef struct adc_cfg {
  uint8_t  CONFIG1; 
  uint8_t  CONFIG2; 
  uint16_t COMPARE1; 
  uint16_t COMPARE2; 
  uint8_t  STATUS2;
  uint8_t  STATUS3; 
  uint8_t  STATUS1A; 
  uint8_t  STATUS1B;
  uint32_t PGA;
  } *tADC_ConfigPtr, tADC_Config ;  
  

#define CAL_BLK_NUMREC 18 
#define PIT_delay 0x600B

typedef struct adc_cal {
 
uint16_t  OFS;
uint16_t  PG;
uint16_t  MG;
uint8_t   CLPD;
uint8_t   CLPS;
uint16_t  CLP4;
uint16_t  CLP3;
uint8_t   CLP2;
uint8_t   CLP1;
uint8_t   CLP0;
uint8_t   dummy;
uint8_t   CLMD;
uint8_t   CLMS;
uint16_t  CLM4;
uint16_t  CLM3;
uint8_t   CLM2;
uint8_t   CLM1;
uint8_t   CLM0;
} tADC_Cal_Blk ;  

//-----------------------------------------------------------------------------------------------


void  InitGPIO();
uint8_t TFC_GetDIP_Switch();
void lcd_init();
void InitPIT();

//ADC+DAC
void InitDAC();
void InitADCs();
uint8_t ADC_Cal(ADC_MemMapPtr);

void ADC_Config_Alt(ADC_MemMapPtr, tADC_ConfigPtr);

void ADC_Read_Cal(ADC_MemMapPtr, tADC_Cal_Blk *);

void InitADC0();
void dma_init(void);


void InitUARTs();
void Uart0_Br_Sbr(int sysclk, int baud);
void InitTPM(char n);
void ClockSetup();

#endif /* TFC_bspGPIO_H_ */
