#include "TFC.h"

extern char ready;
extern char  value[FILESMAX];
void InitGPIO(){
	//enable Clocks to all ports
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	//Setup Pins as GPIO
	PORTE_PCR21 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   
	PORTE_PCR20 = PORT_PCR_MUX(1); 
	PORTB_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	
	//GPIO Configuration - Pushbutton - Input
	PORTD_PCR7 = PORT_PCR_MUX(1); // assign PTD7 as GPIO
	GPIOD_PDDR &= ~PORT_LOC(7);  // PTD7 is Input
	PORTD_PCR7 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK | PORT_PCR_IRQC(0x0a);
	PORTD_PCR6 = PORT_PCR_MUX(1); // assign PTD6 as GPIO
	GPIOD_PDDR &= ~PORT_LOC(6);  // PTD6 is Input
	PORTD_PCR6 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK | PORT_PCR_IRQC(0x0a);
	enable_irq(INT_PORTD-16); // Enable Interrupts
	set_irq_priority (INT_PORTD-16,0);  // Interrupt priority = 0 = max
	
	
	//GPIO Configuration - DIP Switches - Input
	PORTC_PCR4 = PORT_PCR_MUX(1); // assign PTC4 as GPIO
	PORTC_PCR5 = PORT_PCR_MUX(1); // assign PTC5 as GPIO
	PORTC_PCR6 = PORT_PCR_MUX(1); // assign PTC6 as GPIO
	
	//GPIO Configuration - LEDs - Output
	PORTD_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;  //Blue
	GPIOD_PDDR |= BLUE_LED_LOC; //Setup as output pin
	
	PORTB_PCR18 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Red  
    PORTB_PCR19 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Green
    GPIOB_PDDR |= RED_LED_LOC + GREEN_LED_LOC; //Setup as output pins
    
    // LCD
       PORTE_PCR3 =  PORT_PCR_MUX(1); // LCD - RS
       PORTE_PCR4 =  PORT_PCR_MUX(1); // LCD - R/w
       PORTE_PCR5 =  PORT_PCR_MUX(1); // LCD - ENABLE

       PORTB_PCR0 = PORT_PCR_MUX(1); // LCD - DATA OUTPUT
       PORTB_PCR1 = PORT_PCR_MUX(1); // LCD - DATA OUTPUT
       PORTB_PCR2 = PORT_PCR_MUX(1); // LCD - DATA OUTPUT
       PORTB_PCR3 = PORT_PCR_MUX(1); // LCD - DATA OUTPUT

       GPIOB_PDDR |= 0xF; // Setup LCD DATA pins as output
  
       //ENGINE
       PORTC_PCR12 = PORT_PCR_MUX(1); // ENGINE - DATA OUTPUT
       PORTC_PCR13 = PORT_PCR_MUX(1); // ENGINE - DATA OUTPUT
       PORTC_PCR16 = PORT_PCR_MUX(1); // ENGINE - DATA OUTPUT
       PORTC_PCR17 = PORT_PCR_MUX(1); // ENGINE - DATA OUTPUT
       
       //TPM
       PORTE_PCR23 = PORT_PCR_MUX(3); // Motor - assign PTE23 as TPM2_CH1
       PORTC_PCR1 = PORT_PCR_MUX(4); // Trigger - assign PTC1 as TPM0_CH0
       PORTC_PCR2 = PORT_PCR_MUX(4); // Echo - assign PTC2 as TPM0_CH1
	   
	   
       
       
       
}

//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(){
  
	char init_value;

	if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
        else init_value = 0x3F;
	
	LCD_RS_DIR(OUTPUT_PIN);
	LCD_EN_DIR(OUTPUT_PIN);
	LCD_RW_DIR(OUTPUT_PIN);
        LCD_DATA_DIR |= OUTPUT_DATA;
        LCD_RS(0);
	LCD_EN(0);
	LCD_RW(0);
        
	DelayMs(15);
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	DelayMs(5);
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	DelayUs(200);
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	
	if (LCD_MODE == FOURBIT_MODE){
		LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
		LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
		lcd_strobe();
		lcd_cmd(0x28); // Function Set
	}
        else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots 
	
	lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
	lcd_cmd(0x1); //Display Clear
	lcd_cmd(0x6); //Entry Mode
	lcd_cmd(0x80); //Initialize DDRAM address to zero
}



void InitPIT(){
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable the Clock to the PIT Modules
	PIT_LDVAL0 = PIT_delay * 50;
	PIT_LDVAL1 = PIT_delay*500;
	PIT_MCR |= PIT_MCR_FRZ_MASK; // stop the pit when in debug mode
	enable_irq(INT_PIT-16); //  //Enable PIT IRQ on the NVIC
	set_irq_priority(INT_PIT-16,1);  // Interrupt priority = 0 = max
}


//-----------------------------------------------------------------
// DipSwitch data reading
//-----------------------------------------------------------------
uint8_t TFC_GetDIP_Switch(){
	
	uint8_t DIP_Val=0;
	
	DIP_Val = (GPIOC_PDIR>>4) & 0xF;

	return DIP_Val;
}

//-----------------------------------------------------------------
//  DMA configuration
//-----------------------------------------------------------------

void dma_init(void)
{
	ready = 0;
	
	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;
	
	// Disable DMA Mux channel first
	DMAMUX0_CHCFG0 = 0x00;
	
	// Configure DMA
	DMA_SAR0 = (uint32_t)&UART0_D; // DMA source
	
	DMA_DSR_BCR0 = DMA_DSR_BCR_DONE_MASK;
	
	DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
				 DMA_DCR_ERQ_MASK |		// Enable peripheral request
				 DMA_DCR_CS_MASK  |
				 DMA_DCR_SSIZE(1) |		// Set source size to 16 bits
				 DMA_DCR_DINC_MASK|		// Set increments to destination address
				 DMA_DCR_DMOD(8)  |     // Destination address modulo of 16 Bytes
				 DMA_DCR_DSIZE(1));		// Set destination size of 16 bits 
				 
	
	DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_SOURCE(2);
}




void dma_init1(void){
			DMAMUX0_CHCFG1 = 0x00;
			
			// Configure DMA
			DMA_SAR1 = (uint32_t)&value;
			DMA_DAR1 = (uint32_t)&DAC0_DAT0L;
			DMA_DSR_BCR1 = DMA_DSR_BCR_BCR(2048); 
			
			DMA_DCR1 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
						 DMA_DCR_ERQ_MASK |		// Enable peripheral request
						 DMA_DCR_CS_MASK  |		// cycle stealing
						 DMA_DCR_SSIZE(2) |		// Set source size to 16 bits
						 DMA_DCR_SINC_MASK|		// Set increments to destination address
						 DMA_DCR_SMOD(8)  |     // Destination address modulo of 2Kb Bytes	
						 DMA_DCR_DSIZE(2));		// Set destination size of 16 bits 
						 
			
			DMAMUX0_CHCFG1 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(60) | DMAMUX_CHCFG_TRIG_MASK; 
	
	enable_irq(INT_DMA1 - 16);
}





//-----------------------------------------------------------------
//  UART0 configuration
//-----------------------------------------------------------------
void InitUARTs(){
	
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // Make sure clock for PORTA is enabled
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK; // Enable peripheral clock
 
	PORTA_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;  // RX 
	PORTA_PCR2 = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;  // TX
	
	//Select PLL/2 Clock
	SIM_SOPT2 &= ~(3<<26);
	SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1); 
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
	
	//We have to feed this function the clock in KHz!
	Uart0_Br_Sbr(CORE_CLOCK/2/1000, SDA_SERIAL_BAUD);
	 //Enable receive interrupts
     
	UART0_C2 = UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK | UART_C2_RIE_MASK; // Enable Transmitter, Receiver, Receive interrupt
	set_irq_priority(INT_UART0-16,0);
	enable_irq(INT_UART0-16);
	
}

//--------------------------------------------------------------------
//  UART0 - Selection of BR (Baud Rate) and OSR (Over Sampling Ratio)
//--------------------------------------------------------------------
void Uart0_Br_Sbr(int sysclk, int baud){
	
    uint8 i;
    uint32 calculated_baud = 0;
    uint32 baud_diff = 0;
    uint32 osr_val = 0;
    uint32 sbr_val, uart0clk;
    uint32 baud_rate;
    uint32 reg_temp = 0;
    uint32 temp = 0;
    
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    
    // Disable UART0 before changing registers
    UART0_C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);
  
    // Verify that a valid clock value has been passed to the function 
    if ((sysclk > 50000) || (sysclk < 32))
    {
        sysclk = 0;
        reg_temp = SIM_SOPT2;
        reg_temp &= ~SIM_SOPT2_UART0SRC_MASK;
        reg_temp |= SIM_SOPT2_UART0SRC(0);
        SIM_SOPT2 = reg_temp;
			
			  // Enter infinite loop because the 
			  // the desired system clock value is 
			  // invalid!!
			  while(1);
				
    }
   
    
    // Initialize baud rate
    baud_rate = baud;
    
    // Change units to Hz
    uart0clk = sysclk * 1000;
    // Calculate the first baud rate using the lowest OSR value possible.  
    i = 4;
    sbr_val = (uint32)(uart0clk/(baud_rate * i));
    calculated_baud = (uart0clk / (i * sbr_val));
        
    if (calculated_baud > baud_rate)
        baud_diff = calculated_baud - baud_rate;
    else
        baud_diff = baud_rate - calculated_baud;
    
    osr_val = i;
        
    // Select the best OSR value
    for (i = 5; i <= 32; i++)
    {
        sbr_val = (uint32)(uart0clk/(baud_rate * i));
        calculated_baud = (uart0clk / (i * sbr_val));
        
        if (calculated_baud > baud_rate)
            temp = calculated_baud - baud_rate;
        else
            temp = baud_rate - calculated_baud;
        
        if (temp <= baud_diff)
        {
            baud_diff = temp;
            osr_val = i; 
        }
    }
    
    if (baud_diff < ((baud_rate / 100) * 3))
    {
        // If the OSR is between 4x and 8x then both
        // edge sampling MUST be turned on.  
        if ((osr_val >3) && (osr_val < 9))
            UART0_C5|= UART0_C5_BOTHEDGE_MASK;
        
        // Setup OSR value 
        reg_temp = UART0_C4;
        reg_temp &= ~UART0_C4_OSR_MASK;
        reg_temp |= UART0_C4_OSR(osr_val-1);
    
        // Write reg_temp to C4 register
        UART0_C4 = reg_temp;
        
        reg_temp = (reg_temp & UART0_C4_OSR_MASK) + 1;
        sbr_val = (uint32)((uart0clk)/(baud_rate * (reg_temp)));
        
         /* Save off the current value of the uartx_BDH except for the SBR field */
        reg_temp = UART0_BDH & ~(UART0_BDH_SBR(0x1F));
   
        UART0_BDH = reg_temp |  UART0_BDH_SBR(((sbr_val & 0x1F00) >> 8));
        UART0_BDL = (uint8)(sbr_val & UART0_BDL_SBR_MASK);
        
        /* Enable receiver and transmitter */
        UART0_C2 |= (UART0_C2_TE_MASK
                    | UART0_C2_RE_MASK );
    }
    else
		{
        // Unacceptable baud rate difference
        // More than 3% difference!!
        // Enter infinite loop!
        while(1);
			
		}					
    
}

//-----------------------------------------------------------------
// TPMx - Initialization
//-----------------------------------------------------------------
void InitTPM(char n){
	switch(n){
	case 0:
		TPM0_SC = 0; 
		TPM0_SC |= TPM_SC_PS(7);
		TPM0_MOD = 0x2DC7; 
		TPM0_C0SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK ;
		TPM0_C0V = 0x07; 
		TPM0_C1SC |= TPM_CnSC_ELSB_MASK + TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK + TPM_CnSC_CHF_MASK;//input capture 
		TPM0_CONF |= TPM_CONF_DBGMODE(3);
		enable_irq(INT_TPM0-16); // Enable Interrupts 
		set_irq_priority (INT_TPM0-16,0);  // Interrupt priority = 0 = max
				
		break;
	case 1:	
		break;
		
	case 2: 
		TPM2_SC = 0; 
		TPM2_SC |= TPM_SC_PS(7)+TPM_SC_TOIE_MASK;
		TPM2_MOD = 0x124F; 
		TPM2_C1SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK + TPM_CnSC_CHIE_MASK;
		TPM2_C1V = 0x60;
		TPM2_CONF |= TPM_CONF_DBGMODE(3); 
		
		break;
	default:
		break;
	}
	
}


//-----------------------------------------------------------------
// TPMx - Clock Setup
//-----------------------------------------------------------------
void ClockSetup(){
	    
	    pll_init(8000000, LOW_POWER, CRYSTAL,4,24,MCGOUT); //Core Clock is now at 48MHz using the 8MHZ Crystal
		
	    //Clock Setup for the TPM requires a couple steps.
	    //1st,  set the clock mux
	    //See Page 124 of f the KL25 Sub-Family Reference Manual
	    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want MCGPLLCLK/2=24MHz (See Page 196 of the KL25 Sub-Family Reference Manual
	    SIM_SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK);
	    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1); //We want the MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual
		//Enable the Clock to the TPM0 and PIT Modules
		//See Page 207 of f the KL25 Sub-Family Reference Manual
		SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK + SIM_SCGC6_TPM2_MASK;
	    // TPM_clock = 24MHz , PIT_clock = 48MHz
}
