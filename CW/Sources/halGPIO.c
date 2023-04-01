#include "TFC.h"
typedef struct file_struct{
	int files_number;
	int valid_files[3]; //our addition
	char files_name[3][11];
	char* files[3];
	int files_size[3];
}file_struct;

extern enum my_state {scan, telemeter, receiveFile, execute, display_file, sleep} stateMachine;
extern enum fsm {oneTwo, threeFour, five} sendScroll;
extern file_struct files_management;
extern int file_content_flag;
extern int text_scroll_flag;
extern int file_scroll_flag;
extern int file_send_flag;
extern float dist;
extern int dist_valid;
extern int degree;
extern int telemeter_flag;
extern int telemeter_deg;
extern int telemeter_deg_valid;
extern int split_file;
extern int offset;
extern int done_circular;
extern int execute_name_flag;
extern char execute_name[11];
extern int execute_valid_flag;
extern int execute_blink;
extern int execute_lcd_up;
extern int execute_lcd_down;
extern int execute_telemeter;
extern int execute_scan;

int arg;
int arg2;
int keep=0;

char clr = 'Z';
int counter = 0;
int temp_num=0;
extern int file_state;
extern int current_line;
int circular_flag = 0;

extern char  value[FILESMAX];


int convert_hex(int len,char* hex){ // This function converts a hexadecimal number to a decimal number
	int i;
	int num = 0;
	int base = 1;
	
	for(i= len--;i>=0;i--){
		if(*(hex+i)>='0' && *(hex+i)<='9')
		{
			num += (*(hex+i) - 48) * base;
			base *= 16;
		}
		else if(*(hex+i)>='a' && *(hex+i)<='f')
		{
			num += (*(hex+i) - 87) * base;
			base *= 16;
		}
		else if(*(hex+i)>='A' && *(hex+i)<='F')
		{
			num +=   (*(hex+i) - 55) * base;
			base *= 16;
		}
	}
	return num;
}


void print_num(int num){ // This function prints a number on the LCD display
	char str[5];
	sprintf(str, "%d", num);
	lcd_puts(str);
}

int min(int num1, int num2){ // This function returns the minimum argument out of two
	if (num1<=num2){
		return num1;
	}
	return num2;
}

//-----------------------------------------------------------------
//  RGB Serial Show function
//-----------------------------------------------------------------
void RGB_Serial_Show(uint8_t color){
	
	if(color & 0x01) RED_LED_ON;
	else RED_LED_OFF;
	
	if(color & 0x02)GREEN_LED_ON;
	else GREEN_LED_OFF;
	
	if(color & 0x04)BLUE_LED_ON;
	else BLUE_LED_OFF;
}


//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){
  
	LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

	if (LCD_MODE == FOURBIT_MODE)
	{
		LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
		lcd_strobe();
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
    		LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
		lcd_strobe();
	}
	else
	{
		LCD_DATA_WRITE = c;
		lcd_strobe();
	}
}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){
        
	LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

	LCD_DATA_WRITE &= ~OUTPUT_DATA;       
	LCD_RS(1);
	if (LCD_MODE == FOURBIT_MODE)
	{
    		LCD_DATA_WRITE &= ~OUTPUT_DATA;
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;  
		lcd_strobe();		
                LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
		LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET; 
		lcd_strobe();
	}
	else
	{
		LCD_DATA_WRITE = c;
		lcd_strobe();
	}
          
	LCD_RS(0);   
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){
  
	while(*s){
//		if(*s==0x0a){
//			lcd_new_line;
//			*s++;
//			continue;
//		}
		lcd_data(*s++);
	}
}

//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){
  
	unsigned char i;
        for(i=cnt ; i>0 ; i--) asm("nop"); // tha command asm("nop") takes raphly 1usec
	
}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){
  
	unsigned char i;
        for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec
	
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm("nop");
  asm("nop");
  LCD_EN(0);
}

//******************************************************************
// UART functions
//******************************************************************
/********************************************************************/
/*
 * Wait for a character to be received on the specified uart
 *
 * Parameters:
 *  channel      UART channel to read from
 *
 * Return Values:
 *  the received character
 */
char uart_getchar (UART_MemMapPtr channel)
{
      /* Wait until character has been received */
      while (!(UART_S1_REG(channel) & UART_S1_RDRF_MASK));
    
      /* Return the 8-bit data from the receiver */
      return UART_D_REG(channel);
}
/********************************************************************/
/*
 * Wait for space in the uart Tx FIFO and then send a character
 *
 * Parameters:
 *  channel      UART channel to send to
 *  ch			 character to send
 */ 
void uart_putchar (UART_MemMapPtr channel, char ch)
{
      /* Wait until space is available in the FIFO */
      while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK));
    
      /* Send the character */
      UART_D_REG(channel) = (uint8)ch;
    
 }
/********************************************************************/
/*
 * Check to see if a character has been received
 *
 * Parameters:
 *  channel      UART channel to check for a character
 *
 * Return values:
 *  0       No character received
 *  1       Character has been received
 */
int uart_getchar_present (UART_MemMapPtr channel)
{
    return (UART_S1_REG(channel) & UART_S1_RDRF_MASK);
}
/********************************************************************/
/*
 * Wait for space in the uart Tx FIFO and then send a string
 *
 * Parameters:
 *  channel      UART channel to send to
 *  str			 string to send
 */ 
void UARTprintf(UART_MemMapPtr channel,char* str){
	volatile unsigned char i;
	
	for (i=0 ; str[i] ; i++){
	
	      while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK)); /* Wait until space is available in the FIFO */
	    
	      UART_D_REG(channel) = str[i]; /* Send the character */
	}
}

//-----------------------------------------------------------------
//  PORTD Input interrupt ISR
//-----------------------------------------------------------------

void PORTD_IRQHandler(void){
	volatile unsigned int i;
	// check that the interrupt was for switch
	
	if((PORTD_ISFR & PUSH_BUTT0N1_LOC) && stateMachine == display_file ){
			if(files_management.valid_files[file_state]){
				text_scroll_flag =1;
				file_scroll_flag =0;
			}
	}
	
	
	if((PORTD_ISFR & PUSH_BUTT0N0_LOC) && ( (stateMachine==display_file && file_scroll_flag)) ){
		file_state = (file_state + 1)%(files_management.files_number);
	}
	
	if((PORTD_ISFR & PUSH_BUTT0N0_LOC) && (stateMachine==display_file && text_scroll_flag) ){
		current_line = (current_line + 1)%((files_management.files_size[file_state]/16) +1 );
	}
	
	
	//Debounce or using PFE field
	while(!(GPIOD_PDIR & (SW_POS | PUSH_BUTT0N1_LOC) ));// wait of release the button
	for(i=10000 ; i>0 ; i--); //delay, button debounce
	
	while(!(GPIOD_PDIR & (SW_POS | PUSH_BUTT0N0_LOC) ));// wait of release the button
	for(i=10000 ; i>0 ; i--); //delay, button debounce
	
	PORTD_ISFR |= 0x00000080;  // clear interrupt flag bit of PTD7
}

//-----------------------------------------------------------------
// PIT - ISR = Interrupt Service Routine
//-----------------------------------------------------------------

void PIT_IRQHandler(){
	if(stateMachine == execute){
		PIT_TFLG1 = PIT_TFLG_TIF_MASK;
		if(execute_scan){
			PIT_TFLG0 = PIT_TFLG_TIF_MASK;
			if( arg < arg2){
				TPM2_C1V +=7;
				if(TPM2_C1V >= arg2 ){
					  execute_scan =0 ;
					  return;
				}
			}
			else{
				TPM2_C1V -=7;
				if(TPM2_C1V <= arg2 ){
					  execute_scan =0 ;
					  return;
				}
			}
			return;
			
		}
		else if (execute_blink){
			static uint8_t color = 0;
			static uint8_t color_state = 0;
			if (color_state == 0){
				color_state = 1;
				if(color == 0x8){
					arg--;
					color = 0;
				}
				RGB_Serial_Show(color);
				color++;
			}
			else {
				RGB_LED_OFF;
				color_state = 0;
			}
		}
		else if(execute_lcd_up){
			static uint8_t num = 0;
			lcd_clear();
			lcd_clear();
			lcd_clear();
			lcd_clear();
			RGB_LED_OFF;
			
			if(num == 11){
				arg--;
				num = 0;
				return;
			}
			print_num(num);
			RGB_LED_OFF;
			num++;
		}
		else if(execute_lcd_down){
			static int num = 10;
			lcd_clear();
			lcd_clear();
			lcd_clear();
			lcd_clear();
			RGB_LED_OFF;
			
			if(num == -1){
				arg--;
				num = 10;
				return;
			}
			print_num(num);
			RGB_LED_OFF;
			num--;
		}
		return;
	}
	PIT_TFLG0 = PIT_TFLG_TIF_MASK;
	static uint8_t up = 0;
	
	if(up){
		TPM2_C1V +=7;
		  if(TPM2_C1V >= 0x1BF ){
			up = 0;
		}
	}
	else {
		
		TPM2_C1V -=7;
		if(TPM2_C1V <= 0x66 ){
			up = 1;
		}
	}			
}




//-----------------------------------------------------------------
//  TPM1 - ISR = Interrupt Service Routine - Trigger
//-----------------------------------------------------------------
void FTM0_IRQHandler(){

	TPM0_C1SC |= TPM_CnSC_CHF_MASK; 				//Manual flag down of the timer
	
	static int Rise = 1;
	static int captureR =0;
	static int captureF =0;
	if (Rise ){
		Rise = 0;
		captureR = TPM0_C1V;    // Time of rising edge in Echo pulse
	}
	else{
		Rise = 1;
		captureF = TPM0_C1V;   // Time of falling edge in Echo pulse
		dist = 5*((captureF - captureR)/43.3);  //Calculate distance from sensor (in room temp)
		dist_valid = 1;
	}
}


//-----------------------------------------------------------------
//  UART0 - ISR
//-----------------------------------------------------------------
void UART0_IRQHandler(){
	static uint8_t file_name_flag =0;
	static uint8_t file_size_flag =0;
	static uint8_t file_name_index=0;
	static uint8_t execute_name_index=0;
	uint8_t Temp;
	static int mask =0;
	
	if(UART0_S1 & UART_S1_RDRF_MASK){ // RX buffer is full and ready for reading
		Temp = UART0_D;
		if(mask){
					mask =0;
					return;
				}
		if (stateMachine == telemeter && telemeter_flag) {
			if (telemeter_flag){
				if(Temp != 0x0a){ 
					telemeter_deg*=10;
					telemeter_deg+=(Temp-0x30);
				}
				else {
						telemeter_deg_valid =1;
						telemeter_flag =0;
						return;
				}
			}

		}
		else if (stateMachine == receiveFile) {
			if (file_name_flag){
				if(Temp != 0x0a){ 
					files_management.files_name[files_management.files_number %3][file_name_index] = Temp;
					file_name_index++;
				}
				else {	
						files_management.files_name[files_management.files_number %3][file_name_index] = '\0';
						file_name_index =0;
						file_name_flag =0;
						file_size_flag = 1;
						files_management.files_size[files_management.files_number %3] = 0;
						return;
				}
			}
			if (file_size_flag){
				if(Temp != 0x0a){ 
					files_management.files_size[files_management.files_number %3]*=10;
					files_management.files_size[files_management.files_number %3]+=(Temp-0x30);
				}
				else {
						file_size_flag = 0;
						if (min(1024, files_management.files[files_management.files_number %3] + files_management.files_size[files_management.files_number %3]) > value + FILESMAX) {
							circular_flag = 1;
						}
						file_content_flag = 1;
						return;
				}
			}	
		}
		else if (stateMachine == execute) {
			if (execute_name_flag){
				if(Temp != 0x0a){ 
					execute_name[execute_name_index] = Temp;
					execute_name_index++;
				}
				else {	
					execute_name[execute_name_index] = '\0';
					execute_name_index = 0;
					execute_name_flag = 0;
					execute_valid_flag = 1;
					return;
				}
			}
			else if (execute_telemeter){
				if(Temp == 0x0a){ 
					execute_telemeter=0;
				}
			}
			else if (keep){
				keep = 0;
			}
		}
		else{
			switch(Temp){
		
				
			case 0x31:			
				mask =1;
				stateMachine = scan;
				break;
			case 0x32: 
				mask =1;
				lcd_clear();
				lcd_clear();
				stateMachine = telemeter;
				telemeter_flag = 1;
				break;
			case 0x33:
				mask =1;
				stateMachine = receiveFile;
				if(!(files_management.files_number)){
					files_management.files[0] = value;
				}
				else{
					char* next;
					next = files_management.files[(files_management.files_number-1) %3] + files_management.files_size[(files_management.files_number-1) %3];
					if (next > value+ FILESMAX){
						next = files_management.files_size[(files_management.files_number-1) %3]+files_management.files[(files_management.files_number-1) %3] - FILESMAX ;
					}
					files_management.files[files_management.files_number % 3] = next;
				}
				file_name_flag = 1;
				break;
			case 0x34:			
				mask =1;
				stateMachine = execute;
				execute_name_flag = 1;
				break;
			case 0x35:				//voltage
				mask =1;
				stateMachine = display_file;
				break;
			default:				//sleep
				mask =1;
				stateMachine = sleep;
				break;
			
			}
		}
	}
}


void receive_file_content(){
	DMA_DAR0 = (uint32_t)(files_management.files[files_management.files_number %3]);
	if (circular_flag) {
		DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(min(value + FILESMAX - files_management.files[files_management.files_number%3], 1024));  // number of bytes to be transferred
	}
	else if(!split_file) {

		DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(files_management.files_size[(files_management.files_number) %3]);  // number of bytes to be transferred
	}
	else{
		DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(1024);
	}
	
	DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK; 				// Enable DMA channel 
	//disable_irq(INT_UART0-16);               			    // Disable UART0 interrupt
	UART0_C5 |= UART0_C5_RDMAE_MASK;         				// Enable DMA request for UART0 receiver
	enable_irq(INT_DMA0 - 16);
}



void DMA0_IRQHandler(void)
{
	int i;
		if(circular_flag){
			BLUE_LED_ON;
			DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;// Clear Done Flag
			DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(min(1024, files_management.files_size[files_management.files_number%3] -(value + FILESMAX - files_management.files[files_management.files_number%3])));
			DMA_DAR0 = (uint32_t)value; 
			for(i=0;i<min(files_management.files_number, 3);i++){
				if(files_management.files[i]<= value +files_management.files_size[files_management.files_number%3] -(value + FILESMAX - files_management.files[files_management.files_number%3])){
					files_management.valid_files[i] = 0;
				}
				
			}
			circular_flag=0;
			done_circular =1;
			if (split_file){
				offset += min(1024, value + FILESMAX - files_management.files[files_management.files_number%3]);
				if (files_management.files_size[(files_management.files_number) %3] - offset < 1024){
					split_file =0;
				}
			}
			return;
		}

		if(split_file){
			DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;// Clear Done Flag	
			offset += min(1024, value + FILESMAX - files_management.files[files_management.files_number%3]);
			if (done_circular){
				DMA_DAR0 = (uint32_t)(value + offset);
			}
			else {
				DMA_DAR0 = (uint32_t)(files_management.files[files_management.files_number %3] + offset);
			}
			if ((files_management.files[(files_management.files_number) %3]+ offset + min(files_management.files_size[(files_management.files_number) %3] - offset, 1024) > value + FILESMAX) && !done_circular){
				circular_flag = 1;
				DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(min(value + FILESMAX - files_management.files[files_management.files_number%3] - offset, 1024));
			}
			else {
				DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(min(files_management.files_size[(files_management.files_number) %3] - offset, 1024));
			}
			
			if (files_management.files_size[(files_management.files_number) %3] - offset < 1024){
				split_file =0;
			}
			return;
		}
		
		for(i=0;i<min(files_management.files_number, 3);i++){
			if((files_management.files[i]<=files_management.files_size[files_management.files_number%3] + files_management.files[files_management.files_number%3]) && (files_management.files[files_management.files_number%3] < files_management.files[i])){
				files_management.valid_files[i] = 0;
			}
		}
		
		DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;			// Clear Done Flag
		DMAMUX0_CHCFG0 &= ~DMAMUX_CHCFG_ENBL_MASK;	    // Disable DMA Channel 0
		UART0_C5 &= ~UART0_C5_RDMAE_MASK; 				// Disabling DMA using UART
		enable_irq(INT_UART0-16);						// Enable UART0 interrupt
		RED_LED_ON;
		int j;
		for (j=1000000; j>0; j--);	                    // Delay
		RED_LED_OFF;
		files_management.valid_files[files_management.files_number%3] = 1;
		files_management.files_number = (files_management.files_number+1); // TODO modulo??????????????
		stateMachine = sleep;
		UARTprintf(UART0_BASE_PTR,"Received File!\n");
}

void execute_file(int file_num){
	RGB_LED_OFF;
	int index =0;
	int index_copy = 0;
	char str[15];
	while ((index< files_management.files_size[file_num]) && stateMachine == execute){
		index++;
		switch(*(files_management.files[file_num]+ index)){			
		case 0x31:	//blink_rgb		
			index++;
			index_copy = index;
			arg =0;
			while((*(files_management.files[file_num]+ index_copy) != 0x0d) && index_copy < files_management.files_size[file_num]){ 
				index_copy++;
			}
			arg = convert_hex(index_copy - index -1, files_management.files[file_num]+ index);
			index = index_copy+2;
			execute_blink = 1;
			PIT_MCR &= ~PIT_MCR_MDIS_MASK; //Enable the PIT module
			PIT_TCTRL1 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT1 and its interrupt
			while (arg>0){
				wait();
			}
			PIT_TCTRL0 = 0; //disable PIT0 and its interrupt
			PIT_MCR |= PIT_MCR_MDIS_MASK; //Stop the PIT counter
			PIT_TCTRL1 = 0; //disable PIT1 and its interrupt
			RGB_LED_OFF;
			execute_blink = 0;
			break;
		case 0x32:	//lcd_count_up		
			index++;
			index_copy = index;
			arg =0;
			while((*(files_management.files[file_num]+ index_copy) != 0x0d)&& index_copy < files_management.files_size[file_num]){ 
				index_copy++;
			}
			arg = convert_hex(index_copy - index -1, files_management.files[file_num]+ index);
			index = index_copy+2;
			execute_lcd_up = 1;
			PIT_MCR &= ~PIT_MCR_MDIS_MASK; //Enable the PIT module
			PIT_TCTRL1 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT1 and its interrupt
			while (arg>0){
				wait();
			}
			PIT_MCR |= PIT_MCR_MDIS_MASK; //Stop the PIT counter
			PIT_TCTRL1 = 0; //disable PIT1 and its interrupt
			RGB_LED_OFF;
			execute_lcd_up = 0;
			lcd_clear();
			lcd_clear();
			lcd_clear();
			lcd_clear();
			break;
		case 0x33:	//lcd_count_down
			index++;
			index_copy = index;
			arg =0;
			while((*(files_management.files[file_num]+ index_copy) != 0x0d)&& index_copy < files_management.files_size[file_num]){ 
				index_copy++;
			}
			arg = convert_hex(index_copy - index -1, files_management.files[file_num]+ index);
			index = index_copy+2;
			execute_lcd_down = 1;
			PIT_MCR &= ~PIT_MCR_MDIS_MASK; //Enable the PIT module
			PIT_TCTRL1 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT1 and its interrupt
			while (arg>0){
				wait();
			}
			PIT_MCR |= PIT_MCR_MDIS_MASK; //Stop the PIT counter
			PIT_TCTRL1 = 0; //disable PIT1 and its interrupt
			RGB_LED_OFF;
			execute_lcd_down = 0;
			lcd_clear();
			lcd_clear();
			lcd_clear();
			lcd_clear();
			break;
		case 0x34:	//set_delay		
			index++;
			index_copy = index;
			arg =0;
			while((*(files_management.files[file_num]+ index_copy) != 0x0d)&& index_copy < files_management.files_size[file_num]){ 
				index_copy++;
			}
			arg = convert_hex(index_copy - index -1, files_management.files[file_num]+ index);
			index = index_copy+2;
			PIT_LDVAL1 = PIT_delay*10*arg;
			break;
		case 0x35: //clear_all_leds (no arg)
			RGB_LED_OFF;
			index+=3;// maybe add 1
			break;
		case 0x36:	//servo_deg
			index++;
			index_copy = index;
			arg =0;
			while((*(files_management.files[file_num]+ index_copy) != 0x0d)&& index_copy < files_management.files_size[file_num]){ 
				index_copy++;
			}
			arg = convert_hex(index_copy - index -1, files_management.files[file_num]+ index);
			index = index_copy+2;
			TPM2_C1V = (int)(arg * (351.0 / 180)) + 98;
			TPM2_SC |= TPM_SC_CMOD(1); //Start the TPM2 counter
			TPM0_SC |= TPM_SC_CMOD(1); //Start the TPM0 counter
			execute_telemeter =1;
			
			sprintf(str,"telemeter_%d\n", arg);
			UARTprintf(UART0_BASE_PTR,str);
			while (execute_telemeter){
				if(dist>=0 && dist_valid){
					dist_valid =0;
					char str_telemeter[11];
					sprintf(str_telemeter,"%d\n", (int)dist); 
					UARTprintf(UART0_BASE_PTR,str_telemeter);
					
				}
			}
			TPM0_SC |= TPM_SC_CMOD(0); //Stop the TPM0 counter
			TPM2_SC |= TPM_SC_CMOD(0); //Stop the TPM2 counter
			break;
		case 0x37: //servo_scan		
			index++;
			
			arg = convert_hex(1, files_management.files[file_num]+ index);
			index+=2;
			arg2 = convert_hex(1, files_management.files[file_num]+ index);
			index+=4;
			arg = (int)(arg * (351.0 / 180)) + 98;
			arg2 = (int)(arg2 * (351.0 / 180)) + 98;
			execute_scan =1;
			TPM2_C1V = arg;
			TPM2_SC |= TPM_SC_CMOD(1); //Start the TPM2 counter
			TPM0_SC |= TPM_SC_CMOD(1); //Start the TPM0 counter
			PIT_MCR &= ~PIT_MCR_MDIS_MASK; //Enable the PIT module
			PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT0 and its interrupt
			dist_valid=0;
			UARTprintf(UART0_BASE_PTR,"scan_a\n");
			while (execute_scan) {	
				if(dist>=0 && dist_valid){
					dist_valid =0;
					char str2[11];
					sprintf(str2,"%d_%d\n", (int)dist,(int)TPM2_C1V);
					if (strlen(str2)==0){
						execute_scan=0;
					}
					UARTprintf(UART0_BASE_PTR,str2);
				}
			}
			TPM0_SC |= TPM_SC_CMOD(0); //Stop the TPM0 counter
			TPM2_SC |= TPM_SC_CMOD(0); //Stop the TPM2 counter
			PIT_MCR |= PIT_MCR_MDIS_MASK; //Stop the PIT counter
			PIT_TCTRL0 = 0; //disable PIT0 and its interrupt
			keep =1;
			while(keep){
				wait();
				char str2[11];
				sprintf(str2,"%d_%d\n", (int)dist,(int)TPM2_C1V);
				UARTprintf(UART0_BASE_PTR,str2);
			}
			BLUE_LED_ON;
			break;
		default://sleep (no arg)
			stateMachine = sleep;
			index+=2;
			break;
		}
		
	}
	UARTprintf(UART0_BASE_PTR,"EXIT_A\n");
	stateMachine = sleep;
	
}


