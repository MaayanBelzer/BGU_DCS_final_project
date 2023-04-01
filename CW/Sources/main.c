
//#include "derivative.h" /* include peripheral declarations */

# include "TFC.h"
# include "stdio.h"
#include "string.h"

enum my_state {scan, telemeter,receiveFile, execute, display_file, sleep} stateMachine;
enum fsm {oneTwo, threeFour, five} sendScroll;
int file_content_flag;
char  value[FILESMAX];
char ready = 0;
int file_state;
int current_line = 0;
int text_scroll_flag;
int file_scroll_flag;
int file_send_flag = 0;
float dist=80;
int dist_valid =0;
int degree = 0;
int telemeter_flag =0;
int telemeter_deg =0;
int telemeter_deg_valid =0;
int split_file;
int done_circular;
int offset;
int execute_name_flag = 0;
char execute_name[11];
int execute_valid_flag = 0;
int execute_blink = 0;
int execute_lcd_up = 0;
int execute_lcd_down = 0;
int execute_telemeter=0;
int execute_scan=0;
typedef struct file_struct{
	int files_number;
	int valid_files[3]; //our addition
	char files_name[3][11];
	char* files[3];
	int files_size[3];
}file_struct;

file_struct files_management;

///////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////


int main(void){
	file_content_flag = 0;
	files_management.files_number = 0;
	int i;
	for(i =0;i<3;i++){
		files_management.files_size[i]=0;
		files_management.files[i]=0;
		files_management.valid_files[i]=0;
	}
	
	//file_management files;
	stateMachine = sleep;
	InitGPIO();
	InitPIT();
	lcd_init();
	dma_init();
	ClockSetup();
	InitUARTs();
	InitTPM(0);
	InitTPM(2);
	
	RGB_LED_OFF;
	while (1) {
		lcd_clear();
		lcd_clear();
		DelayUs(200);
		RGB_LED_OFF;
		
		
		if (stateMachine == scan) {
			TPM0_SC |= TPM_SC_CMOD(1); //Start the TPM0 counter
			TPM2_SC |= TPM_SC_CMOD(1); //Start the TPM2 counter
			PIT_MCR &= ~PIT_MCR_MDIS_MASK; //Enable the PIT module
			PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT0 and its interrupt
			while (stateMachine == scan) {	
					if(dist>=0 && dist_valid){
						dist_valid =0;
						char str[11];
						sprintf(str,"%d_%d\n", (int)dist,(int)TPM2_C1V);
						UARTprintf(UART0_BASE_PTR,str);
					}
			}
			TPM0_SC |= TPM_SC_CMOD(0); //Stop the TPM0 counter
			TPM2_SC |= TPM_SC_CMOD(0); //Stop the TPM2 counter
			PIT_MCR |= PIT_MCR_MDIS_MASK; //Stop the PIT counter
			PIT_TCTRL0 = 0; //disable PIT0 and its interrupt
		}
		else if (stateMachine == telemeter) {
			TPM2_SC |= TPM_SC_CMOD(1); //Start the TPM2 counter
			
			if(telemeter_deg_valid){
					TPM2_C1V = telemeter_deg;
					telemeter_deg = 0;
					telemeter_deg_valid =0;
					TPM0_SC |= TPM_SC_CMOD(1); //Start the TPM0 counter
					while (stateMachine == telemeter) {	
						if(dist>=0 && dist_valid){
							dist_valid =0;
							char str[11];
							sprintf(str,"%d\n", (int)dist);
							UARTprintf(UART0_BASE_PTR,str);
							
						}
					}
			}
			TPM0_SC |= TPM_SC_CMOD(0); //Stop the TPM0 counter
			TPM2_SC |= TPM_SC_CMOD(0); //Stop the TPM2 counter
		}
		else if (stateMachine == receiveFile) {
			offset =0;
			split_file =0;
			done_circular=0;
			while (stateMachine == receiveFile) {
				wait();
				if(file_content_flag){
					if(files_management.files_size[files_management.files_number %10] > 1024){
											split_file = 1;
										}
					receive_file_content();
					file_content_flag = 0;
					}
				}
		}
		
		
		else if(stateMachine == execute) {
				while (stateMachine == execute ) {
				wait();
				if(execute_valid_flag){
					int i=0;
					while(i<min(3,files_management.files_number)){
						if(!strcmp(execute_name, files_management.files_name[i])){
							break;
						}
						i++;
						
					}
					if(i ==3){
						lcd_puts("error");
					}
					execute_file(i);
					wait();
						
					}
				}
			
		}
		else if(stateMachine == display_file) {
			while (stateMachine == display_file && file_scroll_flag) {
				wait();
			}
		}
		else { //sleep
			PIT_LDVAL1 = PIT_delay*500;
			execute_valid_flag = 0;
			telemeter_deg_valid =0;
			stop();         // Low Power Mode
			RGB_LED_OFF;
		}
	}

	
	return 0;
	
}


