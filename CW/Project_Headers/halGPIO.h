#ifndef TFC_halGPIO_H_
#define TFC_halGPIO_H_

void PORTD_IRQHandler(void);
void PIT_IRQHandler();
void RGB_Serial_Show(uint8_t color);
int Led_Delay(); 
void print_num(int num);
int min(int num1, int num2);
void FTM0_IRQHandler();
void FTM2_IRQHandler();
void DistanceMeasure (uint16_t samples , uint8_t Xcm);
char uart_getchar (UART_MemMapPtr channel);
void uart_putchar (UART_MemMapPtr channel, char ch);
int uart_getchar_present (UART_MemMapPtr channel);
void UARTprintf(UART_MemMapPtr channel,char* str);
void UART0_IRQHandler();

extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);
extern float Distance[40];
void receive_file_content();
void DMA0_IRQHandler(void);
void execute_file(int i);
int convert_hex(int len, char* hex);

#endif /* TFC_halpGPIO_H_ */
