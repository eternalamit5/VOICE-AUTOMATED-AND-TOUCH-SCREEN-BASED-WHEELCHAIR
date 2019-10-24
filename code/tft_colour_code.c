/******************************/
/* Define Color Code For GLCD */
/* Color = RRRRR GGGGGG BBBBB */
/******************************/
#define no_bg           0x0001			// No Color Back Ground
#define BLACK           0x0000
#define WHITE           0xFFFF
#define RED             0x8000
#define GREEN           0x0400
#define BLUE            0x0010
#define YELLOW          0xFFF0
#define CYAN            0x0410
#define MAGENTA         0x8010
#define BROWN           0xFC00
#define OLIVE	          0x8400

#define BRIGHT_RED      0xF800
#define BRIGHT_GREEN    0x07E0
#define BRIGHT_BLUE     0x001F
#define BRIGHT_YELLOW   0xFFE0
#define BRIGHT_CYAN     0x07FF
#define BRIGHT_MAGENTA  0xF81F

#define LIGHT_GRAY      0x8410
#define DARK_GRAY       0x4208
#define LIGHT_BLUE      0x841F
#define LIGHT_GREEN     0x87F0
#define LIGHT_CYAN      0x87FF
#define LIGHT_RED       0xFC10
#define LIGHT_MAGENTA   0xFC1F

#define GRAY0       	0xE71C
#define GRAY1         	0xC618
#define GRAY2           0xA514
#define GRAY3           0x630C
#define GRAY4           0x4208
#define GRAY5	        0x2104
#define	GRAY6	        0x3186	

#define	BLUE0	        0x1086	
#define	BLUE1       	0x3188 
#define	BLUE2          	0x4314
#define BLUE3         	0x861C	

#define	CYAN0	        0x3D34
#define	CYAN1           0x1DF7		

#define	GREEN0          0x0200	
#define	GREEN1          0x0208	

//------------ Color For Build Button -------------
#define BT_RED          1
#define BT_GREEN        2
#define BT_BLUE         3
#define	BT_YELLOW       4
#define BT_WHITE        5
#define BT_CYAN         6
#define BT_MAGENTA      7
#define BT_BROWN        8

#define BT_LRED         9
#define BT_LGREEN       10
#define BT_LBLUE        11
#define	BT_LYELLOW      12
#define BT_LWHITE       13
#define BT_LCYAN        14
#define BT_LMAGENTA     15
#define BT_LBROWN       16

/* GLCD & Touch(ADS7846) Hardware Interface */
#define GLCD_CS_HIGH()  LPC_GPIO0->FIOPIN |=  (1 << 6);										// P0.6(CS=High:Disable)
#define GLCD_CS_LOW()	LPC_GPIO0->FIOPIN &= ~(1 << 6);										// P0.6(CS=Low:Enable)

#define GLCD_BL_ON()	LPC_GPIO0->FIOPIN |=  (1UL << 5); 									// P0.5(BL=High:ON)
#define	GLCD_BL_OFF()   LPC_GPIO0->FIOPIN &= ~(1UL << 5); 									// P0.5(BL=Low:OFF)

#define	TCS_CS_HIGH()	LPC_GPIO1->FIOPIN |=  (1UL<<21);    								// P1.21(CS=High:Disable)
#define	TCS_CS_LOW()    LPC_GPIO1->FIOPIN &= ~(1UL<<21);   									// P1.21(CS=Low:Enable)

#define PENIRQ_MASK     (1 << 10) 																//P2.6 => 8Bit Mask(01000000)
#define PENIRQ_READ()  (LPC_GPIO0->FIOPIN) & PENIRQ_MASK;   								//P2.6 => 8Bit Mask(01000000)

//********** Valiable delay ***********
unsigned long timeval;

// Variable for Touch Screen Function GLCD 
long dif_adc_X,dif_adc_Y;
long buf_adc_X[100],buf_adc_Y[100];
long ave_adc_X,ave_adc_Y;
long tcs_ave_X[3];																			// Keep Value adc Touch Point X
long tcs_ave_Y[3];																			// Keep Value adc Touch Point Y
long dis_XD_hor[3] = {32,287,160}; 															// Value refer Point X at 10% of Display(X=320:0-319) 3 Position
long dis_YD_hor[3] = {215,120,24}; 															// Value refer Point Y at 10% of Display(Y=240:0-239) 3 Position
long divider,An,Bn,Cn,Dn,En,Fn,X,Y;															// Valiable for keep coefficient Calibrat and position touch Screen
char num1,num2,num3;																		// Variable for keep data Dec to Ascii

/* Prototype Function */
void Initial_Hardware(void);																// Config Hardware Interface
void delay_ms(unsigned long ms);															// Delay Time
extern unsigned long int SystemFrequency; 
/* Function Control Graphic LCD (Driver SPFD5408A)*/
unsigned char GLCD_SPI_Read_Write(unsigned char DataByte);
void GLCD_Write_Command(unsigned char GLCD_Command);
void GLCD_Write_Data(unsigned int GLCD_Data);
void Initial_GLCD_Hor(void);

/* Function Control SPI Touch-Screen(ADS7846)*/
char Get_Status_Touch(void);
unsigned char TSC_SPI_Write(unsigned char DataByte);
void TCS_SPI_Read_Hor(void);
void TCS_Average_X_Y(unsigned char num);
void Touch_Calibrate_Hor(void);
void TCS_Set_Matrix_Hor(void);
void TCS_Get_Point(char num);

/* Function Application of GLCD */
void bg_color(long bg_color);
void text_7x11_hor(char row,long adx,long ady,long fg_clr,long bg_clr);
void lcd_printStr_hor(char *str,long cur_x,long cur_y,
                  long fg_color,long bg_color);
void lcd_print3Cha_hor(char ch1,char ch2,char ch3,char ch4,
                   long cur_x,long cur_y,long fg_color,long bg_color);
void dec_to_ascii(long num);
void hardware_init(void);
void glcd_init(void);
void ticket_print(void);
void uart1_init(void);
void ticketprint_uart1(unsigned char *source,unsigned char *to ,unsigned char *destination);
void ticket_price(unsigned long int price);
void  ticket_number(unsigned long int tnp);
void total_amount(unsigned long int tamount);
void stage_server(unsigned long int totaltickets,unsigned long int totalamount1);
unsigned long int tn=0;
unsigned long int avg_x=0,avg_y=0;
unsigned char num11=0,num12=0,num13=0,num14=0;
unsigned char S1='1',S2='2',S3='3',D1='1',D2='2',D3='3',S=0,D=0;
void delay(unsigned long int dk);
 void disply_clear(unsigned short int clst,unsigned short int rost);
unsigned long int totalmoney=0;
void GPIOPORT0_INIT(void);
void GPIOPORT2_INIT(void);