#include "lpc17xx.h"
#include "math.h"
#include "ILI9341.h"
#include "smarttf.h"
unsigned char voice=0;
   /* Font Table */
extern const char ascii_7x11[95][14]; 	// font.c
  /* Main Function Start Here */
   int main(void)
{  
    SystemInit();	
    SysTick_Config(SystemFrequency/1000);							// Generate Interrupt Every 1 mS 
    hardware_init();
	  glcd_init();
	  GPIOPORT0_INIT();
	  GPIOPORT2_INIT();
	  delay_ms(500);
	  bg_color(RED);
	  delay_ms(500);
	  bg_color(WHITE);
	  delay_ms(500);
	  lcd_printStr_hor("WEL COME TO WHEEL CHAIR",60,220,BRIGHT_MAGENTA,WHITE);   				// Plot Text
	  lcd_printStr_hor("LEFT",50,130, RED,WHITE);   	// Plot Text
	  lcd_printStr_hor("RIGHT",240,130,MAGENTA,WHITE);   	// Plot Text
	  lcd_printStr_hor("STOP",140,130,BLACK ,WHITE);   	// Plot Text
	  lcd_printStr_hor("FORWARD",130,190,BROWN ,WHITE);   // Plot Text
	  lcd_printStr_hor("BACKWARD",130,70,OLIVE,WHITE);   	// Plot Text
	while(1)
  {	
    	
		
    if(Get_Status_Touch())         	    								// return (1) if tap touch screen is detected.
  	{
	  TCS_Get_Point(5); 										// Calculate Position Address Display Keep result in valiable X,Y
	  ticket_print();
		
	//	lcd_printStr_hor("Touch X:",200,54,WHITE,BLACK); 					// Plot String Color White
	 // lcd_printStr_hor("Touch Y:",200,34,WHITE,BLACK);

	 // dec_to_ascii(X);	                                        				// Convert Position X-Addres to ASCII
	 // lcd_print3Cha_hor(num14,num13,num12,num11,265,54,BRIGHT_YELLOW,BLACK); 		        // Plot Position X-Address on GLCD
	 // dec_to_ascii(Y);	                                        				// Convert Position Y-Addres to ASCII
	 // lcd_print3Cha_hor(num14,num13,num12,num11,265,34,BRIGHT_YELLOW,BLACK);			// Plot Position Y-Address on GLCD	
   }
		
	 
	 voice=LPC_GPIO2->FIOPIN1;
		 voice=(voice&0x38);
		//BACKWARD
		 if(voice==0X08)
		{
		LPC_GPIO0->FIOSETH=0X0140;	
//		delay(0x50000000);
//		LPC_GPIO0->FIOCLRH=0X01E0;	
			
		}
		//FORWARD
		else if(voice==0X10)
		{
		//LPC_GPIO0->FIOCLRH=0X01E0;	
		LPC_GPIO0->FIOSETH=0X00A0;	
		}
		//RIGHT
		else if(voice==0X18)
		{
		//LPC_GPIO0->FIOCLRH=0X01E0;	
		LPC_GPIO0->FIOSETH=0X0040;
			
		}
		//LEFT
		else if(voice==0X20)
		{
	//	LPC_GPIO0->FIOCLRH=0X01E0;	
	 LPC_GPIO0->FIOSETH=0X0100;		
			
		}
		//STOP
		else if(voice==0X30)
		{
		LPC_GPIO0->FIOCLRH=0X01E0;	
			
		}
  }
}

/*********************************/
/* SysTick Interrupt Handler(1ms)*/
/*********************************/
void SysTick_Handler (void) 
{           
  timeval++;
}

/************/
/* Delay ms */
/************/
void delay_ms(unsigned long ms)
 {
  timeval = 0;  																			//Restart Time(mS) Count
  while(timeval != ms){;}
 }
void GPIOPORT0_INIT(void)
  {
	LPC_SC->PCONP|=0X00000000;
	LPC_PINCON->PINSEL1|=0X00000000;
	LPC_GPIO0->FIODIRH=0X01E0;
	LPC_GPIO0->FIOCLRH=0X01E0;	
	//LPC_GPIO0->FIOSETH=0X01E0;	
  }
void GPIOPORT2_INIT(void)
  {
	LPC_SC->PCONP|=0X00000000;
	LPC_PINCON->PINSEL4|=0X00000000;
	LPC_GPIO2->FIODIR1=0X00;
//	LPC_GPIO2->FIOCLR1=0XFF;	
	}
/*****************************************/
/* Convert Dec to Ascii 3 digit(000-999) */
/*****************************************/        
void dec_to_ascii(long num)
{
  
	
	num11 =(num%0x0A)+0x30;		//Digit-100
  num  =  num/0X0A;
  num12 =(num%0X0A)+0x30;		//Digit-10
	num  =  num/0X0A;
  num13 =(num%0X0A)+0x30;		//Digit-1	 
	num  =  num/0X0A;
  num14 = (num%0X0A)+0x30;		//Digit-1	 
}

/******************************************/
/* Config Hardware Interface GLCD & Touch */
/* (LCD Driver SPFD5408A + Touch ADS7846) */
/* GLCD SPI Mode Interface */
/* -> P0.6(GPIO:Out)    = CS GLCD         */
/* -> P0.7(SCK1:SSP1)   = SCL	GLCD	  */
/* -> P0.8(MISO1:SSP1)  = SDO GLCD		  */
/* -> P0.9(MOSI1:SSP1)  = SDI	GLCD	  */
/* -> P0.5(GPIO:Out)   = BL GLCD		  */
/* Touch GLCD : ADS7846 SPI Interface     */
/* -> P1.20(SCK0:SSP0)  = DCLK ADS7846    */
/* -> P1.21(GPIO:Out)   = CS ADS7846	  */
/* -> P1.23(MISO0:SSP0) = DOUT ADS7846	  */
/* -> P1.24(MOSI0:SSP0) = DIN ADS7846	  */
/* -> P0.21(GPIO:In)    = PENIRQ ADS7846  */
/******************************************/
void hardware_init(void)
{
  /* Config P1.20,P1.21,P1.23,P1.24 to SSP0 For Read Touch LCD(ADS7846) */
  // P1.20(SCK0:SSP0)  = DCLK ADS7846
  // P1.21(GPIO:Out)   = CS ADS7846
  // P1.23(MISO0:SSP0) = DOUT ADS7846
  // P1.24(MOSI0:SSP0) = DIN ADS7846
  // P0.21(GPIO:In)    = PENIRQ ADS7846              
  LPC_PINCON->PINSEL3 &= ~(3UL<<10); 											// Reset P1.21 Mode = GPIO
  LPC_GPIO1->FIODIR   |=  (1UL<<21);											// P1.21 = ADS7846 CS(Output)
  LPC_GPIO1->FIOPIN   |=  (1UL<<21);    										// P1.21 = High 

  LPC_PINCON->PINSEL1 &= ~(3UL<<10); 											// Reset P0.21 Mode = GPIO
  LPC_GPIO0->FIODIR   &= ~(1UL<<21);											// P0.21 = PENIRQ(Input)

  //Config SSP0 Pin Connect
  LPC_PINCON->PINSEL3 |=  (3UL<<8);         										// Select P1.20 = SCK0(SSP0) 
  LPC_PINCON->PINSEL3 |=  (3UL<<14);        										// Select P1.23 = MISO0(SSP0) 
  LPC_PINCON->PINSEL3 |=  (3UL<<16);        										// Select P1.24 = MOSI0(SSP0) 
  	
  LPC_SC->PCONP       |=  (1 << 21);        										// Enable power to SSPI0 block  
  LPC_SC->PCLKSEL1    &= ~(3<<10);          										// PCLKSP0 = CCLK/4 (18MHz) 
  LPC_SC->PCLKSEL1    |=  (1<<10);          										// PCLKSP0 = CCLK   (72MHz) 
  LPC_SSP0->CPSR       =   72;              										// 72MHz / 72 = 1MHz(maximum of 2MHz is possible)

  LPC_SSP0->CR0        =  (   0 << 7) | 										// CPHA = 0                           
                          (   0 << 6) | 										// CPOL = 0                           
                          (   0 << 4) | 										// Frame format = SPI                 
                          ((8-1)<< 0) ; 										// Data size = 8 bits 
  LPC_SSP0->CR1        =  (   1 << 1);  										// Enable SSP-                       
  LPC_GPIO0->FIODIR   |=  (1UL<< 4);       										// Pin P0.5 = Output(GLCD Backlight)

  /* Config P0.6,P0.7,P0.8,P0.9 to SSP1 For Control GLCD */
  // P0.6(GPIO:Out)   = CS GLCD
  // P0.7(SCK1:SSP1)  = SCL	GLCD
  // P0.8(MISO1:SSP1) = SDO GLCD
  // P0.9(MOSI1:SSP1) = SDI	GLCD
  // P0.5(GPIO:Out)  = BL GLCD
  LPC_GPIO0->FIODIR   |=  (1UL<<5);       										// Pin P0.5 = Output(GLCD Backlight)
  GLCD_BL_ON();       													// Turn-OFF GLCD Backlight 

  LPC_PINCON->PINSEL0 &= ~(3UL<<12); 											// Reset P0.6 Mode = GPIO
  LPC_GPIO0->FIODIR   |=  (1 <<  6);    										// Pin P0.6 is GPIO output(CS GLCD)    
  LPC_GPIO0->FIOSET    =  (1 <<  6);    										// Set P0.6 = High 

  LPC_PINCON->PINSEL0 &= ~(3UL<<14); 											// Reset P0.7 Mode = GPIO
  LPC_PINCON->PINSEL0 |=  (2UL<<14); 											// Select P0.7 = SCK1(SSP1)
  LPC_PINCON->PINSEL0 &= ~(3UL<<16); 											// Reset P0.8 Mode = GPIO
  LPC_PINCON->PINSEL0 |=  (2UL<<16); 											// Select P0.8 = MISO1(SSP1)
  LPC_PINCON->PINSEL0 &= ~(3UL<<18); 										        // Reset P0.9 Mode = GPIO
  LPC_PINCON->PINSEL0 |=  (2UL<<18); 											// Select P0.9 = MOSI1(SSP1)

  LPC_SC->PCONP       |=  (1 << 10);    										// Enable power to SSP1 block       
  LPC_SC->PCLKSEL0    |=  (2 << 20);    										// SSP1 clock = CCLK/2 (36MHz)   
  LPC_SSP1->CPSR       =  2;            									       // Clock Rate = 18MHz         

  LPC_SSP1->CR0        =  (   1 << 7) | 													// CPHA = 1                           
                          (   1 << 6) | 													// CPOL = 1                           
                          (   0 << 4) | 													// Frame format = SPI                 
                          ((8-1)<< 0) ; 													// Data size = 8 bits                          
  LPC_SSP1->CR1        =  (   1 << 1);  													// Enable SSP1                       
}

/****************************/
/* GLCD SPI Sent Data 8 bit */
/****************************/
unsigned char GLCD_SPI_Read_Write(unsigned char DataByte)    
{
  LPC_SSP1->DR = DataByte;
  while (LPC_SSP1->SR & (1 << 4));      													// Wait for transfer to finish       
  return (LPC_SSP1->DR);                													// Return received value              
}

/******************************************************/
/* Write Address Command(Index Reg.)(Use Device ID=0) */
/******************************************************/
void GLCD_Write_Command(unsigned char GLCD_Command)
{
  GLCD_CS_LOW();                     														// Enable GLCD Interface

  GLCD_D_CX_LOW();
  GLCD_SPI_Read_Write(0x00);																// Sent Byte 2 = data 8 bit High Index Reg.: 0x00
  GLCD_SPI_Read_Write(GLCD_Command);														// Sent Byte 3 = data 8 bit Low index reg. : cmm
    
  GLCD_CS_HIGH();                           												// Disable GLCD Interface
}
 
/***************************************/
/* Write data to LCD (Use Device ID=0) */
/***************************************/
void GLCD_Write_Data(unsigned int GLCD_Data)
{	
  GLCD_CS_LOW();                     														// Enable GLCD Interface

  GLCD_D_CX_HIGH();
//  GLCD_SPI_Read_Write(0x72);                												// Byte 1 = [Device ID Code:01110[0]]+[RS:1] + [R/W:0]
  GLCD_SPI_Read_Write(GLCD_Data >> 8);      												// Byte 2 = Data 8 bit High 
  GLCD_SPI_Read_Write(GLCD_Data);           												// Byte 3 = Data 8 bit Low 
  
  GLCD_CS_HIGH();                           												// Disable GLCD Interface
}


/********************************************/
/* Function Check Status Press Touch Screen */
/* Return Value : 0 = Not Touched screen    */
/*                1 = Touched screen        */
/********************************************/
char Get_Status_Touch(void)
{
  uint32_t pen_val;  
  
  pen_val = PENIRQ_READ();													// Read PENIRQ Pin Logic
  if(pen_val == PENIRQ_MASK)												// Logic "1" = Not Press
  	return 0 ;																			// Not Press
  else
    return 1 ;																			// Press Touch screen
}

/***************************************************************************/
/*    Function SPI Write and Read Data 1 Byte from Touch Screen ADS7846    */
/***************************************************************************/
/* Parameter    : DataByte = data or command control ADS7846 (1Byte)       */ 
/* Return Value : Return value adc from touched times 1 byte Pass Function */         
/***************************************************************************/
unsigned char TCS_SPI_Write(unsigned char DataByte)			  	
{
  LPC_SSP0->DR = DataByte; 																	// Write and Read a byte on SPI interface.
  while (LPC_SSP0->SR & 0x10);          													// Wait BYS for transfer to finish 
  return (LPC_SSP0->DR);                													// Return received value 
}

/*****************************************************************/ 
/* Function Read X-Y-Position ADC Touch Screen-12 bit (ADS7846)  */
/* Parameter    : None                                           */
/* Return Value : dif_adc_X = Keep Result ADC X-Position(12 bit) */
/*                dif_adc_Y = Keep result ADC Y-Position(12 bit) */                      
/*****************************************************************/
void TCS_SPI_Read_Hor(void)
{
  unsigned long tcs_adc_X=0 ,tcs_adc_Y=0;
	//unsigned long dif_adc_X=0,dif_adc_Y=0;
  unsigned char buf_data[4];

  if(Get_Status_Touch())
  {
    TCS_CS_LOW();																			// Enable Touch Interface

  buf_data[0] = TCS_SPI_Write(0xD0);														// Write Command Measure X-Position 
	buf_data[0] = TCS_SPI_Write(0x00);														// Read ADC data X-Position (7-bit byte High) data: 0ddddddd	(bit) 
	buf_data[1] = TCS_SPI_Write(0x90);														// Write Command Measure Y-Position ; Read ADC data X-Position (5-bit byte Low)  data:ddddd000(bit)
	buf_data[2] = TCS_SPI_Write(0x00);														// Read ADC data Y-Position(7-bit byte High) data: 0ddddddd (bit)
	buf_data[3] = TCS_SPI_Write(0x00);														// Read ADC data Y-Position(5-bit byte Low)  data: ddddd000 (bit)

	tcs_adc_X  = buf_data[0];																// Mark ADC Data X-Position 12 bit
	tcs_adc_X  = tcs_adc_X << 5;
	tcs_adc_X |= buf_data[1] >> 3;
	tcs_adc_X  = tcs_adc_X & 0x00000FFF;
	 
	tcs_adc_Y  = buf_data[2];   															// Mark ADC Data Y-Position 12 bit
	tcs_adc_Y  = tcs_adc_Y << 5;   															// Shift 7 bit High
	tcs_adc_Y |= buf_data[3] >> 3;   														// Shift 5 bit low
	tcs_adc_Y  = tcs_adc_Y & 0x00000FFF;													// total ADC data 12 bit
	
	TCS_CS_HIGH();																			// Disable Touch Interface			

    //Result	 	 
  dif_adc_X = 4095-tcs_adc_Y;  															// ADC 12 bit :LCD Start Landscape
	dif_adc_Y = tcs_adc_X;	
  }
}



/*******************************************************************/
/*          Function Get Position Address real of Display          */
/*******************************************************************/
/* Parameter    : num = Number times for Measure adc from Touched  */ 
/* Return Value : X = Keep value address X-Position		           */
/*                Y = Keep value address Y-Position                */ 
/*******************************************************************/
void TCS_Get_Point(char num)
{
  char nm; 
	
  nm=0;
  while(Get_Status_Touch() && nm<num)	  													// Measure touch x,y 10 times if the Touch kept pressed,
  {
    TCS_SPI_Read_Hor();                     												// Read value ADC Touch X-Y 
	avg_x+=	dif_adc_X; 																// keep value ADC Touch-X
	avg_y+=	dif_adc_Y; 																// keep value ADC Touch-Y

 	nm++; 
  }

  if(nm==num)	                      														// if the touch xy successfuly collected,
  {	
   avg_x=avg_x/5;
	 avg_y=avg_y/5;
   X=avg_x;
   Y=avg_y;	
   avg_x=0;
   avg_y=0;		
  }
}

/******************************************************/
/*   Function Set  Background color or Clear Screen   */
/******************************************************/
/* Parameter : bg_color =  BackGround color of Screen */
/******************************************************/
void bg_color(long bg_color)
{	
  unsigned long cnt;
GLCD_Write_Command(0x2A);	 // 0x2A
GLCD_Write_Data(0);
GLCD_Write_Command(0x2B);	 // 0x2B
GLCD_Write_Data(0);

//
// Clear the contents of the display buffer.
// 
GLCD_Write_Command(0x2C);	 // 0x2C
for(cnt = 0; cnt < (320 * 240); cnt++)
{
	GLCD_Write_Data(bg_color);
  //GLCD_Write_Data(0xFF);
} 
}

/****************************************************************************/
/**                Function Print Text 1 Charecter size 7x11                */
/****************************************************************************/
/* Parameter : row      = Ascii Code (Position buffer keep text)		    */
/*             adx,ady  = Position X,Y for begin plot text by will 			*/
/*                        begin plot from bottom left to top left   		*/
/*             fg_clr   = Color of font text										*/
/*             bg_clr   = Color of background (if bg_clr = no_bg or 1=	*/
/*                        non color background)								*/
/****************************************************************************/
void text_7x11_hor(char row,long adx,long ady,long fg_clr,long bg_clr)
{
  long ax,ay;
  char m,n,tx;
     
  ax = adx;
  ay = ady; 

  row = row-0x20;

  // Print Text 1 Charecter(data 14 Byte) 
  for(m=0;m<14;m++)
  {
    // Sent data byte1=8 bit
    tx = ascii_7x11[row][m];  																//Read data Ascii
	
	for(n=0;n<8;n++)		       															//Loop Sent data  1 byte(8bit)
	{
	  if(tx & 0x80)				   															//if data bit7 = 1 ,Plot Color area Charecter
	  {              
	    GLCD_Write_Command(0x2A);  															//Command Set Adddress Hor(X)
        GLCD_Write_Data(ay);  																//Sent X_Address CGRAM	 		
        GLCD_Write_Command(0x2B);  															//Command Set Address Ver(Y)
        GLCD_Write_Data(ax);  																//Sent Y_Address CGRAM
		GLCD_Write_Command(0x2C);  															//Command Write data 
        GLCD_Write_Data(fg_clr);
	   }
	   else						   															//if data bit7 = 0 ,Plot Color area back ground Charecter
	   {
	     if(bg_clr != 1)     
         {
		    GLCD_Write_Command(0x2A);  														//Command Set Adddress Hor(X)
            GLCD_Write_Data(ay);  															//Sent X_Address CGRAM	
            GLCD_Write_Command(0x2B);  														//Command Set Adddress Ver(Y)
            GLCD_Write_Data(ax);  															//Sent Y_Address CGRAM
			GLCD_Write_Command(0x2C);  														//Command Write data
            GLCD_Write_Data(bg_clr);  														//Sent Data
		  }
		}

		tx <<= 1;  																			// Shift Right data 1 bit
		ay   = ay+1;  																		// Increment Y-address
	} 
	m++;  																					//Increment Next pointter byte Data 


	// Sent data byte2=3 bit 
	tx = ascii_7x11[row][m];  																//Read data byte2	
	for(n=0;n<3;n++)			   															//Loop sent data byte2 = 3 bit
	{						     
	  if(tx & 0x80)				   															//if data bit7 = 1 ,Plot Color area Charecter
	  {              
	    GLCD_Write_Command(0x2A);  															//Command Set Adddress Hor(X)
        GLCD_Write_Data(ay);  																//Sent X_Address CGRAM		
        GLCD_Write_Command(0x2B);  															//Command Set Adddress Ver(Y)
        GLCD_Write_Data(ax);  																//Sent Y_Address CGRAM
		GLCD_Write_Command(0x2C);  															//Command Write data
        GLCD_Write_Data(fg_clr);
      }
	  else						   															//if data bit7 = 0 ,Plot Color area back ground Charecter
	  {
	    if(bg_clr != 1)     
        {
		  GLCD_Write_Command(0x2A);  														//Command Set Adddress Hor(X)
          GLCD_Write_Data(ay);  															//Sent X_Address CGRAM	 		
          GLCD_Write_Command(0x2B);  														//Command Set Adddress Ver(Y)
          GLCD_Write_Data(ax);  															//Sent Y_Address CGRAM
		  GLCD_Write_Command(0x2C);  														//Command Write data
          GLCD_Write_Data(bg_clr);
		}
	  }

	  tx <<= 1;  																			//Shift Right data 1 bit
	  ay = ay+1;  																			//Increment Y-address
	} 

	ax = ax+1; 																				//Complete sent data 2 byte(11bit) Increment X-Address
	ay = ady; 																				//Set Position Y-address old
  }	

  // Fill Back ground Color Position space between Charecter 1 Colum 
  if(bg_clr != 1)     
  {
    for(n=0;n<11;n++)
	{
	  GLCD_Write_Command(0x2A);  															//Command Set Adddress Hor(X)
      GLCD_Write_Data(ay);  																//Sent X_Address CGRAM	 		
      GLCD_Write_Command(0x2B);  															//Command Set Adddress Ver(Y)
      GLCD_Write_Data(ax);  																//Sent Y_Address CGRAM
	  GLCD_Write_Command(0x2C);  															//Command Write data
      GLCD_Write_Data(bg_clr);
	  ay = ay+1;  																			//Increment Y-Address
    }
  }
}

/**********************************************************************/
/*                         Function Print String                      */
/**********************************************************************/
/* Parameter : *str = Charecter ASCII (String)					      */
/*             cur_x,cur_y = Position X,Y for begin plot text by will */
/*                           begin plot from bottom left to top left  */
/*             fg_color = color of Text								  */
/*             bg_color = color Background of text                    */											
/**********************************************************************/
void lcd_printStr_hor(char *str,long cur_x,long cur_y,long fg_color,long bg_color)
{
  unsigned char i;

  for (i=0; str[i] != '\0'; i++)
  {					 
    text_7x11_hor(str[i],cur_x,cur_y,fg_color,bg_color);
	cur_x += 8; 	
  }
}
 
/***********************************/
/* Function Print Text 3 Charecter */
/***********************************/
void lcd_print3Cha_hor(char ch1,char ch2,char ch3,char ch4,long cur_x,long cur_y,long fg_color,long bg_color)
{	  			 
  text_7x11_hor(ch1,cur_x,cur_y,fg_color,bg_color);
  cur_x += 8; 
  text_7x11_hor(ch2,cur_x,cur_y,fg_color,bg_color);
  cur_x += 8; 
  text_7x11_hor(ch3,cur_x,cur_y,fg_color,bg_color);
  cur_x += 8;  	
	text_7x11_hor(ch4,cur_x,cur_y,fg_color,bg_color);
  cur_x += 8;
}

void ticket_print(void)
  {
		//BACKWARD
	if( ((0X578<X)&&(X<0X898))&&((0XBB8<Y)&&(Y<0XDAC)))               
	  {
			//lcd_printStr_hor("FORWARD",30,10,BROWN ,WHITE);   // Plot Text
		//	LPC_GPIO0->FIOCLRH=0X01E0;	
		  LPC_GPIO0->FIOSETH=0X0140;	
		 }
		//FORAWARD
	  else if( ((0X578<X)&&(X<0X898))&&((1000<Y)&&(Y<1700)))               
	  {
			//lcd_printStr_hor("FORWARD",30,10,BROWN ,WHITE);   // Plot Text
		//	LPC_GPIO0->FIOCLRH=0X01E0;	
		  LPC_GPIO0->FIOSETH=0X00A0;	
		 }
		//LEFT
		 else if( ((650<X)&&(X<1000))&&((2200<Y)&&(Y<2700)))               
	  {
			//lcd_printStr_hor("FORWARD",30,10,BROWN ,WHITE);   // Plot Text
		//	LPC_GPIO0->FIOCLRH=0X01E0;
      LPC_GPIO0->FIOSETH=0X0100;			
		 
		 }
		//STOP
		 else if( ((1500<X)&&(X<2000))&&((2200<Y)&&(Y<2700)))               
	  {
			//lcd_printStr_hor("FORWARD",30,10,BROWN ,WHITE);   // Plot Text
			LPC_GPIO0->FIOCLRH=0X01E0;	
		}
		//RIGHT
		 else if( ((2900<X)&&(X<3200))&&((2200<Y)&&(Y<2700)))               
	  {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
			//lcd_printStr_hor("FORWARD",30,10,BROWN ,WHITE);   // Plot Text
		//	LPC_GPIO0->FIOCLRH=0X01E0;	
		 	 LPC_GPIO0->FIOSETH=0X0040;	
		 }
		
		
	// lcd_printStr_hor("FORWARD",30,10,BROWN ,WHITE);   // Plot Text
		delay(0x20000);
	//	disply_clear(44,20);	
	//	disply_clear(80,30);	
	 
		return;
	 }

	
	
	 
	 
	 void disply_clear(unsigned short int clst,unsigned short int rost)
	 {
		 unsigned long int row=0,col=0;
		 for(row=rost;row<320;row++)
		 {
			for(col=clst;col<clst+11;col++)
			 {
	    GLCD_Write_Command(0x2A);  															//Command Set Adddress Hor(X)
      GLCD_Write_Data(col);  																//Sent X_Address CGRAM	 		
      GLCD_Write_Command(0x2B);  															//Command Set Address Ver(Y)
      GLCD_Write_Data(row);  																//Sent Y_Address CGRAM
		  GLCD_Write_Command(0x2C);  															//Command Write data 
      GLCD_Write_Data(0x00);
			 }
		 }
	  }
	
	 void delay(unsigned long int dk)
	 {
		unsigned long int di=0; 
		for(di=0;di<dk;di++) ;
		 
	 }