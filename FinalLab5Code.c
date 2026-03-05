
#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>

// ~C51~  

#define SYSCLK 72000000L
#define BAUDRATE 115200L
#define SARCLK 18000000L

#define LCD_RS P1_7
#define LCD_E  P2_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0
#define CHARS_PER_LINE 16


#define SERVO_MIN (65536 - 6000)   // 1.0ms
#define SERVO_MID (65536 - 9000)   // 1.5ms
#define SERVO_MAX (65536 - 12000)  // 2.0ms

#define LED1 P3_7
#define LED2 P3_3
#define LED3 P3_2
#define LED4 P3_1
#define SPEAKER P0_2


unsigned char overflow_count;


float Volts_at_Pin(unsigned char pin);
unsigned int ADC_at_Pin(unsigned char pin);
double Timer0quarterT(float quarterperiod);
double Timer0quarterT2(float quarterperiod);
void waitms (unsigned int ms);
void InitADC (void);
void InitPinADC (unsigned char portno, unsigned char pinno);
void LCDprint(char * string, unsigned char line, bit clear);
void LCD_4BIT (void);
void TIMER0_Init(void);
void Timer3us(unsigned char us);
void Init_Servo_PCA(void);
void SetServoByPhase(float phase_angle);

char _c51_external_startup (void)
{
    // Disable Watchdog with key sequence
    SFRPAGE = 0x00;
    WDTCN = 0xDE; //First key
    WDTCN = 0xAD; //Second key
  
    VDM0CN=0x80;       // enable VDD monitor
    RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

    #if (SYSCLK == 48000000L)   
        SFRPAGE = 0x10;
        PFE0CN  = 0x10; // SYSCLK < 50 MHz.
        SFRPAGE = 0x00;
    #elif (SYSCLK == 72000000L)
        SFRPAGE = 0x10;
        PFE0CN  = 0x20; // SYSCLK < 75 MHz.
        SFRPAGE = 0x00;
    #endif
    
    #if (SYSCLK == 12250000L)
        CLKSEL = 0x10;
        CLKSEL = 0x10;
        while ((CLKSEL & 0x80) == 0);
    #elif (SYSCLK == 24500000L)
        CLKSEL = 0x00;
        CLKSEL = 0x00;
        while ((CLKSEL & 0x80) == 0);
    #elif (SYSCLK == 48000000L) 
        // Before setting clock to 48 MHz, must transition to 24.5 MHz first
        CLKSEL = 0x00;
        CLKSEL = 0x00;
        while ((CLKSEL & 0x80) == 0);
        CLKSEL = 0x07;
        CLKSEL = 0x07;
        while ((CLKSEL & 0x80) == 0);
    #elif (SYSCLK == 72000000L)
        // Before setting clock to 72 MHz, must transition to 24.5 MHz first
        CLKSEL = 0x00;
        CLKSEL = 0x00;
        while ((CLKSEL & 0x80) == 0);
        CLKSEL = 0x03;
        CLKSEL = 0x03;
        while ((CLKSEL & 0x80) == 0);
    #else
        #error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
    #endif
    
    P0MDOUT |= 0x11; 
    XBR0     = 0x01;                       
    XBR1     = 0X01; 
    XBR2     = 0x40; 


    #if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
        #error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
    #endif
    SCON0 = 0x10;
    TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
    TL1 = TH1;     
    TMOD &= ~0xf0;  
    TMOD |=  0x20;                        
    TR1 = 1; 
    TI = 1;  
    
    return 0;
}

void Init_Servo_PCA(void) {
    SFRPAGE = 0x00;
    
    
    PCA0MD  &= ~0x0E; 
    
    
    PCA0CPM0 = 0xC2; 

  
    PCA0CPL0 = (SERVO_MID & 0xFF);
    PCA0CPH0 = (SERVO_MID >> 8);

    PCA0CN0 |= 0x40;
}

void SetServoByPhase(float phase_angle) {
    float temp_pulse;
    unsigned int final_counts;
    
    float gain = 1.0; 
    phase_angle = phase_angle * gain;
    

    if (phase_angle > 180.0)  phase_angle = 180.0;
    if (phase_angle < -180.0) phase_angle = -180.0;

    
    temp_pulse = 9000.0 + (phase_angle * (6000.0 / 180.0));

   
    final_counts = 65536 - (unsigned int)temp_pulse;

    SFRPAGE = 0x00;
    PCA0CPL0 = (final_counts & 0xFF);
    PCA0CPH0 = (final_counts >> 8);
}
void TIMER0_Init(void)
{
    TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
    TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
    TR0=0; // Stop Timer/Counter 0
}

void InitADC (void)
{
    SFRPAGE = 0x00;
    ADEN=0; // Disable ADC
    
    ADC0CN1=
        (0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.      
        (0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
    
    ADC0CF0=
        ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
        (0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
    
    ADC0CF1=
        (0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
        (0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
    
    ADC0CN0 =
        (0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
        (0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
        (0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
        (0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
        (0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
        (0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
        (0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

    ADC0CF2= 
        (0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
        (0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
        (0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
    
    ADC0CN2 =
        (0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
        (0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

    ADEN=1; // Enable ADC
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
    unsigned char i;               // usec counter
    
    // The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
    CKCON0|=0b_0100_0000;
    
    TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
    TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
    
    TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
    for (i = 0; i < us; i++)       // Count <us> overflows
    {
        while (!(TMR3CN0 & 0x80));  // Wait for overflow
        TMR3CN0 &= ~(0x80);         // Clear overflow indicator
    }
    TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
    unsigned int j;
    unsigned char k;
    for(j=0; j<ms; j++)
        for (k=0; k<4; k++) Timer3us(250);
}

#define VDD 3.307 // The measured value of VDD in volts

void InitPinADC (unsigned char portno, unsigned char pinno)
{
    unsigned char mask;
    
    mask=1<<pinno;

    SFRPAGE = 0x20;
    switch (portno)
    {
        case 0:
            P0MDIN &= (~mask); 
            P0SKIP |= mask; 
        break;
        case 1:
            P1MDIN &= (~mask);
            P1SKIP |= mask;
        break;
        case 2:
            P2MDIN &= (~mask);
            P2SKIP |= mask; 
        break;
        default:
        break;
    }
    SFRPAGE = 0x00;
}

double Timer0quarterT(float quarterperiod) {
    double max_val = 0.0;
    double ADC_val;      
    int reload_val;
    
    reload_val = (65536 - (quarterperiod));
    
    TL0=0; 
    TH0=0;
    TF0=0; 
    
    TH0 = (reload_val >> 8);
    TL0 = (reload_val & 0xFF);
    
    while(P3_0==0); 
    while(P3_0==1); 
    while(P3_0==0); 
    
    TR0 = 1; 
    
    while (!TF0) { 
        ADC_val = Volts_at_Pin(QFP32_MUX_P2_2);
        if (ADC_val > max_val) max_val = ADC_val;
    }
    
    TF0 = 0; 
    TR0 = 0; // Stop timer
    return max_val; 
}

double Timer0quarterT2(float quarterperiod) {
    double max_val = 0.0;
    double ADC_val;      
    int reload_val;
    
    reload_val = (65536 - (quarterperiod));
    
    TL0=0; 
    TH0=0;
    TF0=0; 
    
    TH0 = (reload_val >> 8);
    TL0 = (reload_val & 0xFF);
    
    while(P3_0==0); 
    while(P3_0==1); 
    while(P3_0==0); 
    
    TR0 = 1; 
    
    while (!TF0) { 
        ADC_val = Volts_at_Pin(QFP32_MUX_P2_3);
        if (ADC_val > max_val) max_val = ADC_val;
    }
    
    TF0 = 0; 
    TR0 = 0; // Stop timer
    return max_val; 
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    ADC0MX = pin;   // Select input from pin
    ADINT = 0;
    ADBUSY = 1;     // Convert voltage at the pin
    while (!ADINT); // Wait for conversion to complete
    return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
     return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

void LCD_pulse (void)
{
    LCD_E=1;
    Timer3us(40);
    LCD_E=0;
}

void LCD_byte (unsigned char x)
{
    ACC=x; //Send high nible
    LCD_D7=ACC_7;
    LCD_D6=ACC_6;
    LCD_D5=ACC_5;
    LCD_D4=ACC_4;
    LCD_pulse();
    Timer3us(40);
    ACC=x; //Send low nible
    LCD_D7=ACC_3;
    LCD_D6=ACC_2;
    LCD_D5=ACC_1;
    LCD_D4=ACC_0;
    LCD_pulse();
}

void WriteData (unsigned char x)
{
    LCD_RS=1;
    LCD_byte(x);
    waitms(2);
}

void WriteCommand (unsigned char x)
{
    LCD_RS=0;
    LCD_byte(x);
    waitms(5);
}

void LCD_4BIT (void)
{
    LCD_E=0; 
    waitms(20);
    WriteCommand(0x33);
    WriteCommand(0x33);
    WriteCommand(0x32); // Change to 4-bit mode
    WriteCommand(0x28);
    WriteCommand(0x0c);
    WriteCommand(0x01); // Clear screen 
    waitms(20); 
}

void LCDprint(char * string, unsigned char line, bit clear)
{
    int j;
    WriteCommand(line==2?0xc0:0x80);
    waitms(5);
    for(j=0; string[j]!=0; j++) WriteData(string[j]);
    if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); 
}

// ---------------- MAIN FUNCTION ----------------
void main (void)
{   
    xdata char buff1[17];
    xdata char buff2[15];
    float max_pin_val;
    float max_pin_val2;
    float period;
    double d_time;
    double phase;

	SPEAKER = 0;
		
    waitms(500); 
    printf("\x1b[2J"); 
    
    InitPinADC(2, 2); 
    InitPinADC(2, 3); 
    InitPinADC(2, 4); 
    InitPinADC(2, 5); 
    InitADC();
    LCD_4BIT();
    TIMER0_Init();
    Init_Servo_PCA(); 

    while(1)
    {
        TL0=0; 
        TH0=0;
        TF0=0;
        overflow_count=0;
        
        while(P3_0!=0); 
        while(P3_0!=1); 
        
        TR0=1; 
        
        while(P3_0!=0) 
        {
            if(TF0==1) { TF0=0; overflow_count++; }
        }
        
        while(P3_0!=1) 
        {
            if(TF0==1) { TF0=0; overflow_count++; }
        }
        
        TR0=0; 
        
        if(TF0==1) { TF0=0; overflow_count++; }
        
        period=(overflow_count*65536.0+TH0*256.0+TL0)*(12.0/SYSCLK);  
        
        TL0=0; 
        TH0=0;
        TF0=0;
        overflow_count=0;

        while(P3_0==0); 
        while(P3_0==1); 
        
        TR0=1; 
        
        while(P2_6==0)
        {
            if(TF0==1) { TF0=0; overflow_count++; }
        }
        
        while(P2_6==1)
        {
            if(TF0==1) { TF0=0; overflow_count++; }
        }
        
        TR0=0; 
        
        d_time=(overflow_count*65536.0+TH0*256.0+TL0)*(12.0/SYSCLK);
        
        phase = d_time * (360.0/period);
        
        if (phase > 182) {
            phase = phase - 360.0;
        }

        phase = phase*-1; 
        
        if(phase>-1 && phase<0.3){
			phase=0;
		}
		
		if (phase<0.1 && phase>-0.1){
			SPEAKER = 1;
		}
		else{
			SPEAKER=0;
		}
		
		if(0<=phase && 90>=phase) {
			LED1=1;
			LED2=0;
			LED3=0;
			LED4=0;
		}
		 if(phase>90&& phase<180){
			LED1=0;
			LED2=1;
			LED3=0;
			LED4=0;
		}
		
		if(phase<0&& phase>-90){
			LED1=0;
			LED2=0;
			LED3=1;
			LED4=0;
		}
		
		 if(phase<-90&& phase>-180){
			LED1=0;
			LED2=0;
			LED3=0;
			LED4=1;
		}

      
        SetServoByPhase(phase);

        max_pin_val = Timer0quarterT(period/2.0);
        max_pin_val2 = Timer0quarterT2(period/2.0);
        
        max_pin_val = max_pin_val/1.4142135624;
        max_pin_val2 = max_pin_val2/1.4142135624; 
        
       
        
        printf ("%f,", max_pin_val);
        printf ("%f,", max_pin_val2);
        printf ("%f,", period);
        printf ("%.4f\n", phase);
        
        printf ("\rMax Volts at P2.2: %fV\n", max_pin_val);
        printf ("\rMax Volts at P2.3: %fV\n", max_pin_val2);
        printf ("Phase: %.1f deg\r\n", phase);
        
        if(phase>0){
        sprintf(buff1, "Phase: %.1f LA", phase);
        LCDprint(buff1, 1, 1); 
        
        }  
        
       if(phase<0){
        sprintf(buff1, "Phase: %.1f LE", phase);
        LCDprint(buff1, 1, 1); 
       }  
        
          if(phase==0){
        sprintf(buff1, "Phase: %.1f", phase);
        LCDprint(buff1, 1, 1); 
        }  
        sprintf(buff2, "V1: %.2f V2:%.2f", max_pin_val, max_pin_val2);
        LCDprint(buff2, 2, 1);
            
        waitms(1000);
    }
}