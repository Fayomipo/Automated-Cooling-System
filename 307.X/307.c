#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#define _XTAL_FREQ 20000000    //Corresponding to the crystal oscillator

#define LCD_MODE 0
#include "lcd.h"               // including the LCD Library/ header file

int temperature, temperature1, last_temp = 0;       // The variables are integers with initial value of zero

void update_temp(void)         // A function for the calculation of A/D Converter
{
    ADCON0bits.CHS = 0;
    __delay_ms(2);
    ADCON0bits.GO_DONE = 1;
    while(ADCON0bits.GO_DONE)
    temperature = ((ADRESH*256) + ADRESL);
    temperature1 = temperature/2.046;
    if (last_temp != temperature1) {
        lcd_backspace(5);
        lcd_write_int(temperature1);
        last_temp = temperature1;
    }
    
    __delay_ms (50);
}

void pwm_init(void)         // Varying the output voltage with PWM
{
    PR2=255;
    CCPR1L = CCP1CONbits.CCP1X =  CCP1CONbits.CCP1Y = 0;
    TRISCbits.TRISC2 = 0;
    T2CONbits.T2CKPS=0;
    CCP1CONbits.CCP1M = 0b1100;
}

#define pwm_start() TMR2 = 0; T2CONbits.TMR2ON=1
#define pwm_stop() T2CONbits.TMR2ON=0
#define pwm_set_dutycycle(percent) CCPR1L= (PR2+1)* percent/100

void main (void)
{
    OPTION_REGbits.nRBPU = 1;
    TRISA = 0b00000001;
    TRISB =  TRISC = TRISD = TRISE = 0;
    PORTA = PORTB = PORTC = PORTD = PORTE=0;
    ADCON0 = 0b11000001;
    ADCON1 = 0b10001110;
    
    T2CON=0;
    pwm_init();
    pwm_start();
    
    lcd_init(_2line, _5x8);
    lcd_set_cursor(FIRST_ROW, 0);
    lcd_write_str("Temperature is: ");
    lcd_set_cursor(SECOND_ROW, 4);
    lcd_write_str("°C ");
    lcd_set_cursor(SECOND_ROW, 0);
     
    PORTDbits.RD3 = 1;  // To show that the MC gives an output
    
    /*There are 3 main stages of operation over which the speed of the DC Fan 
    is varied and a fourth when the temperature is too high(extreme) and an indicator starts blinking.*/ 
    while(1)
    {
        update_temp();
        
        if(temperature1 >37)       // The maximum temperature that affects the speed of the Fan
        {
            RD0 = 1;
            pwm_set_dutycycle(99);  //Gives the maximum speed of the fan
        }
        else if (temperature1 >= 32)
        {
            RD0 = 1;
            pwm_set_dutycycle(66);
        }
        else if (temperature1 >= 27) // The temperature at which the Fan comes on i.e the minimum temperature that affects the Fan
        {
            RD0 = 1;
            pwm_set_dutycycle(33);  //Gives the lowest speed of the Fan
        }
        else
        {
            RD0 = 0;
            pwm_set_dutycycle(0);
        }
        
        if(temperature1 >= 55)      // Minimum temperature considered to be extreme, at which the indicator starts blinking.
        {
            RD1 = 1;
            for (int i=10; i--;) update_temp();
//            __delay_ms(500);
            RD1 = 0;
            for (int i=10; i--;) update_temp();
//            __delay_ms(500);
        }
    
    }
    return;
}