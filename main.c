/*
 * File:   main.c
 * Author: muth.inc
 * Created on 14 juillet 2019, 14:06
 */

// PIC12LF1840 Configuration Bit Settings
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#include <xc.h>
#include <stdint.h>
#include <math.h>
#include <pic12lf1840.h>

// CPU freq
#define _XTAL_FREQ  (32000000UL)

#define LED_data PORTAbits.RA5
#define TEST_pin PORTAbits.RA4

// off timeout in ms (1800000UL = 30min)
#define OFF_TIMEOUT 30000UL
// settle timeout in ms (3000UL = 3sec)
#define SETTLE_TIMEOUT 3000UL

uint16_t voltage = 0;
uint32_t mili_count = 0;

void interrupt isr(void) {
    if (INTCONbits.TMR0IF) {
        TMR0 = 131;  
        INTCONbits.TMR0IF = 0;
        mili_count++;
        TEST_pin = !TEST_pin;
    }
}

void init(){
    OSCCONbits.IRCF = 0b1110;   // 8/32MHz clock
    WDTCONbits.SWDTEN = 0;      // watch dog disable

    // pin config
    ANSELA = 0b00000100;    // analog on port AN2
    PORTA = 0x00;
    TRISA = 0b00000100;     // input/output
    LED_data = 1;
    
    //conf ADC
    ADCON1bits.ADFM = 1;        // right justify result
    ADCON0bits.CHS = 0b0010;    // AN2 is ADC input
    ADCON1bits.ADPREF = 0b00;   // Positive ref is Vdd (default)
    ADCON1bits.ADCS = 0b110;    // 2us conversion
    ADCON0bits.ADON = 1;        // Turn on ADC module
    
    //conf timer0
    OPTION_REGbits.T0CS = 0;    // internal clock source
    OPTION_REGbits.PSA = 0;     // prescaler on timer0
    OPTION_REGbits.PS = 0b101;  // prescaler 1:64
    TMR0 = 131;                 // count 125 (8e6 / (64*125)) = 1000 (1ms)
    
    // conf interrupts
    INTCONbits.TMR0IE = 1;      // enable timer0 int
    INTCONbits.PEIE = 1;        // enable perif int
    INTCONbits.T0IE = 1;        // enable timer0 int
    INTCONbits.GIE = 1;         // enable interrupts
}

void getVoltage() {
    voltage = 0;

    // get  voltage
    ADCON0bits.GO_nDONE = 1;    // Start a conversion
    while (ADCON0bits.GO_nDONE) {} ;// Wait for it to be completed

    voltage = (ADRESH<<8) + ADRESL;

}

// reverse the bits in a char
uint32_t bitflip(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return (uint32_t)b;
}

void sendSK6812rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    char i;
    
    r = bitflip(r);
    g = bitflip(g);
    b = bitflip(b);
    w = bitflip(w);

    INTCONbits.GIE = 0;
    
    for (i=0; i<8; i++){
        if (g & 1 == 1) {
            LED_data = 1;
            NOP();
            LED_data = 0;
        } else {
            LED_data = 1;
            LED_data = 0;
        }
        g = g >> 1;
    }
    
    for (i=0; i<8; i++){
        if (r & 1 == 1) {
            LED_data = 1;
            NOP();
            LED_data = 0;
        } else {
            LED_data = 1;
            LED_data = 0;
        }
        r = r >> 1;
    }
    
    for (i=0; i<8; i++){
        if (b & 1 == 1) {
            LED_data = 1;
            NOP();
            LED_data = 0;
        } else {
            LED_data = 1;
            LED_data = 0;
        }
        b = b >> 1;
    }
    
    for (i=0; i<8; i++){
        if (w & 1 == 1) {
            LED_data = 1;
            NOP();
            LED_data = 0;
        } else {
            LED_data = 1;
            LED_data = 0;
        }
        w = w >> 1;
    }
    
    INTCONbits.GIE = 1;
}

void resetSK6812() {
    LED_data = 0;
    __delay_us(80);
}

void main(void) {
    init();
    uint16_t bright = 0;
    uint16_t last_voltage = 0;
    uint8_t loop_count = 0;
    int32_t diff = 0;
    
    while(1) {
        // ~ 50hz loop
        __delay_ms(20);
        
        // update analog knob value
        getVoltage();
        
        // reset timeout only if significant change seen on the knob
        diff = voltage - last_voltage;
        if ( diff > 8 || diff <= -8) {
            mili_count = 0;
            last_voltage = voltage;
        }
        
        // decrease bright slowly after timeout (30 min)
        // Or change the brightness, but settle to avoid ADC noise 
        if ( (mili_count) > OFF_TIMEOUT ) {
            loop_count++;
            if (bright > 0 && loop_count > 100) {
                bright--;
                loop_count = 0;
            }
        } else if (mili_count < SETTLE_TIMEOUT) {
            bright = voltage;
        }
        
        // brightness curve
        if (bright < 256) {
            resetSK6812();
            sendSK6812rgbw(bright, 0, 0, 0);
            
        } else if (bright >= 256 && bright < 512) {
            resetSK6812();
            sendSK6812rgbw(0xFF, 0, 0, (bright - 256));
            
        } else if (bright >= 512 && bright < 768) {
            resetSK6812();
            sendSK6812rgbw(0xFF, (bright - 512), (bright - 512)/2, 0xFF);
            
        } else if (bright >= 768 && bright < 1024) {
            resetSK6812();
            sendSK6812rgbw(0xFF - (bright - 768), 0xFF, 0x7F, 0xFF - (bright - 768));
            
        } 
        
    }
    
    return;
}
