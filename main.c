/* 
 * File:   main.c
 * Author: Sergio Boch 20887
 *
 * Created on May 16, 2022, 5:15 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 1000000

uint8_t POTEN = 0, CONTADOR = 0, SLEEP_FLAG = 0, POT_DATA = 1, ADDRESS = 0;

void setup(void);
uint8_t read_EEPROM(uint8_t ADDRESS);
void write_EEPROM(uint8_t ADDRESS, uint8_t data);

void __interrupt() isr (void){
    if(PIR1bits.ADIF){              
        POTEN = ADRESH;              
        PORTD = POTEN;             
        PIR1bits.ADIF = 0;          
    }
    else if(INTCONbits.RBIF){          
        if(!PORTBbits.RB0){                     
            if(SLEEP_FLAG==1){ 
                SLEEP_FLAG = 0;
            }
            else{            
                write_EEPROM(ADDRESS, POTEN); 
                SLEEP_FLAG = 1;
            }
        }
        INTCONbits.RBIF = 0;       
    }
    return;
}

void main(void) {
    setup();
    while(1){ 
        if(ADCON0bits.GO == 0){  
            ADCON0bits.GO = 1;       
        }
        if(SLEEP_FLAG == 1){              
            PIE1bits.ADIE = 0;
            SLEEP();                
        }
        else if(SLEEP_FLAG == 0){            
            PIE1bits.ADIE = 1;
        }
        PORTC = read_EEPROM(ADDRESS); 
        __delay_ms(500);
        if (PORTAbits.RA5 == 0)
            PORTAbits.RA5 = 1;
        else
            PORTAbits.RA5 = 0;
    }
    return;
}

void setup(void){
    ANSEL = 0b00000001;
    ANSELH = 0;
    
    TRISB = 0b00000001;;
    PORTB = 0;
    
    TRISA = 0b00000001;
    TRISC = 0;
    TRISD = 0;
    
    PORTC = 0;
    PORTD = 0;
    PORTA = 0;
   
    ADCON0bits.ADCS = 0b00;     // Fosc/2
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0;    // Seleccionamos el AN1
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time

    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0x01;
    INTCONbits.RBIE = 1;   
    IOCB = 0x01;         
    INTCONbits.RBIF = 0;
}

uint8_t read_EEPROM(uint8_t ADDRESS){
    EEADR = ADDRESS;
    EECON1bits.EEPGD = 0;  
    EECON1bits.RD = 1;      
    return EEDAT;              
}

void write_EEPROM(uint8_t ADDRESS, uint8_t data){
    EEADR = ADDRESS;
    EEDAT = data;
    EECON1bits.EEPGD = 0; 
    EECON1bits.WREN = 1;  
    
    INTCONbits.GIE = 0;    
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;   
    EECON1bits.WREN = 0;  
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1; 
}
