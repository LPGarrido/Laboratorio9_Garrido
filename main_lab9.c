/* 
 * File:   PWM.c
 * Author: Christopher Chiroy
 *
 * CCP en modo PWM, con señal de salida en pin CCP1, periodo de 2ms y ciclo 
 * de trabajo variable usando potenciometro
 * 
 * Created on 17 april 2022, 20:12
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 500000
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 16               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 75             // Valor máximo de ancho de pulso de señal PWM


/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal
//unsigned short anterior_s1 = 0;
//unsigned short anterior_s2 = 0;
uint8_t conteo=0;               // Conteo para PWM manual
uint8_t contador_ADC=0;
unsigned short num_ADC = 0;
unsigned short PWM3 = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
void RESET_TMR0(uint8_t TMR_VAR);   // resetear tmr0
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){                      // Fue interrupción del ADC?
        if(contador_ADC == 5){
            contador_ADC = 0;
            if(ADCON0bits.CHS == 0){            // Verificamos sea AN0 el canal seleccionado
                CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                ADCON0bits.CHS = 0b0001;
            }
            else if(ADCON0bits.CHS == 1){            // Verificamos sea AN0 el canal seleccionado
                CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR>>1 & 0b1;
                ADCON0bits.CHS = 0b0010;
            }
            else if(ADCON0bits.CHS == 2){            // Verificamos sea AN0 el canal seleccionado
                num_ADC = map(ADRESH, IN_MIN, IN_MAX, 0, 35);
                ADCON0bits.CHS = 0b0000;   
            }    
        }
        PIR1bits.ADIF = 0;                  // Limpiamos bandera de interrupción
    }
    
     
    if (INTCONbits.T0IF){
        conteo++;
        contador_ADC++;
        if(conteo == 36){
            conteo = 0;
            PWM3 = num_ADC;
        }
        RESET_TMR0(250);            // Reseteo cada 0.2 ms
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion 
            /*if(ADCON0bits.CHS == 0b0000)    
                ADCON0bits.CHS = 0b0001;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0001)
                ADCON0bits.CHS = 0b0010;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0010)
                ADCON0bits.CHS = 0b0000;    // Cambio de canal
            */
            __delay_us(40);                 // Tiempo de adquisici?n
            
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversi?n
        }
        if(conteo <= PWM3){
            PORTCbits.RC3 = 1;
        }
        else {
            PORTCbits.RC3 = 0;
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0x07;                // AN0, AN1 y AN2 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    TRISA = 0x07;                // AN0, AN1 y AN2 como entrada
    PORTA = 0;                      
    TRISC = 0;                // PORTC como salida
    PORTC = 0;                      
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b011;    // 500kHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b11;     // Fosc/XX presonalizada
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuración PWM
    PR2 = 155;                  // periodo de 20ms
    
    // Configuración CCP
    // CCP1 para SERVO 1
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    CCPR1L = 16>>2;
    CCP1CONbits.DC1B = 16 & 0b11;    // 
    // CCP2 para SERVO 2
    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP2
    CCP2CON = 0;                // Apagamos CCP2
    CCP2CONbits.CCP2M = 0b1100; // PWM
    CCPR2L = 16>>2;
    CCP2CONbits.DC2B0 = 16 & 0b1; // 
    CCP2CONbits.DC2B1 = 16>>1 & 0b1;
    
    // Configuracion del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM

    // TMR0
    OPTION_REGbits.T0CS = 0;    // TMR0 como temporizador
    OPTION_REGbits.PSA = 0;     // prescaler a TMR0
    OPTION_REGbits.PS = 0b001;  // prescaler (001) = 1:4
    TMR0 = 250;
    
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    INTCONbits.T0IE = 1;        // Habilitamos interrupciones del TMR0
    INTCONbits.T0IF = 0;        // Limpiamos bandera de interrupción
    
}

/* Función para hacer la interpolación lineal del valor de la entrada analógica 
*  usando solo el registro ADRESH (8 bits) al ancho de pulso del PWM (10 bits), 
* usando la ecuación:
*  y = y0 + [(y1 - y0)/(x1-x0)]*(x-x0)
*  -------------------------------------------------------------------
*  | x0 -> valor mínimo de ADC | y0 -> valor mínimo de ancho de pulso|
*  | x  -> valor actual de ADC | y  -> resultado de la interpolación | 
*  | x1 -> valor máximo de ADC | y1 -> valor máximo de ancho de puslo|
*  ------------------------------------------------------------------- 
*/
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

void RESET_TMR0(uint8_t TMR_VAR){
    TMR0 = TMR_VAR;                 // TMR0 = valor
    INTCONbits.T0IF = 0;            // Limpiamos bandera de interrupción
    return;
}