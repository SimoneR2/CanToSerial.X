#define USE_AND_MASKS
/*==============================================================================
 *PROGRAM: Can to Serial converter
 *WRITTEN BY: Simone Righetti
 *DATA: 07/02/2016
 *VERSION: 1.0
 *FILE SAVED AS: CanToSerial.c
 *FOR PIC: 18F4480
 *CLOCK FREQUENCY: 16 MHz
 *PROGRAM FUNCTION: 

======================================          
=         INPUT AND OUTPUTS          =            
=   RA0 => Tensione batteria (ADC)   =            
=   RA1 => Warning LED               =            
=   RD4-5-6-7 => ECCP (PWM Motore)   =   
=   RB2/RB3 => CANBus                =            
======================================
 */
#include <string.h>
#include <stdio.h>
#define _XTAL_FREQ 16000000
#include <xc.h>
#include "pic18_config.h"
#include "idCan.h"
#include "CANlib.h"
#include <usart.h>
#include "delay.h"
#include "delay.c"
void configurazione_iniziale(void);
void serialWrite(unsigned char *array);
volatile bit newMessageCan = 0;
volatile bit newMessageUsart = 0;
CANmessage msg;
unsigned char usartTx [11] = 0;
unsigned char usartRx[11] = 0;
volatile bit RTR = 0;
unsigned long id = 0;
unsigned char data [8] = 0;

__interrupt(low_priority) void ISR_bassa(void) {
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        if (CANisRxReady()) {
            CANreceiveMessage(&msg); //leggilo e salvalo

            //istruzioni per salvare i dati arrivati
            usartTx [0] = msg.RTR;
            usartTx [1] = msg.identifier;
            usartTx [2] = (msg.identifier >> 8);
            for (int i = 0; i < 8; i++) {
                usartTx[(i + 3)] = msg.data[i];
            }
            newMessageCan = 1;
        }
        PIR3bits.RXB0IF = 0;
        PIR3bits.RXB1IF = 0;
    }
    if (PIR1bits.RCIF == 1) {
        getsUSART((char*) usartRx, 11);
    }
}

void main(void) {
    while (1) {
        if (newMessageCan == 1) {
            putsUSART((char *) usartTx);
            newMessageCan = 0;
        }
        if (newMessageUsart == 1) {
            RTR = usartRx [0];
            id = usartRx[1];
            id = ((id << 8) | usartRx[0]);
            for (int i = 0; i < 8; i++) {
                data[i] = usartRx[(i + 3)];
            }
            if (CANisTxReady()) {
                if (RTR == 1){
                CANsendMessage(id, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
                }
                else {
                    CANsendMessage(id, data, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                }
            }
        }
    }
}

void configurazione_iniziale(void) {
    CloseUSART(); //disabilita periferica per poterla impostare

    RCONbits.IPEN = 1; //abilita priorità interrupt
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    PIR1bits.RCIF = 0; //azzera flag interrupt ricezione seriale

    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    IPR1bits.RCIP = 0; //interrupt bassa priorità ricezione seriale

    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    PIE1bits.RCIE = 1; //abilita interrupt ricezione seriale

    INTCONbits.GIEH = 1; //abilita interrupt 
    INTCONbits.GIEL = 1; //abilita interrupt periferiche



    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE &
            CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON); //Canbus 125kHz (da cambiare)

    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_BRGH_HIGH, 103); //Seriale asincrona, 8bit, 9600baud

    LATA = 0x00;
    TRISA = 0b01111101;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0b10111111;

    LATD = 0x00;
    TRISD = 0x00;

    LATE = 0x00;
    TRISE = 0xFF;
    delay_ms(2);
}