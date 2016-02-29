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
=   RA1 => Warning LED               =            
=   RD4-5-6-7 => ECCP (PWM Motore)   =   
=   RB2/RB3 => CANBus                =  
=   RD Luci segnalazione           
======================================
 */
//#include <string.h>
//#include <stdio.h>
#define _XTAL_FREQ 16000000
#include <xc.h>
#include "pic18_config.h"
#include "idCan.h"
#include "CANlib.h"
#include "usart.h"
#include "delay.h"
#include "delay.c"

#define SerialTx PORTDbits.RD0 
#define SerialRx PORTDbits.RD1 
#define CanTx PORTDbits.RD2 
#define CanRx PORTDbits.RD3 
#define MotoreFlag PORTDbits.RD4
#define AbsFlag PORTDbits.RD5
#define SterzoFlag PORTDbits.RD6

void configurazione_iniziale(void);
void can_interpreter(void);

volatile bit newMessageCan = 0;
volatile bit newMessageUsart = 0;
CANmessage msg;
unsigned char USART_Rx[7] = 0;
volatile bit RTR_Flag = 0;
unsigned long id = 0;
unsigned char data [8] = 0;

volatile bit dir = 0;
unsigned char set_steering[8] = 0;
unsigned char set_speed[8] = 0;
unsigned char analogic_brake[8] = 0;

volatile unsigned char set_steering_old = 0;
volatile unsigned char set_speed_old[8] = 0;
volatile unsigned char analogic_brake_old = 0;

volatile unsigned long timeCounter = 0; //1 = 10mS
unsigned long previousTimeCounter = 0;

unsigned char SerialOutput [] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA} ;
unsigned int left_speed = 0;
unsigned int right_speed = 0;

__interrupt(low_priority) void ISR_bassa(void) {
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        CanRx = ~CanRx; //toggle led ricezione can bus
        if (CANisRxReady()) {
            CANreceiveMessage(&msg); //leggilo e salvalo
            RTR_Flag = msg.RTR;
            id = msg.identifier;
            newMessageCan = 1;
            for (char i = 0; i < 8; i++) {
                data[i] = msg.data[i];
            }
        }
        PIR3bits.RXB0IF = 0;
        PIR3bits.RXB1IF = 0;
    }
    if (PIR1bits.RC1IF == 1) {
        SerialRx = ~SerialRx; //toggle led ricezione seriale
        getsUSART((char*) USART_Rx, 7);
        if ((USART_Rx[0] == 0xAA)&&(USART_Rx[6] == 0xAA)) {

            set_steering_old = set_steering[0];
            set_speed_old[0] = set_speed[0];
            set_speed_old[1] = set_speed[1];
            analogic_brake_old = analogic_brake[0];

            set_speed[2] = USART_Rx[1]; //direction
            set_speed[0] = USART_Rx[2];
            set_speed[1] = USART_Rx[3];
            set_steering[0] = USART_Rx[4];
            analogic_brake[0] = USART_Rx[5];
        }
        PIR1bits.RC1IF = 0;
    }
    if (PIR2bits.TMR3IF) { //interrupt timer, ogni 10mS
        timeCounter++; //incrementa di 1 la variabile timer
        TMR3H = 0x63; //reimposta il timer
        TMR3L = 0xC0; //reimposta il timer
        PIR2bits.TMR3IF = 0; //azzera flag interrupt timer
    }
}

void main(void) {
    configurazione_iniziale();
    while (1) {
        if (newMessageCan == 1) {
            
            can_interpreter();
            putsUSART((char *) SerialOutput);
            newMessageCan = 0;
            
        }
        if (newMessageUsart == 1) {
            CanTx = ~CanTx; //toggle led invio can bus
            if (set_steering_old != set_steering[0]) {
                CANsendMessage(STEERING_CHANGE, set_steering, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            }
            if ((set_speed_old[0] != set_speed[0])&&(set_speed_old[1] != set_speed[1])) {
                CANsendMessage(SPEED_CHANGE, set_speed, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            }
            if (analogic_brake_old != analogic_brake[0]) {
                analogic_brake[1] = 0x01;
                CANsendMessage(BRAKE_SIGNAL, analogic_brake, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            }
            newMessageUsart = 0;
        }
    }
}

void can_interpreter(void) {
    if (id == ECU_STATE) {
        if (RTR_Flag == 1) { //Se è arrivata la richiesta presenza centraline
            previousTimeCounter = timeCounter;
            data[0] = 0x03;
            while (CANisTxReady() != 1);
            CANsendMessage(id, data, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            MotoreFlag = 1;
            AbsFlag = 0; //resetta flag
            SterzoFlag = 0; //resetta flag
        } else {
            if (data[0] == 0x01) {
                AbsFlag = 1;
                PORTCbits.RC5 = 0; //DEBUG
            }
            if (data[0] == 0x02) {
                SterzoFlag = 1;
                PORTCbits.RC4 = 0; //DEBUG
            }
        }
        if (previousTimeCounter - timeCounter > 450) {
            SerialOutput[4] = SterzoFlag; //preparazione pacchetto seriale
            SerialOutput[4] = ((SerialOutput[5] < 1) | AbsFlag); //preparazione pacchetto seriale
            SerialOutput[4] = ((SerialOutput[5] < 1) | MotoreFlag); //preparazione pacchetto seriale
        }
    }
    if ((id == ACTUAL_SPEED)&&(RTR_Flag == 0)) {
        left_speed = data[1];
        left_speed = ((left_speed << 8) | (data[0]));
        right_speed = data[3];
        right_speed = ((right_speed << 8) | (data[2]));
        SerialOutput[3] = ((left_speed + right_speed) / 2);
        SerialOutput[2] = (((left_speed + right_speed) / 2) >> 8);
    }
    if (id == LOW_BATTERY) {
        SerialOutput[5] = data[0];
    }
}

void configurazione_iniziale(void) {
    CloseUSART(); //disabilita periferica per poterla impostare

    RCONbits.IPEN = 1; //abilita priorità interrupt
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    PIR1bits.RC1IF = 0; //azzera flag interrupt ricezione seriale

    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    IPR1bits.RC1IP = 0; //interrupt bassa priorità ricezione seriale

    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    PIE1bits.RC1IE = 1; //abilita interrupt ricezione seriale
    PIE2bits.TMR3IE = 1; //abilita interrupt timer3

    INTCONbits.GIEH = 1; //abilita interrupt 
    INTCONbits.GIEL = 1; //abilita interrupt periferiche



    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE &
            CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON); //Canbus 125kHz (da cambiare)

    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_BRGH_HIGH & USART_CONT_RX, 103); //Seriale asincrona, 8bit, 9600baud

    //RCSTAbits.SPEN = 1; //abilita la periferica

    LATA = 0x00;
    TRISA = 0b01111101;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0b10111111;

    //impostazione timer3 per contatore========
    T3CON = 0x01; //abilita timer
    PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3
    IPR2bits.TMR3IP = 0; //interrupt bassa priorità timer 3
    TMR3H = 0x63;
    TMR3L = 0xC0;
    //==========================================
}