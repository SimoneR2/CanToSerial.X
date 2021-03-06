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

#define SerialTx LATDbits.LATD0 
#define SerialRx LATDbits.LATD1 
#define CanTx LATDbits.LATD2 
#define CanRx LATDbits.LATD3 
#define MotoreFlag LATDbits.LATD4
#define AbsFlag LATDbits.LATD5
#define SterzoFlag LATDbits.LATD6

void configurazione_iniziale(void);
void can_interpreter(void);
void usart_interpreter(void);
void usart_tx(void);
void usart_rx(void);

volatile bit sendMessage = 0;
volatile bit newMessageCan = 0;
volatile bit newMessageUsart = 0;
CANmessage msg;
unsigned char USART_Rx[7] = 0;
volatile bit RTR_Flag = 0;
volatile unsigned long id = 0;
BYTE data [8] = 0;

unsigned char buffer = 0;

volatile bit dir = 0;
unsigned char set_steering[8] = 0;
unsigned char set_speed[8] = 0;
unsigned char analogic_brake[8] = 0;

volatile unsigned char set_steering_old = 0;
volatile unsigned char set_speed_old[8] = 0;
volatile unsigned char analogic_brake_old = 0;

volatile unsigned long timeCounter = 0; //1 = 10mS
unsigned long previousTimeCounter = 0;

unsigned char SerialOutput [] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, '\0'};
unsigned char SerialOutputOld [] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, '\0'};
unsigned int left_speed = 0;
unsigned int right_speed = 0;
unsigned char spam_counter = 0;

__interrupt(high_priority) void ISR_alta(void) {
    usart_rx();

    if ((PIR3bits.RXB1IF == 1) || (PIR3bits.RXB0IF == 1)) { //RICEZIONE CAN
        CanRx = ~CanRx; //toggle led ricezione can bus
        if (CANisRxReady()) {
            CANreceiveMessage(&msg); //leggilo e salvalo
            RTR_Flag = msg.RTR;
            id = msg.identifier;
            newMessageCan = 1;
            for (unsigned char i = 0; i < 8; i++) {
                data[i] = msg.data[i];
            }
        }
        PIR3bits.RXB1IF = 0;
        PIR3bits.RXB0IF = 0;
    }
    if (PIR2bits.TMR3IF == 1) { //interrupt timer, ogni 10mS
        timeCounter++; //incrementa di 1 la variabile timer
        TMR3H = 0x63; //reimposta il timer
        TMR3L = 0xC0; //reimposta il timer
        PIR2bits.TMR3IF = 0; //azzera flag interrupt timer
    }
}

void main(void) {
    configurazione_iniziale();
    CanTx = 1;
    CanRx = 1;
    SerialTx = 1;
    SerialRx = 1;
    MotoreFlag = 1;
    SterzoFlag = 1;
    AbsFlag = 1;
    delay_ms(500);
    CanTx = 0;
    CanRx = 0;
    SerialTx = 0;
    SerialRx = 0;
    MotoreFlag = 0;
    SterzoFlag = 0;
    AbsFlag = 0;
    while (1) {
        if (newMessageCan == 1) {
            can_interpreter();
            SerialOutput[0] = 0xAA; //costruzione pacchetto standard
            SerialOutput[6] = 0xAA; //costruzione pacchetto standard
            SerialOutput[7] = '\0'; //costruzione pacchetto standard
            usart_tx();
            newMessageCan = 0;
        }

        if (newMessageUsart == 1) {
            usart_interpreter();
            if (set_steering_old != set_steering[0]) {
                while (CANisTxReady() != 1);
                CANsendMessage(STEERING_CHANGE, set_steering, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                CanTx = ~CanTx; //toggle led invio can bus
            }
            if ((set_speed_old[0] != set_speed[0])&&(set_speed_old[1] != set_speed[1])) {
                while (CANisTxReady() != 1);
                CANsendMessage(SPEED_CHANGE, set_speed, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                CanTx = ~CanTx; //toggle led invio can bus
            }
            if (analogic_brake_old != analogic_brake[0]) {
                analogic_brake[1] = 0x01;
                while (CANisTxReady() != 1);
                CANsendMessage(BRAKE_SIGNAL, analogic_brake, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                CanTx = ~CanTx; //toggle led invio can bus
            }
            newMessageUsart = 0;
        }
    }
}

void can_interpreter(void) {

    if (id == ECU_STATE) {
        if (RTR_Flag == 1) { //Se � arrivata la richiesta presenza centraline
            previousTimeCounter = timeCounter;
            data[0] = 0x03;
            while (CANisTxReady() != 1);
            CANsendMessage(ECU_STATE, data, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            MotoreFlag = 1;
            AbsFlag = 0; //resetta flag
            SterzoFlag = 0; //resetta flag
            CanTx = 1;
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
        if (SerialOutput[2] == 0) {
            SerialOutput[2] = 0b10000000;
        }
    }
    if (id == LOW_BATTERY) {
        SerialOutput[5] = data[0];
    }
}

void usart_interpreter(void) {
    //SALVATAGGIO DATI========================
    set_steering_old = set_steering[0];
    set_speed_old[0] = set_speed[0];
    set_speed_old[1] = set_speed[1];
    analogic_brake_old = analogic_brake[0];
    //========================================

    //INTERPRETE PER BIT DIREZIONE============
    if (USART_Rx[1] == 1) {
        set_speed[2] = 0; //direction bit
    }
    if (USART_Rx[1] == 2) {
        set_speed[2] = 1; //direction bit
    }
    //========================================

    set_speed[0] = USART_Rx[3];
    set_speed[1] = USART_Rx[2];

    if (set_speed [1] == 0b10000000) {
        set_speed[1] = 0b00000000;
    }

    set_steering[0] = USART_Rx[4];

    analogic_brake[0] = USART_Rx[5];
}

void usart_rx(void) {
    if (PIR1bits.RCIF == 1) {
        PIE1bits.RCIE = 0; //disabilita interrupt ricezione seriale
        for (unsigned char i = 0; i < 8; i++) {
            SerialRx = ~SerialRx;

            while (PIR1bits.RCIF != 1);
            USART_Rx[i] = RCREG;
            PIR1bits.RCIF = 0;
        }
        if ((USART_Rx[0] == 0xAA)&&(USART_Rx[6] == 0xAA)) {
            PORTAbits.RA1 = 1;
            newMessageUsart = 1;
        }
        PIR1bits.RCIF = 0; //disabilita interrupt ricezione seriale
        PIE1bits.RCIE = 1; //disabilita interrupt ricezione seriale

    }
}

void usart_tx(void) {
    /*
     * La routine di invio dati delle librerie
     * considera il primo dato a 0 come la fine
     * della trasmissione, per cui in caso di 
     * dato = 0, il programma lo pone a 1.
     */
    for (char i = 0; i < 6; i++) {
        if (SerialOutput[i] == 0) {
            SerialOutput[i] = 1; //debug
        }
    }

    /*
     * Queste istruzioni servono ad evitare
     * di continuare a mandare messaggi via
     * seriale verificando se quello che 
     * si sta per mandare � uguale o meno
     * a quello gi� mandato in precedenza
     */

    spam_counter = 0;
    for (char i = 1; i < 6; i++) {
        if (SerialOutput[i] == SerialOutputOld[i]) {
            spam_counter++;
        }
    }
    if ((BusyUSART() != 1)&&(spam_counter != 5)) {
        INTCONbits.GIEH = 0;
        INTCONbits.GIEL = 0;
        SerialTx = ~SerialTx;
        putsUSART(SerialOutput);
        for (char i = 0; i < 6; i++) {
            SerialOutputOld[i] = SerialOutput[i];
        }
    }
    INTCONbits.GIEH = 1;
        INTCONbits.GIEL = 1;

}

void configurazione_iniziale(void) {
    CloseUSART(); //disabilita periferica per poterla impostare
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON); //Canbus 125kHz (da cambiare)
    //    CANOperationMode(CAN_OP_MODE_CONFIG);
    //    CANSetFilter(CAN_FILTER_B1_F1, 0b00000000010, CAN_CONFIG_STD_MSG);
    //    CANOperationMode(CAN_OP_MODE_NORMAL);
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_BRGH_HIGH & USART_CONT_RX, 25); //Seriale asincrona, 8bit, 9600baud

    RCSTAbits.SPEN = 1; //abilita la periferica


    RCONbits.IPEN = 1; //abilita priorit� interrupt
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    PIR1bits.RCIF = 0; //azzera flag interrupt ricezione seriale

    IPR3bits.RXB1IP = 1; //interrupt bassa priorit� per can
    IPR3bits.RXB0IP = 1; //interrupt alta priorit� per can
    IPR1bits.RCIP = 1; //interrupt

    

    LATA = 0x00;
    TRISA = 0b01111101;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0b10111111;

    LATD = 0x00;
    TRISD = 0x00;

    LATE = 0x00;
    TRISE = 0b00000100;

    ADCON1bits.PCFG0 = 1; //AN0 input analogico
    ADCON1bits.PCFG1 = 1; //AN0 input analogico
    ADCON1bits.PCFG2 = 1; //AN0 input analogico
    ADCON1bits.PCFG3 = 1; //AN0 input analogico

    //impostazione timer3 per contatore========
    T3CON = 0x01; //abilita timer
    PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3
    IPR2bits.TMR3IP = 0; //interrupt bassa priorit� timer 3

    TMR3H = 0x63;
    TMR3L = 0xC0;
    //==========================================
    PIE2bits.TMR3IE = 1; //abilita interrupt timer3

    INTCONbits.GIEH = 1; //abilita interrupt 
    INTCONbits.GIEL = 1; //abilita interrupt periferiche
//    while (RCREG != 0xAA); //attesa per evitare di bloccare la telecomando
//    WriteUSART(0xAA);
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    PIE1bits.RCIE = 1; //disabilita interrupt ricezione seriale
    delay_ms(300);
}