/*
 * file: general.c
 * author: J. van Hooydonk
 * comments: general variables, settings and routines
 *
 * revision history:
 *  v1.0 Creation (26/05/2025)
 */

#include "general.h"

// <editor-fold defaultstate="collapsed" desc="initialisation">

/**
 * initialisation
 */
void init()
{
    // init the LN driver and give the function pointer for the callback
    lnInit(&lnRxMessageHandler);
    // init Marklin ground detection and give the function pointer for the callback
    gndDetectionInit(&gndDetectionHandler);
    // init a temporary LN message queue for transmitting a LN message
    initQueue(&lnTxMsg);
    // init of the hardware elements (timer, ISR, port)
    initTmr3();
    initIsr();
    // init ports
    initPorts();
}

/**
 * initialisation of the timer 3
 */
void initTmr3(void)
{
    // timer 3 will by used as startup to drive 8 servo motors
    // timer 3 must give a low priority interrupt every 2500탎
    // so that 8 x 2500탎 will give a time of 20ms (= servo period time)
    TMR3CLK = 0x01; // clock source to Fosc / 4
    T3CON = 0b00110000; // T3CKPS = 0b11 (1:8 prescaler)
    // SYNC = 0 (ignored)
    // RD16 = 0 (timer 3 in 8 bit operation)
    // TMR1ON = 0 (timer 3 is disabled)
    WRITETIMER3(~TIMER3_2500us); // set delay in timer 3
}

/**
 * initialisation of the interrupt service routine
 */
void initIsr(void)
{
    // set global interrupt parameters
    INTCONbits.IPEN = true; // enable priority levels on interrupt
    INTCONbits.GIEH = true; // enable all high priority interrupts
    INTCONbits.GIEL = true; // enable all low priority interrupts
    // set timer 3 interrupt parameters
    IPR4bits.TMR3IP = false; // timer 3 interrupt low priority
    PIE4bits.TMR3IE = true; // enable timer 3 overflow interrupt
    T3CONbits.ON = true; // enable timer 3
}

/**
 * initialistaion of the IO pins (to read the DIP switch address) *
 */
void initPorts()
{
    // setup digital inputs to read the DIP switches
    // PORTA = A0 A1 -- --  -- -- -- --
    // PORTC = -- -- -- --  A5 A4 A3 A2
    // PORTD = -- -- -- --  -- -- A7 A6

    // we only need to read 7 DIP switches (A7 - A6 - C0 - C1 - C2 - C3 - D0)
    // this makes the address A4 - A10 for the complete LN address selection
    // A0 - A3 will be the index of the 16 detectors
    TRISA |= 0xc0; // disable output (= input) on pin A6 - A7
    TRISC |= 0x0f; // disable output (= input) on pin C0 - C3
    TRISC |= 0x03; // disable output (= input) on pin D0 - D1

    ANSELA &= 0x3f; // enable TTL input buffer on pin A6 - A7
    ANSELC &= 0xf0; // enable TTL input buffer on pin C0 - C3
    ANSELD &= 0xfc; // enable TTL input buffer on pin D0 - D1
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ISR low priority">

// there are two possible low interrupt triggers, coming from
// the EUSART data receiver and/or coming from the timer 1 overrun flag

/**
 * low priority interrupt service routine
 */
void __interrupt(low_priority) isrLow(void)
{
    if (PIR4bits.TMR1IF)
    {
        // timer 1 interrupt
        // clear the interrupt flag and handle the request
        PIR4bits.TMR1IF = false;
        lnIsrTmr1();
    }
    if (PIR3bits.RC1IF)
    {
        // EUSART RC interupt
        if (RC1STAbits.FERR)
        {
            // EUSART framing error (linebreak detected)
            // read RCREG to clear the interrupt flag and FERR bit
            _ = RC1REG;
            // retreive (recover) the last transmitted LN message
            recoverLnMessage(&lnTxTempQueue);
            // this framing error detection takes about 600탎
            // (10bits x 60탎) and a linebreak duration is specified at
            // 900탎, so add 300탎 after this detection time to complete
            // a full linebreak
            startLinebreak(LINEBREAK_SHORT);
        }
        else
        {
            // EUSART data received
            // handle the received data byte
            lnIsrRc();
        }
    }
    if (PIR4bits.TMR3IF)
    {
        // timer 3 interrupt
        // clear the interrupt flag and handle the request
        PIR4bits.TMR3IF = false;
        // reload timer 3
        WRITETIMER3(~TIMER3_2500us); // set delay in timer 3
    }
    if (PIR1bits.ADIF)
    {
        // ADC completed
        PIR1bits.ADIF = false;
        // sample (next) GND detection input
        sampleGndDetectionInput();
        
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ISR (low/high priority)">

// there are two possible high interrupt triggers, coming from
// the timer 3 overrun flag and/or coming from the comparator

/**
 * high priority interrupt service routine
 */
void __interrupt(high_priority) isrHigh(void)
{
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="routines">

void gndDetectionHandler(uint8_t index, bool value)
{
    // create a 'general sensor input state report'
    // reference https://wiki.rocrail.net/doku.php?id=loconet:ln-pe-en
    //           https://wiki.rocrail.net/doku.php?id=loconet:lnpe-parms-en
    // OPCODE = 0xB2 (OPC_INPUT_REP) 
    // IN1 = sensor address
    //       0, A7, A6, A5, A4, A3, A2, A1
    //       (A1 - A3 = index of gnd detection input)
    //       (A4 - A7 = DIP switches 1 - 4)
    // IN2 = sensor address and status
    //       0, X, I, L, A11, A10, A9, A8
    //       (A8 - A11 = DIP switches 5 - 8)
    //       (L = sensor level = state of gnd detection input)
    //       (I = A0)
    //       (X = 0)

    // get DIP switch address
    uint8_t address = getDipSwitchAddress();

    // make arguments IN1, IN2
    uint8_t IN1 = (((address << 3) & 0x78) + ((index >> 1) & 0x07)) & 0x7f;
    uint8_t IN2 = (address >> 4) & 0x0f;
    if ((index & 0x01) == 0x01)
    {
        IN2 |= 0x20;
    }
    if (value)
    {
        IN2 |= 0x10;
    }

    // enqueue message
    enQueue(&lnTxMsg, 0xB2);
    enQueue(&lnTxMsg, IN1);
    enQueue(&lnTxMsg, IN2);
    // transmit the LN message
    lnTxMessageHandler(&lnTxMsg);    
}

/**
 * this is the callback function for the LN receiver
 * @param lnRxMsg: the lN message queue
 */
void lnRxMessageHandler(lnQueue_t* lnRxMsg)
{
    while (!isQueueEmpty(lnRxMsg))
    {
        // analyse the received LN message from queue
        switch (lnRxMsg->values[lnRxMsg->head])
        {
            case 0x82:
            {
                // global power OFF request
                break;
            }
            case 0x83:
            {
                // global power ON request
                for (uint8_t i = 0; i < 16; i++)
                {
                    gndDetectionInputUpdateRequired[i] = true;
                }
                break;
            }
        }
        // clear the received LN message from queue
        deQueue(lnRxMsg);
    }
}

/**
 * get the state of the DIP switches (0 - 9)
 * @return the address (or value of the DIP switches)
 */
uint8_t getDipSwitchAddress()
{
    // return the address
    // address = 0 0 0 0  0 0 0 0  0 A6 A5 A4  A3 A2 A1 A0

    // we need to read 8 DIP switches (A0 - A7)
    // this makes the address A4 - A11 for the complete LN address selection
    // A0 - A3 will be the index of the gnd detection input (16 detectors)
    uint8_t address = 0;

    address += ((PORTA >> 7) & 0x01); // A0 on PORT A, pin 7
    address += ((PORTA >> 5) & 0x02); // A1 on PORT A, pin 6
    address += ((PORTC << 2) & 0x3c); // A5 - A2 on PORT C, pin 0 - 3
    address += ((PORTD << 6) & 0xc0); // A6 - A7 on PORT D, pin 0 - 1

    return address;
}

// </editor-fold>
