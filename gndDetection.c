/* 
 * file: gndDetection.c
 * author: J. van Hooydonk
 * comments: Marklin ground detection for PCB of G. Giebens https://github.com/GeertGiebens
 *
 * revision history:
 *  v0.1 Creation (22/06/2025)
 */

#include "gndDetection.h"
#include "ln.h"

/**
 * ground detection initialisation
 * @param fptr: the function pointer to the (callback) ground detection handler
 */
void gndDetectionInit(gndDetectionCallback_t fptr)
{
    // initialise ground detection callback function (function pointer)
    gndDetectionCallback = fptr;
    // init of the ground detection inputs
    gndDetectionInputInit();
    // set debounce state to maximum value
    for (uint8_t i = 0; i < 16; i++)
    {
        // 
        oldGndDetectionInputState[i] = false;
        // clear debounce states
        debounceState0[i] = 0;
        debounceState1[i] = 0;
        // set initial flag state to update required
        gndDetectionInputUpdateRequired[i] = true;
    }
    // init of the ADC
    gndDetectionInitAdc();
}

/**
 * init of the ground detection pins
 */
void gndDetectionInputInit()
{
    // setup the digital input ports and pins
    //  - input 1: RD2
    //  - input 2: RD3
    //  - input 3: RC4
    //  - input 4: RC5
    //  - input 5: RD4
    //  - input 6: RD5
    //  - input 7: RD6
    //  - input 8: RD7
    //  - input 9: RB0
    //  - input 10: RB1
    //  - input 11: RB2
    //  - input 12: RB3
    //  - input 13: RB4
    //  - input 14: RB5
    //  - input 15: RB6
    //  - input 16: RB7
    //  - input sync: RE0

    // set all pins of PORTB to inputs, no pull-up and enable analog mode
    TRISB = 0xff;
    WPUB = 0x00;
    ANSELB = 0xff;
    // set pins 4 and 5 of PORTC to inputs, no pull-up and enable analog mode
    TRISC |= 0x30;
    WPUC &= 0xcf;
    ANSELC |= 0x30;
    // set pins 2 to 7 of PORTD to inputs, no pull-up and enable analog mode
    TRISD |= 0xfc;
    WPUD &= 0x03;
    ANSELD |= 0xfc;
    // set pin 0 of PORTE to input, no pull-up and disable analog mode
    TRISE |= 0x01;
    WPUE &= 0x0fe;
    ANSELE &= 0xfe;
}

/**
 * init the ADC
 */
void gndDetectionInitAdc()
{
    // enable the ADC
    ADCON0bits.ADON = true;
    // select FRC clock
    ADCON0bits.ADCS = true;
    // result right justified
    ADCON0bits.ADFM = true;
    // set ADC interrupt to low priority
    IPR1bits.ADIP = false;
    // set ADC Interrupt enable bit
    PIE1bits.ADIE = true;
    // start scanning
    startAdc(channels[gndDetectionInputIndex]);
}

/**
 * get a sample from the GND detection input(s)
 */
void sampleGndDetectionInput()
{
    // this function has been  called after the (low priority) interrupt
    // from the ADC was fired
    if (getPowerState())
    {
        // store ADC result
        adcResults[gndDetectionInputIndex] = ((uint16_t) (ADRESH << 8) + ADRESL);
        // if all inputs are readed then handle the ADC results
        if (gndDetectionInputIndex == 0x0f)
        {
            handleGndDetectionInputs();
        }
        // start next GND detection conversion
        gndDetectionInputIndex += 1;
        gndDetectionInputIndex &= 0x0f;
    }
    else
    {
        // reset GND detection input index and debounce state
        for (uint8_t i = 0; i < 16; i++)
        {
            // clear the debounce states
            debounceState0[i] = 0;
            debounceState1[i] = 0;
            // set update required flag
            gndDetectionInputUpdateRequired[i] = true;
        }
        // reset index counter
        gndDetectionInputIndex = 0;
    }
    startAdc(channels[gndDetectionInputIndex]);
}

/**
 * get the state of the power
 * @return the state of the power
 */
bool getPowerState()
{
    // check pin RE0 if power is ON or OFF
    if (RE0)
    {
        // power OFF
        delayPowerOn = 0;
        delayPowerOff += 1;
        if (delayPowerOff > POWER_OFF_DELAY)
        {
            powerState = false;
        }
    }
    else
    {
        // power ON
        delayPowerOff = 0;
        delayPowerOn += 1;
        if (delayPowerOn > POWER_ON_DELAY)
        {
            powerState = true;
        }
    }
    return powerState;
}

/**
 * handle the GND detection inputs
 */
void handleGndDetectionInputs()
{
    // this function has been called after all the inputs are sampled
    for (uint8_t i = 0; i < 16; i++)
    {
        if (adcResults[i] < THRESHOLD_LOW)
        {
            if (oldGndDetectionInputState[i] || gndDetectionInputUpdateRequired[i])
            {
                debounceState0[i] += 1;
                debounceState1[i] = 0;
                if ((debounceState0[i] > DEBOUNCE_DELAY_0) || gndDetectionInputUpdateRequired[i])
                {
                    // then update the new state (= false)
                    oldGndDetectionInputState[i] = false;
                    debounceState0[i] = 0;
                    updateGndDetectionInput(i, true);
                    // clear update required flag
                    gndDetectionInputUpdateRequired[i] = false;
                }
            }
        }
        else if (adcResults[i] > THRESHOLD_HIGH)
        {
            if (!oldGndDetectionInputState[i] || gndDetectionInputUpdateRequired[i])
            {
                debounceState0[i] = 0;
                debounceState1[i] += 1;
                if ((debounceState1[i] > DEBOUNCE_DELAY_1) || gndDetectionInputUpdateRequired[i])
                {
                    // then update the new state (= false)
                    oldGndDetectionInputState[i] = true;
                    debounceState1[i] = 0;
                    updateGndDetectionInput(i, false);
                    // clear update required flag
                    gndDetectionInputUpdateRequired[i] = false;
                }
            }
        }
        else
        {
            debounceState0[i] = 0;
            debounceState1[i] = 0;
        }
    }
}

/**
 * start the ADC conversion
 * @param channel: the channel to be read (RA0 = 0x00)
 */
void startAdc(uint8_t channel)
{
    // before starting the conversion, discharge the cap
    dischargeSampleCap();
    // connect ADC to the channel number
    ADPCH = channel;
    // start conversion
    ADCON0bits.ADGO = true;
}

/**
 * this function connects the ADC to Vss and thus discharge the cap to provide
 * an accurtae reading
 */
void dischargeSampleCap()
{
    // connect ADC to Vss
    ADPCH = 0x3c;
}

/**
 * update the GND detetction input state
 * @param index: the index of the GND detetction input
 * @param state: the value ot the GND detetction input
 */
void updateGndDetectionInput(uint8_t index, bool value)
{
    // handle the input state (in the callback function)
    (*gndDetectionCallback)(index, value);
}
