#include <msp430.h>
#include <stdint.h>

volatile uint16_t lastCap = 0;
volatile uint16_t pulseWidth = 0;
volatile uint8_t  gotPulse = 0;

void initCapture(void)
{
    P3DIR &= ~BIT7;            // TSOP38238 output on P3.7
    P3SEL0 |= BIT7;
    P3SEL1 &= ~BIT7;

    TB0CTL   = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
    TA0CCTL3 = CM_1 | CCIS_0 | CAP | SCS | CCIE; // rising-edge capture
}

#pragma vector = TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    if (TB0IV == TB0IV_TBCCR3) {
        uint16_t now = TB0CCR3;
        pulseWidth = now - lastCap; // µs difference at 1 MHz
        lastCap = now;
        gotPulse = 1;
    }
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;
    initCapture();
    __enable_interrupt();

    while (1) {
        if (gotPulse) {
            gotPulse = 0;
            uint16_t us = pulseWidth;

            if (us > 250 && us < 350)
                ; // 10-cycle pulse
            else if (us > 1200 && us < 1500)
                ; // 50-cycle pulse
            else if (us > 2500 && us < 3000)
                ; // 80-cycle pulse
        }
    }
}
