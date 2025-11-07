#include <msp430.h>
#include <stdint.h>

volatile uint16_t lastCap = 0;
volatile uint16_t pulseWidth = 0;
volatile uint8_t  gotPulse = 0;

volatile uint16_t riseTime = 0;
volatile uint16_t fallTime = 0;
//volatile uint16_t pulseWidth = 0;
volatile uint8_t  edge = 0;       // 0 = waiting for rising, 1 = waiting for falling
//volatile uint8_t  gotPulse = 0;

void initCapture(void)
{
    // Configure P2.7 as TB0.6 capture input
    P2DIR &= ~BIT7;             // Input
    P2SEL0 |= BIT7;             // Select TB0.6 function
    P2SEL1 &= ~BIT7;

    // Configure Timer_B0 in continuous mode using SMCLK
    TB0CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR;

    // Capture on BOTH edges, synchronize, interrupt enabled
    TB0CCTL6 = CM_3 | CCIS_0 | CAP | SCS | CCIE;
}

#pragma vector = TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    switch (__even_in_range(TB0IV, TB0IV_TBCCR6))
    {
        case TB0IV_TBCCR6:
        {
            uint16_t now = TB0CCR6;

            if (edge == 0)
            {
                // Rising edge detected
                riseTime = now;
                edge = 1;

                // Switch to capture falling edge next
                TB0CCTL6 = CM_2 | CCIS_0 | CAP | SCS | CCIE;
            }
            else
            {
                // Falling edge detected
                fallTime = now;
                pulseWidth = (uint16_t)(riseTime - fallTime);  // Auto-wrap works fine
                gotPulse = 1;
                edge = 0;

                // Switch back to rising edge for next pulse
                TB0CCTL6 = CM_1 | CCIS_0 | CAP | SCS | CCIE;
            }

            break;
        }
        default:
            break;
    }
}



void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;
    
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    P2DIR |= BIT1 | BIT2 | BIT3 | BIT4;   // Set P2.1–P2.4 as outputs
    P2OUT &= ~(BIT1 | BIT2 | BIT3 | BIT4);

    initCapture();
    __enable_interrupt();

    while (1) {
        if (gotPulse) {
            gotPulse = 0;
            uint16_t us = pulseWidth;

            if (us > 50 && us < 200)
            {
                //P2OUT ^= BIT1;
                __delay_cycles(10000);
            }
            else if (us > 400 && us < 600)
            {
                P2OUT ^= BIT2;
                __delay_cycles(10000);
            }
            else if (us > 700 && us < 900)
            {
                P2OUT ^= BIT3;
                __delay_cycles(10000);
            }

            else if (us > 1550 && us < 1650)
            {
                P2OUT ^= BIT4;
                __delay_cycles(10000);
            }
            else if (us > 15000 && us<25000)
            {
                P2OUT ^= BIT1;
                __delay_cycles(10000);
            }
            else if ( us > 27000 && us < 35000)
            {
                P2OUT ^= BIT2;
                __delay_cycles(10000);
            }
            else if ( us > 45000 && us < 55000)
            {
                P2OUT ^= BIT3;
                __delay_cycles(10000);
            }
            else if ( us > 56000 && us <65000)
            {
                P2OUT ^= BIT4;
                __delay_cycles(10000);
            }      
        }
    }
}

