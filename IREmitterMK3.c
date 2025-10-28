#include <msp430.h>
#include <stdint.h>

#define F_CARRIER   38000UL         // 38 kHz carrier
#define CYCLE_US    (1000000UL / F_CARRIER)  // ≈ 26.3 µs per cycle

static void initCarrier(void);
static void sendBurst(uint16_t cycles);

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;
    initCarrier();

    while (1)
    {
        sendBurst(20);    // short burst (~526 µs)
        sendBurst(50);    // mid burst (~1.3 ms)
        sendBurst(80);    // long burst (>70 → needs big gap)
    }
}

static void initCarrier(void)
{
    // P3.6 → TB0.2 output
    P3DIR  |= BIT6;
    P3SEL0 |= BIT6;
    P3SEL1 &= ~BIT6;

    TB0CCR0  = (1000000UL / F_CARRIER) - 1;   // period @1MHz
    TB0CCR2  = TB0CCR0 / 3;                   // ~33% duty
    TB0CCTL2 = OUTMOD_7;                      // reset/set
    TB0CTL   = TBSSEL__SMCLK | MC__UP;        // up mode
}

// cycles = number of carrier cycles
static void sendBurst(uint16_t cycles)
{
    uint32_t burst_us = cycles * CYCLE_US;
    uint32_t gap_us   = (cycles > 70) ? burst_us * 4 : burst_us;

    TB0CCTL2 |= OUTMOD_7;                     // enable modulation
    __delay_cycles(burst_us);                 // 1 µs per cycle at 1MHz
    TB0CCTL2 &= ~OUTMOD_7;                    // stop modulation
    __delay_cycles(gap_us);                   // enforce gap
}
