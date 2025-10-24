#include <msp430.h>
#include <stdbool.h>

#define F_SMCLK (1000000UL)
typedef unsigned char unit8_t;

typedef enum {
  DIR_STOP,
  DIR_FORWARD,
  DIR_RIGHT,
  DIR_LEFT,
} DirState;

volatile uint8_t  inst         = 0;
volatile bool     txpending    = false;

static inline void initGPIO(void);
static inline void initUART(void);
static inline void sendByte(uint8_t b);

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;
  PM5CTL0 &= ~LOCKLPM5;       //Unllock GPIO

  initGPIO();
  initUART();

  while (1) {
    sendByte(b: DIR_STOP);       __delay_cycles(F_SMCLK);
    sendByte(b: DIR_FORWARD);       __delay_cycles(F_SMCLK);
    sendByte(b: DIR_RIGHT);       __delay_cycles(F_SMCLK);
  }
}

static inline void initGPIO(void)
{
  P1DIR |= BIT0;     //P1.0 output
  P1OUT &= ~BIT0;    //start OFF
}

static inline void initUART(void)
{
  P4SEL0 |= BIT2;                //P4.2 = UCA0TXD
  P4SEL1 &= ~BIT2;

  UCA0CTLW0 |= UCSWRST;          //hold in reset
  UCA0CTLW0 |= UCSSEL__SMCLK     //SMCLK = 1 MHz
  UCA0BR0 = 26;                  // 1 000 000 / 2400 = 416 -> 26*16
  UCA0BR1 = 0;                   
  UCA0MCTLW = UCBRF_1 | UCOS16;  // fractional part + overlapping
  UCA0CTLW0 &= ~UCSWRST;         //release reset

  UCA0IE |= UCTXIE;              //enable TX interrupt
}


#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch (__even_in_range(UCA0IV, USCI_UART_UCTXIFG)) {
    case USCI_UART_UCTXIFG:
      if (inst == DIR_STOP)          P1OUT |= BIT0;
      elseif (inst == DIR_FORWARD)       P1OUT ^= BIT0;
      elseif (inst == DIR_REVERSE)   P1OUT &= ~BIT0;

      txpending = false;             //transmission finished
      __bic_SR_register_on_exit(LPM0_bits);
      break;
    default: break;
  }
}
  















  
