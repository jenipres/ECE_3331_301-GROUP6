#define IRCONTROLTYPE   1

//NEC protocol - 1 uS tick period - SMCLK 1Mhz - better accuracy
#define STARTBITMIN     8200    // 8.2 ms
#define STARTBITMAX     10000   // 10.0 ms
#define PAUSEBITMIN     4200    // 4.2 ms
#define PAUSEBITMAX     4800    // 4.8 ms
#define BITMIN          400     // 0.4 ms
#define BITMAX          1000    // 1.0 ms
#define ZEROBITMIN      400     // 0.4 ms
#define ZEROBITMAX      700     // 0.7 ms
#define ONEBITMIN       1400    // 1.4 ms
#define ONEBITMAX       1900    // 1.9 ms


void setupTimer(void){  //Configure P3.7 for TB0.3 capture
    P3DIR  &= ~BIT7;    //Input
    P3SEL0 |=  BIT7;    //Peripheral
    P3SEL1 &= ~BIT7;    //Route TB0.3

    TB0CCTL3 = CM_3 + CCIS_0 + SCS + CAP + CCIE;    //Capture mode both edges
    TB0CCR4 = 50000;                                  //error detection timeout (possible adjust)*****
    TB0CCTL4 = CCIE;                        //maybe +CM_2
    TB0CTL = TASSEL_2 + MC_2 + TAIE + ID_0;         //SMCLK = 1MHz
}
//globals we'll use for decoding
volatile uint16_t last_edge = 0;
volatile uint8_t bit_index = 0;
volatile uint32_t ir_data = 0;
volatile uint8_t ir_ready = 0;

//ISR -- Decoding and interprets IR signal
#pragma vector = TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void) {
    switch(__even_in_range(TB0IV, TB0IV_TBIFG)) {

    case TB0IV_TBCCR3: {  // Edge capture
        uint16_t now = TB0CCR3;
        uint16_t width = now - last_edge;
        last_edge = now;

        // Reset timeout
        TB0CCR4 = now + 50000;
        TB0CCTL4 &= ~CCIFG;

        //Decode pulses
        if (width >= STARTBITMIN && width <= STARTBITMAX) {
            // Start of frame
            bit_index = 0;
            ir_data = 0;
        }
        else if (width >= ZEROBITMIN && width <= ZEROBITMAX) {
            // Logic 0
            ir_data <<= 1;
            bit_index++;
        }
        else if (width >= ONEBITMIN && width <= ONEBITMAX) {
            // Logic 1
            ir_data = (ir_data << 1) | 1;
            bit_index++;
        }
        else if (width >= PAUSEBITMIN && width <= PAUSEBITMAX) {
            // Pause / inter-frame space
            bit_index = 0;
        }
        else {
            // Noise or error
            bit_index = 0;
            ir_data = 0;
        }

        //full 32-bit command received
        if (bit_index >= 32) {
            ir_ready = 1;    //Signal main loop
            bit_index = 0;
        }
        break;
    }

    case TB0IV_TBCCR4: {  //Timeout
        bit_index = 0;
        ir_data = 0;
        ir_ready = 0;
        TB0CCTL4 &= ~CCIFG;
        break;
    }

    case TB0IV_TBIFG:     //Timer overflow
        TB0CTL &= ~TBIFG;
        break;

    default: break;
    }
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   //Stop watchdog
    setupTimer();

    __enable_interrupt();

    while (1) {
        if (ir_ready) {
            //get full 32-bit IR command
            uint32_t cmd = ir_data;
            ir_ready = 0;

            // map to rover controls
            if (cmd == 0xA90) {//match to emitter
                dir_A_forward();
                dir_B_forward();
            }
            else if (cmd == 0x690) {//match to emitter
                dir_A_backward();
                dir_B_backward();
            }
            else if (cmd == 0xE90) {//match to emitter
                dir_A_forward();
                dir_B_backward();
            }
            else if (cmd == 0x190) {//match to emitter
                dir_B_forward();
                dir_A_backward();
            }
            else {
                // stop / default
            }
        }
    }
}



