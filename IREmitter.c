//IR Transmitter- NEC protocol
//SMCLK = 1 MHz (DCO)
//Carrier = 38 kHz (Timer_A0, CCR0)
//Default output pin: P1.2 (change macros below to match your board)
//Use a transistor to drive IR LED (MCU pin controls transistor base/gate)

#include <msp430.h>
#include <stdint.h>

//User-adjustable pin mapping (change these for your MCU/board)
/*
 * Pick a pin that maps to TA0.1 (Timer_A output). Many MSP430 variants use P1.2 for TA0.1,
 * but check your device datasheet / pin table and change these macros accordingly.
 *
 * Update:
 *   IR_PORT_SEL0 / IR_PORT_SEL1  -> PxSEL0 / PxSEL1 register for the chosen port
 *   IR_PORT_DIR / IR_PORT_OUT    -> PxDIR / PxOUT register
 *   IR_PIN                       -> BITn for chosen pin (e.g., BIT2)
 */
#define IR_PORT_SEL0   P1SEL0
#define IR_PORT_SEL1   P1SEL1
#define IR_PORT_DIR    P1DIR
#define IR_PORT_OUT    P1OUT
#define IR_PIN         BIT2      // P1.2 by default; change if necessary

// ---------- Carrier frequency parameters ----------
#define SMCLK_FREQ     1000000UL   // 1 MHz
#define CARRIER_HZ     38000UL     // target 38 kHz

// Compute Timer_A0 CCR0 top for ~38 kHz with SMCLK=1MHz
#define CARRIER_TOP    ((SMCLK_FREQ / CARRIER_HZ) - 1)   // ~26

// ---------- NEC timing (microseconds, since SMCLK=1 MHz) ----------
#define NEC_MARK       560     // mark duration for bits (µs)
#define NEC_ONE_SPACE  1690    // space duration for logical '1' (µs)
#define NEC_ZERO_SPACE 560     // space duration for logical '0' (µs)
#define NEC_LEAD_MARK  9000    // leader mark (µs)
#define NEC_LEAD_SPACE 4500    // leader space (µs)
#define NEC_POST_MARK  560     // end mark (sometimes optional)

// ---------- Delay helper (1 µs resolution) ----------
static inline void delay_us(uint32_t us) {
    // With CS configured to 1 MHz (MCLK=SMCLK=DCO=1MHz), __delay_cycles(us) ~= us microseconds
    while (us) {
        // __delay_cycles(1) might be optimized away in some compilers; using loop ensures behavior.
        __delay_cycles(1);
        --us;
    }
}

// ---------- Carrier control ----------
// We keep Timer_A running, and toggle the pin function to enable/disable the PWM output.
// When the pin is mapped to timer output (SEL0=1, SEL1=0 on FR series), the PWM appears.
// When the pin is plain GPIO and driven low, no carrier is emitted.

static inline void carrier_init_gpio_off(void) {
    // Ensure pin is output low and not mapped to timer
    IR_PORT_SEL0 &= ~IR_PIN;
    IR_PORT_SEL1 &= ~IR_PIN;
    IR_PORT_DIR  |=  IR_PIN;   // output
    IR_PORT_OUT  &= ~IR_PIN;   // low
}

static inline void carrier_on(void) {
    // Route pin to Timer_A0.1 (primary function assumed: SEL0=1, SEL1=0)
    IR_PORT_SEL0 |= IR_PIN;
    IR_PORT_SEL1 &= ~IR_PIN;
    // leave direction as peripheral
}

static inline void carrier_off(void) {
    // Route pin back to GPIO and drive low
    IR_PORT_SEL0 &= ~IR_PIN;
    IR_PORT_SEL1 &= ~IR_PIN;
    IR_PORT_DIR  |= IR_PIN;
    IR_PORT_OUT  &= ~IR_PIN;
}

// ---------- Timer_A carrier setup ----------
void init_carrier_timer(void) {
    // Timer_A0 config:
    // TA0CCR0 = CARRIER_TOP
    // TA0CCR1 = 50% duty
    // TA0CCTL1 = OUTMOD_7 (reset/set) or OUTMOD_6 depending on preference; OUTMOD_7 is common.
    TA0CCR0 = (uint16_t)CARRIER_TOP;
    TA0CCR1 = (uint16_t)((CARRIER_TOP + 1) / 2);  // roughly 50% duty
    TA0CCTL1 = OUTMOD_7;   // Reset/Set output mode

    // Keep timer running in Up mode (counts 0..CCR0). We can keep it running always.
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR; // SMCLK, up mode, clear TAR
}

// ---------- NEC send primitives ----------
static inline void send_mark(uint32_t usec) {
    carrier_on();
    delay_us(usec);
    // Do not turn off here if caller wants immediate space; caller will turn off.
}

static inline void send_space(uint32_t usec) {
    carrier_off();
    delay_us(usec);
}

// send one NEC bit: mark + space (LSB-first)
static void send_nec_bit(uint8_t bit) {
    send_mark(NEC_MARK);
    if (bit)
        send_space(NEC_ONE_SPACE);
    else
        send_space(NEC_ZERO_SPACE);
}

// send a byte LSB-first
static void send_byte_lsb_first(uint8_t b) {
    for (uint8_t i = 0; i < 8; ++i) {
        send_nec_bit((b >> i) & 1);
    }
}

// send full NEC (addr, ~addr, cmd, ~cmd)
void send_nec(uint8_t addr, uint8_t cmd) {
    // Leader
    send_mark(NEC_LEAD_MARK);
    send_space(NEC_LEAD_SPACE);

    // 32 bits: addr, ~addr, cmd, ~cmd  (LSB first for each byte)
    send_byte_lsb_first(addr);
    send_byte_lsb_first((uint8_t)~addr);
    send_byte_lsb_first(cmd);
    send_byte_lsb_first((uint8_t)~cmd);

    // Final mark (often a trailing 560 us)
    send_mark(NEC_POST_MARK);
    carrier_off();
}

// ---------- Clock setup (make sure DCO/SMCLK at 1 MHz) ----------
void setup_clock_to_1MHz(void) {
    // FRxx style CS registers. If your MSP430 variant differs, use the appropriate clock init.
    // Unlock CS registers
#ifdef CSKEY
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_0; // DCO = ~1 MHz
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__VLOCLK; // MCLK=SMCLK=DCO, ACLK=VLO
    CSCTL3 = DIVM__1 | DIVS__1;
    CSCTL0_H = 0; // lock
#else
    // If CS registers not present on your device, you can set DCO via other means or rely on default ~1MHz DCO.
#endif
}

// ---------- Example main: map pushbuttons to send commands ----------
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog

    // Setup clocks (1 MHz)
    setup_clock_to_1MHz();

    // Init carrier Timer but keep output pin off (GPIO low)
    init_carrier_timer();
    carrier_init_gpio_off();

    // Example buttons on P1.1..P1.4 (change to match your hardware)
    P1DIR &= ~(BIT1 | BIT2 | BIT3 | BIT4);    // inputs
    P1OUT |=  (BIT1 | BIT2 | BIT3 | BIT4);    // pull-ups
    P1REN |=  (BIT1 | BIT2 | BIT3 | BIT4);

    // Simple debounce/backoff variables
    for (;;) {
        // Simple polling of buttons; in a real remote you'd use interrupts + debouncing
        if (!(P1IN & BIT1)) {   // button pressed (active low)
            // Example: send "forward" -> addr 0x00, cmd 0x10 (choose your codes)
            send_nec(0x00, 0x10);
            __delay_cycles(200000); // small delay between repeats (200 ms)
        }
        else if (!(P1IN & BIT2)) {
            send_nec(0x00, 0x11);  // e.g., backward
            __delay_cycles(200000);
        }
        else if (!(P1IN & BIT3)) {
            send_nec(0x00, 0x12);  // left
            __delay_cycles(200000);
        }
        else if (!(P1IN & BIT4)) {
            send_nec(0x00, 0x13);  // right
            __delay_cycles(200000);
        }
        // sleep or small wait to reduce CPU usage
        __bis_SR_register(LPM0_bits + GIE); // optional: requires wake on button; simpler to use small delay
    }

    return 0;
}

