// ==== Overcurrent trip config (edit to your hardware) ====
static const float I_MAX_A        = 1.00f;   // measured loaded max current (amps)
static const float RSENSE_OHMS    = 0.10f;   // shunt resistor value (ohms)
static const float SENSE_GAIN     = 1.0f;    // op-amp gain from shunt to ADC (1.0 if none)
static const float VREF_VOLTS     = 2.5f;    // internal reference voltage used by ADC

// If you want per-channel thresholds later:
// static const float I_MAX_A_CH0 = 1.00f;  // for P8.6 (A19)
// static const float I_MAX_A_CH1 = 1.00f;  // for P8.7 (A20)

extern void pwm_enable(bool on);
extern void pwm_apply(void);

static volatile uint8_t overcurrent_fault = 0;

// Convert trip current (1.5× I_MAX) to 12-bit ADC codes
static inline uint16_t oc_trip_counts(void) {
    const float itrip   = 1.5f * I_MAX_A;
    const float v_sense = itrip * RSENSE_OHMS * SENSE_GAIN;
    float codes = (v_sense / VREF_VOLTS) * 4095.0f;
    if (codes < 0.0f)   codes = 0.0f;
    if (codes > 4095.0f) codes = 4095.0f;
    return (uint16_t)(codes + 0.5f);
}

void adc12_init(void) {
    // --- Make sure P1.0 is configured as output for the fault LED ---
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // --- Select analog function on P8.6 and P8.7 ---
    // FR6989 map: P8.6=A19, P8.7=A20
    P8SEL1 |= BIT6 | BIT7;
    P8SEL0 |= BIT6 | BIT7;

    // --- Turn on 2.5 V internal reference ---
    REFCTL0 |= REFVSEL_2 | REFON;                  // 2.5 V
    while (REFCTL0 & REFGENBUSY) { __no_operation(); }
    __delay_cycles(8000);                          // ~1 ms @ 8 MHz for reference to settle

    // --- ADC12_B setup: 12-bit, sample timer, repeated sequence of channels (A19->A20) ---
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;             // 16 ADC clocks sample, ADC on
    ADC12CTL1 = ADC12SHP       |                   // sample-and-hold pulse mode (timer)
                ADC12SSEL_3    |                   // SMCLK source
                ADC12DIV_3     |                   // SMCLK/4 for ADC clock
                ADC12CONSEQ_3;                     // repeated sequence of channels
    ADC12CTL2 = ADC12RES_2;                        // 12-bit

    // MEM0: A19 (P8.6), VRSEL=internal 2.5 V
    ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_19;

    // MEM1: A20 (P8.7), VRSEL=internal 2.5 V, mark end of sequence
    ADC12MCTL1 = ADC12VRSEL_1 | ADC12INCH_20 | ADC12EOS;

    // Enable interrupts for both memory locations
    ADC12IER0 = ADC12IE0 | ADC12IE1;

    // Precompute threshold once (shared for both channels here)
    (void)oc_trip_counts();

    // Enable and start conversions (continuous)
    ADC12CTL0 |= ADC12ENC;
    ADC12CTL0 |= ADC12SC;
}

// === ADC12 ISR: trip if either channel exceeds threshold ===
#pragma vector=ADC12_B_VECTOR
__interrupt void ADC12_B_ISR(void) {
    switch (__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG)) {
    case ADC12IV_ADC12IFG0: { // MEM0 (P8.6 / A19)
        if (!overcurrent_fault) {
            uint16_t s0 = ADC12MEM0;
            if (s0 >= oc_trip_counts()) {
                overcurrent_fault = 1;
                pwm_enable(false);        // kill PWM immediately
                P1OUT |= BIT0;            // fault LED on
                ADC12CTL0 &= ~(ADC12ENC | ADC12SC); // stop conversions (optional)
            }
        }
    } break;

    case ADC12IV_ADC12IFG1: { // MEM1 (P8.7 / A20)
        if (!overcurrent_fault) {
            uint16_t s1 = ADC12MEM1;
            if (s1 >= oc_trip_counts()) {
                overcurrent_fault = 1;
                pwm_enable(false);
                P1OUT |= BIT0;
                ADC12CTL0 &= ~(ADC12ENC | ADC12SC);
            }
        }
    } break;

    default:
        break;
    }

    // Wake the main loop in case it's sleeping in LPM0
    __bic_SR_register_on_exit(LPM0_bits);
}
