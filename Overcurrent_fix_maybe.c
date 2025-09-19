// ===== Overcurrent tuning (integers only) =====
#define THRESH_HI_A  500   // raise if false-tripping at idle, lower if not tripping on stall
#define THRESH_LO_A  450   // hysteresis (≈ HI - 50)
#define THRESH_HI_B  500
#define THRESH_LO_B  450

#define TRIP_MS      50    // sustain time (40–60 ms)
#define CLEAR_MS     1000  // button hold to clear (ms)
#define ARM_DELAY_MS 250   // ignore overcurrent for first 250 ms after power-up/resume
#define REARM_MS     100   // must sit below THRESH_LO for 100 ms before we allow a new trip

void OC_1msTask(bool clear_button_pressed)
{
  // ------- ARMING (blanking) window after power-up or resume -------
  static uint16_t arm_ms = ARM_DELAY_MS;
  static bool armed = false;
  if (!armed) {
    if (arm_ms) arm_ms--;
    else armed = true;
  }

  // ------- Read ADCs (optionally average 3 quick samples) -------
  // If you prefer single samples, replace read_avg() calls with adc_read_inch().
  // Tiny local averaging to reduce PWM-phase jitter:
  auto uint16_t read_avg(uint8_t inch) {
    uint32_t acc = 0;
    acc += adc_read_inch(inch);
    acc += adc_read_inch(inch);
    acc += adc_read_inch(inch);
    return (uint16_t)(acc / 3);
  }
  uint16_t adcA = read_avg(4);   // A4 (P8.7)  - Motor A sense
  uint16_t adcB = read_avg(5);   // A5 (P8.6)  - Motor B sense

  // ------- 1-pole IIR smoothing (alpha = 1/8), integer math -------
  static uint32_t filtA = 0, filtB = 0;               // Q8.8 fixed-point
  if (filtA == 0) {                                   // init first run
    filtA = ((uint32_t)adcA) << 8;
    filtB = ((uint32_t)adcB) << 8;
  }
  filtA += ((((uint32_t)adcA) << 8) - filtA) >> 3;    // >>3 = /8
  filtB += ((((uint32_t)adcB) << 8) - filtB) >> 3;
  uint16_t a_avg = (uint16_t)(filtA >> 8);
  uint16_t b_avg = (uint16_t)(filtB >> 8);

  // ------- Hysteresis decision per channel -------
  static bool overA = false, overB = false;
  if (!overA) overA = (a_avg >= THRESH_HI_A); else overA = (a_avg > THRESH_LO_A);
  if (!overB) overB = (b_avg >= THRESH_HI_B); else overB = (b_avg > THRESH_LO_B);

  // ------- Require a "quiet" period below LO before we allow trips again -------
  static uint16_t rearm_ms = 0;
  if (a_avg < THRESH_LO_A && b_avg < THRESH_LO_B) {
    if (rearm_ms < REARM_MS) rearm_ms++;
  } else {
    rearm_ms = 0;
  }
  bool can_trip = (rearm_ms >= REARM_MS);

  // Final overcurrent decision for this tick
  bool over_now = armed && can_trip && (overA || overB);

  // ------- Your existing state machine (unchanged) -------
  static uint16_t over_ms = 0, btn_ms = 0;
  extern void (*cut_cb)(void);       // from your module
  extern void (*resume_cb)(void);    // from your module
  static uint8_t state = 0;          // 0=NORMAL, 1=OVER, 2=FAULT

  switch (state) {
    case 0: // NORMAL
      if (over_now) { over_ms = 1; state = 1; }
      break;

    case 1: // OVER (timing)
      if (!over_now) { over_ms = 0; state = 0; }
      else if (++over_ms >= TRIP_MS) {
        if (cut_cb) cut_cb();              // cut PWM
        btn_ms = 0;
        state = 2;                         // FAULT
      }
      break;

    case 2: // FAULT (latched)
      btn_ms = clear_button_pressed ? (uint16_t)(btn_ms + (btn_ms < 0xFFFF)) : 0;
      if (btn_ms >= CLEAR_MS) {
        if (resume_cb) resume_cb();        // resume PWM
        // re-arm blanking after resume
        armed = false; arm_ms = ARM_DELAY_MS;
        rearm_ms = 0;
        over_ms = btn_ms = 0;
        state = 0;
      }
      break;
  }
}
