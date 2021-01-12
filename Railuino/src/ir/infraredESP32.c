// ===================================================================
// === Low-level IR stuff ============================================
// ===================================================================

/**
* I took the original infrared2.c and tried to rewrite the code according to
* IRSend.cpp by David Conran
* https://github.com/crankyoldgit/IRremoteESP8266
*/

// From the datasheet
// https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf
// CPU clock frequency is adjustable from 80 MHz to 240 MHz
// Integrated crystal: 40 MHz crystal
#define SYSCLOCK 40000000
//#define RC5_T1      889
//#define TOPBIT 0x80000000
// Constants
// RC-5/RC-5X
const uint16_t kRc5T1 = 889;
const uint32_t kRc5MinCommandLength = 113778;
const uint32_t kRc5MinGap = kRc5MinCommandLength - kRC5RawBits * (2 * kRc5T1);
const uint16_t kRc5ToggleMask = 0x800;  // The 12th bit.
const uint16_t kRc5SamplesMin = 11;

// Common (getRClevel())
const int16_t kMark = 0;
const int16_t kSpace = 1;

// Modulate the IR LED for the given period (usec) and at the duty cycle set.
/// @param[in] usec The period of time to modulate the IR LED for, in
///  microseconds.
/// @return Nr. of pulses actually sent.
/// @note
///   The ESP8266 has no good way to do hardware PWM, so we have to do it all
///   in software. There is a horrible kludge/brilliant hack to use the second
///   serial TX line to do fairly accurate hardware PWM, but it is only
///   available on a single specific GPIO and only available on some modules.
///   e.g. It's not available on the ESP-01 module.
///   Hence, for greater compatibility & choice, we don't use that method.
/// Ref:
///   https://www.analysir.com/blog/2017/01/29/updated-esp8266-nodemcu-backdoor-upwm-hack-for-ir-signals/
void mark(uint16_t usec) {
    // Handle the simple case of no required frequency modulation.
    if (!modulation || _dutycycle >= 100) {
        ledOn();
        _delayMicroseconds(usec);
        ledOff();
        return 1;
    }

    // I have no idea (yet) if we will need frequency modulation, so I assume, we do.

    // Not simple, so do it assuming frequency modulation.
    uint16_t counter = 0;
    IRtimer usecTimer = IRtimer();
    // Cache the time taken so far. This saves us calling time, and we can be
    // assured that we can't have odd math problems. i.e. unsigned under/overflow.
    uint32_t elapsed = usecTimer.elapsed();

    while (elapsed < usec) {  // Loop until we've met/exceeded our required time.
    ledOn();
    // Calculate how long we should pulse on for.
    // e.g. Are we to close to the end of our requested mark time (usec)?
    _delayMicroseconds(std::min((uint32_t)onTimePeriod, usec - elapsed));
    ledOff();
    counter++;
    if (elapsed + onTimePeriod >= usec)
    return counter;  // LED is now off & we've passed our allotted time.
    // Wait for the lesser of the rest of the duty cycle, or the time remaining.
    _delayMicroseconds(
        std::min(usec - elapsed - onTimePeriod, (uint32_t)offTimePeriod));
        elapsed = usecTimer.elapsed();  // Update & recache the actual elapsed time.
    }
    return counter;
}

/// Turn the pin (LED) off for a given time.
/// Sends an IR space for the specified number of microseconds.
/// A space is no output, so the PWM output is disabled.
/// @param[in] time Time in microseconds (us).
void space(uint32_t time) {
    ledOff();
    if (time == 0) return;
    _delayMicroseconds(time);
}


/// Set the output frequency modulation and duty cycle.
/// @param[in] freq The freq we want to modulate at.
///  Assumes < 1000 means kHz else Hz.
/// @param[in] duty Percentage duty cycle of the LED.
///   e.g. 25 = 25% = 1/4 on, 3/4 off.
///   If you are not sure, try 50 percent.
///   This is ignored if modulation is disabled at object instantiation.
/// @note Integer timing functions & math mean we can't do fractions of
///  microseconds timing. Thus minor changes to the freq & duty values may have
///  limited effect. You've been warned.
void _enableIROut(uint32_t freq, uint8_t duty) {
    // Set the duty cycle to use if we want freq. modulation.
    if (modulation) {
        _dutycycle = std::min(duty, kDutyMax);
    } else {
        _dutycycle = kDutyMax;
    }
    if (freq < 1000)  // Were we given kHz? Supports the old call usage.
    freq *= 1000;

    uint32_t period = calcUSecPeriod(freq);
    // Nr. of uSeconds the LED will be on per pulse.
    onTimePeriod = (period * _dutycycle) / kDutyMax;
    // Nr. of uSeconds the LED will be off per pulse.
    offTimePeriod = period - onTimePeriod;
}

void enableIROut(uint32_t khz) {
    // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
    // The IR output will be on pin 3 (OC2B).
    // This routine is designed for 36-40KHz; if you use it for other values, it's up to you
    // to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
    // TIMER2 is used in phase-correct PWM mode, with OCR2A controlling the frequency and OCR2B
    // controlling the duty cycle.
    // There is no prescaling, so the output frequency is 16MHz / (2 * OCR2A)
    // To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
    // A few hours staring at the ATmega documentation and this will all make sense.
    // See my Secrets of Arduino PWM at http://arcfn.com/2009/07/secrets-of-arduino-pwm.html for details.
    _enableIROut(khz, 25);
    /// @param[in] duty Percentage duty cycle of the LED.
    ///   e.g. 25 = 25% = 1/4 on, 3/4 off.
    ///   If you are not sure, try 50 percent.
    ///   This is ignored if modulation is disabled at object instantiation.

    //
    //   // Disable the Timer2 Interrupt (which is used for receiving IR)
    //   TIMSK3 = _BV(OCIE3A); //Timer2 Overflow Interrupt
    //
    //   pinMode(3, OUTPUT);
    //   digitalWrite(3, LOW); // When not sending PWM, we want it low
    //
    //   // COM2A = 00: disconnect OC2A
    //   // COM2B = 00: disconnect OC2B; to send signal set to 10: OC2B non-inverted
    //   // WGM2 = 101: phase-correct PWM with OCRA as top
    //   // CS2 = 000: no prescaling
    //   TCCR3A = _BV(WGM31);
    //   TCCR3B = _BV(WGM33) | _BV(CS30);
    //
    //   // The top value for the timer.  The modulation frequency will be SYSCLOCK / 2 / OCR2A.
    //   ICR3 = SYSCLOCK / 2 / khz/ 1000;
    //   OCR3A = ICR3 / 3;
    //
}

/// Send a Philips RC-5/RC-5X packet.
/// Status: RC-5 (stable), RC-5X (alpha)
/// @param[in] data The message to be sent.
/// @param[in] nbits The number of bits of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
/// @note Caller needs to take care of flipping the toggle bit.
///   That bit differentiates between key press & key release.
///   For RC-5 it is the MSB of the data.
///   For RC-5X it is the 2nd MSB of the data.
/// @todo Testing of the RC-5X components.
void sendRC5(const uint64_t data, uint16_t nbits, bool extended) {
  if (nbits > sizeof(data) * 8) return;  // We can't send something that big.
  bool skipSpace = true;
  bool field_bit = true;
  // Set 36kHz IR carrier frequency & a 1/4 (25%) duty cycle.
  enableIROut(36);

    data = data << (32 - nbits);

  IRtimer usecTimer = IRtimer();
  for (uint16_t i = 0; i <= repeat; i++) {
    usecTimer.reset();

    // Header
    // First start bit (0x1). space, then mark.
    if (skipSpace)
      skipSpace = false;  // First time through, we assume the leading space().
    else
      space(kRc5T1);
    mark(kRc5T1);
    // Field/Second start bit.
    if (field_bit) {  // Send a 1. Normal for RC-5.
      space(kRc5T1);
      mark(kRc5T1);
    } else {  // Send a 0. Special case for RC-5X. Means 7th command bit is 1.
      mark(kRc5T1);
      space(kRc5T1);
    }

    // Data
    for (uint64_t mask = 1ULL << (nbits - 1); mask; mask >>= 1)
      if (data & mask) {  // 1
        space(kRc5T1);    // 1 is space, then mark.
        mark(kRc5T1);
      } else {         // 0
        mark(kRc5T1);  // 0 is mark, then space.
        space(kRc5T1);
      }
    // Footer
    space(std::max(kRc5MinGap, kRc5MinCommandLength - usecTimer.elapsed()));
  }
}
