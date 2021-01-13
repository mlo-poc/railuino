#include "IRtimer.cpp"
// ===================================================================
// === Low-level IR stuff ============================================
// ===================================================================

/**
* I took the original infrared2.c and tried to rewrite the code according to
* IRSend.cpp by David Conran
* https://github.com/crankyoldgit/IRremoteESP8266
*/
#define ESP32
// From the datasheet
// https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf
// CPU clock frequency is adjustable from 80 MHz to 240 MHz
// Integrated crystal: 40 MHz crystal
#define SYSCLOCK 40000000
#define ALLOW_DELAY_CALLS true

//#define RC5_T1      889
//#define TOPBIT 0x80000000
// Constants
// RC-5/RC-5X

const uint16_t kRC5RawBits = 14;
const uint16_t kRC5Bits = kRC5RawBits - 2;

const uint16_t kRc5T1 = 889;
const uint32_t kRc5MinCommandLength = 113778;
const uint32_t kRc5MinGap = kRc5MinCommandLength - kRC5RawBits * (2 * kRc5T1);
const uint16_t kRc5ToggleMask = 0x800;  // The 12th bit.
const uint16_t kRc5SamplesMin = 11;

uint16_t onTimePeriod;
uint16_t offTimePeriod;

#if defined(ESP32)
const uint16_t IRpin = 32;
#else
const uint16_t IRpin = 3;
#endif

// Constants
// Offset (in microseconds) to use in Period time calculations to account for
// code excution time in producing the software PWM signal.
#if defined(ESP32)
// Calculated on a generic ESP-WROOM-32 board with v3.2-18 SDK @ 240MHz
const int8_t kPeriodOffset = -2;
#elif (defined(ESP8266) && F_CPU == 160000000L)  // NOLINT(whitespace/parens)
// Calculated on an ESP8266 NodeMCU v2 board using:
// v2.6.0 with v2.5.2 ESP core @ 160MHz
const int8_t kPeriodOffset = -2;
#else  // (defined(ESP8266) && F_CPU == 160000000L)
// Calculated on ESP8266 Wemos D1 mini using v2.4.1 with v2.4.0 ESP core @ 40MHz
const int8_t kPeriodOffset = -5;
#endif  // (defined(ESP8266) && F_CPU == 160000000L)

const uint8_t kDutyDefault = 50;  // Percentage
const uint8_t kDutyMax = 100;     // Percentage
// delayMicroseconds() is only accurate to 16383us.
// Ref: https://www.arduino.cc/en/Reference/delayMicroseconds
const uint16_t kMaxAccurateUsecDelay = 16383;
//  Usecs to wait between messages we don't know the proper gap time.
const uint32_t kDefaultMessageGap = 100000;


// Common (getRClevel())
const int16_t kMark = 0;
const int16_t kSpace = 1;

const int8_t outputOn = HIGH;
const int8_t outputOff = LOW;

uint8_t _dutycycle;

boolean modulation = false;


#if ALLOW_DELAY_CALLS
/// An ESP8266 RTOS watch-dog timer friendly version of delayMicroseconds().
/// @param[in] usec Nr. of uSeconds to delay for.
void _delayMicroseconds(uint32_t usec) {
    // delayMicroseconds() is only accurate to 16383us.
    // Ref: https://www.arduino.cc/en/Reference/delayMicroseconds
    if (usec <= kMaxAccurateUsecDelay) {
        delayMicroseconds(usec);
    } else {
        // Invoke a delay(), where possible, to avoid triggering the WDT.
        delay(usec / 1000UL);  // Delay for as many whole milliseconds as we can.
        // Delay the remaining sub-millisecond.
        delayMicroseconds(static_cast<uint16_t>(usec % 1000UL));
    }
}
#else  // ALLOW_DELAY_CALLS
/// A version of delayMicroseconds() that handles large values and does NOT use
/// the watch-dog friendly delay() calls where appropriate.
/// @note Use this only if you know what you are doing as it may cause the WDT
///  to reset the ESP8266.
void _delayMicroseconds(uint32_t usec) {
    for (; usec > kMaxAccurateUsecDelay; usec -= kMaxAccurateUsecDelay)
    delayMicroseconds(kMaxAccurateUsecDelay);
    delayMicroseconds(static_cast<uint16_t>(usec));
}
#endif  // ALLOW_DELAY_CALLS


/// Turn off the IR LED.
void ledOff() {
    digitalWrite(IRpin, outputOff);
}

/// Turn on the IR LED.
void ledOn() {
    digitalWrite(IRpin, outputOn);
}

void initIR() {
    if (modulation)
    _dutycycle = kDutyDefault;
    else
    _dutycycle = kDutyMax;
    pinMode(IRpin, OUTPUT);
    ledOff();  // Ensure the LED is in a known safe state when we start.
}

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
        //        return 1;
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
    //    return counter;  // LED is now off & we've passed our allotted time.
    // Wait for the lesser of the rest of the duty cycle, or the time remaining.
    _delayMicroseconds(
        std::min(usec - elapsed - onTimePeriod, (uint32_t)offTimePeriod));
        elapsed = usecTimer.elapsed();  // Update & recache the actual elapsed time.
    }
    //    return counter;
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


/// Calculate the period for a given frequency.
/// @param[in] hz Frequency in Hz.
/// @param[in] use_offset Should we use the calculated offset or not?
/// @return nr. of uSeconds.
/// @note (T = 1/f)
uint32_t calcUSecPeriod(uint32_t hz) {
    if (hz == 0) hz = 1;  // Avoid Zero hz. Divide by Zero is nasty.
    uint32_t period =
    (1000000UL + hz / 2) / hz;  // The equiv of round(1000000/hz).
    // Apply the offset and ensure we don't result in a <= 0 value.
    //  if (use_offset)
    //    return std::max((uint32_t)1, period + periodOffset);
    //  else
    return std::max((uint32_t)1, period);
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
    uint64_t sdata = data;
    if (nbits > sizeof(sdata) * 8) return;  // We can't send something that big.
    bool skipSpace = true;
    bool field_bit = true;
    // Set 36kHz IR carrier frequency & a 1/4 (25%) duty cycle.
    enableIROut(36);

    // Binary Left Shift Operator.
    // The left operands value is moved left
    // by the number of bits specified by the right operand.
    sdata = sdata << (32 - nbits);

    IRtimer usecTimer = IRtimer();
//    for (uint16_t i = 0; i <= repeat; i++) {
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
        if (sdata & mask) {  // 1
            space(kRc5T1);    // 1 is space, then mark.
            mark(kRc5T1);
        } else {         // 0
            mark(kRc5T1);  // 0 is mark, then space.
            space(kRc5T1);
        }
        // Footer
        space(std::max(kRc5MinGap, kRc5MinCommandLength - usecTimer.elapsed()));
//    }
}
