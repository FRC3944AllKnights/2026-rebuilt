#include "subsystems/LEDStrip.h"

LEDStrip::LEDStrip(int port, int length) : m_led(port) {
    m_led.SetLength(length);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void LEDStrip::setLEDs(char patternID) {
    // Set LEDs to a specific pattern based on input ID
    // patternID: character representing the desired pattern (e.g., 'Y' for yellow, 'B' for blue)
    frc::LEDPattern& pattern = patternOff; // Default pattern
    if (patternID == 'Y') {
        pattern = patternYellow;
    }
    else if (patternID == 'B') {
        pattern = patternBlue;
    }
    else if (patternID == 'W') {
        pattern = patternWhite;
    }
    pattern.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}