#pragma once

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>

#include "Constants.h"

class LEDStrip {
public:
    LEDStrip(int port, int length);
    void setLEDs(char patternID);
    
private:
    frc::AddressableLED m_led;
    std::array<frc::AddressableLED::LEDData, ShooterConstants::kLength> m_ledBuffer;
    frc::LEDPattern patternWhite = frc::LEDPattern::Solid(frc::Color::kWhite);
    frc::LEDPattern patternYellow = frc::LEDPattern::Solid(frc::Color::kYellow);
    frc::LEDPattern patternBlue = frc::LEDPattern::Solid(frc::Color::kBlue);
    frc::LEDPattern patternOff = frc::LEDPattern::Solid(frc::Color::kBlack);
};