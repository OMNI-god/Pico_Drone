#include "RCConfig.h"

RCConfig::RCConfig()
{
    for (auto &channel : channels)
    {
        channel = RCChannelConfig{};
    }

    // ------------------------------------------------------------
    // Primary flight controls
    // ------------------------------------------------------------

    // CH1 -> Roll
    channels[0].function = RCFunction::Roll;

    // CH2 -> Pitch
    channels[1].function = RCFunction::Pitch;

    // CH3 -> Throttle
    channels[2].function = RCFunction::Throttle;

    // CH4 -> Yaw
    channels[3].function = RCFunction::Yaw;

    // ------------------------------------------------------------
    // Switches
    // ------------------------------------------------------------

    // CH5 -> Arm
    channels[4].function = RCFunction::Arm;

    // CH6 -> Flight mode
    channels[5].function = RCFunction::FlightMode;

    // CH7 -> Beeper
    channels[6].function = RCFunction::Beeper;

    // CH8 -> Calibration
    channels[7].function = RCFunction::Calibration;

    // ------------------------------------------------------------
    // AUX channels
    // ------------------------------------------------------------

    channels[8].function = RCFunction::Aux1;
    channels[9].function = RCFunction::Aux2;
    channels[10].function = RCFunction::Aux3;
    channels[11].function = RCFunction::Aux4;
    channels[12].function = RCFunction::Aux5;
    channels[13].function = RCFunction::Aux6;
    channels[14].function = RCFunction::Aux7;
    channels[15].function = RCFunction::Aux8;
}