#include "RCConfig.h"

RCConfig::RCConfig()
{
    // --------------------------------------------------------
    // Default channel mapping
    //
    // CH1 = Roll
    // CH2 = Pitch
    // CH3 = Throttle
    // CH4 = Yaw
    // CH5 = Arm
    // CH6 = Flight Mode
    //
    // CH7-CH16 = AUX
    // --------------------------------------------------------

    channels[0].function =
        RCFunction::Roll;

    channels[1].function =
        RCFunction::Pitch;

    channels[2].function =
        RCFunction::Throttle;

    channels[3].function =
        RCFunction::Yaw;

    channels[4].function =
        RCFunction::Arm;

    channels[5].function =
        RCFunction::FlightMode;

    channels[6].function =
        RCFunction::Aux1;

    channels[7].function =
        RCFunction::Aux2;

    channels[8].function =
        RCFunction::Aux3;

    channels[9].function =
        RCFunction::Aux4;

    channels[10].function =
        RCFunction::Aux5;

    channels[11].function =
        RCFunction::Aux6;

    channels[12].function =
        RCFunction::Aux7;

    channels[13].function =
        RCFunction::Aux8;

    channels[14].function =
        RCFunction::None;

    channels[15].function =
        RCFunction::None;
}