#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

namespace Constants {
        struct Serial {
            public:
                static constexpr int serial_baud = 115200;
                static constexpr int serial1_baud = 115200;
        };
        struct Pins {
            public:
                constexpr static int reactionWheelFwd = 16;
                constexpr static int reactionWheelRev = 17;

                constexpr static int led = 15;
                constexpr static int voltChecker = 0;

                constexpr static int sdCs = 0;
                constexpr static int epromWp = 0;
        };

        struct avgReadings {
            public:
                constexpr static int AltAvgReadings = 9;
                constexpr static int VoltAvgReadings = 5;
        };

        struct reactionPID {
            public:
                constexpr static double kp = 0.25;
                constexpr static double ki = 0.0;
                constexpr static double kd = 0.0;
        };

        struct altimeter {
            public:
                constexpr static double seaLevelPressure = 1013.25;
        };
};
#endif // CONSTANTS_HPP
