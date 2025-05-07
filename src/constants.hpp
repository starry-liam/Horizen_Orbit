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
                constexpr static int reactionWheelFwd = 6;
                constexpr static int reactionWheelRev = 5;

                constexpr static int led = 15;
                constexpr static int voltChecker = 20;

                constexpr static int sdCs = 0;
                constexpr static int epromWp = 9;

                constexpr static int ctrlPin = 17;
                constexpr static int userPin = 16;
        };

        struct avgReadings {
            public:
                constexpr static int AltAvgReadings = 9;
                constexpr static int VoltAvgReadings = 5;
        };

        struct reactionPID {
            public:
                constexpr static double kp = 1.0;
                constexpr static double ki = 0.0;
                constexpr static double kd = 0.0;
        };

        struct altimeter {
            public:
                constexpr static double seaLevelPressure = 1013.25;
        };
        struct thresh
        {
            public:
                constexpr static int buttonThreshold = 700; // Threshold for button press detection
        };
        
};
#endif // CONSTANTS_HPP
