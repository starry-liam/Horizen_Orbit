#ifndef EEPROM_HPP
#define EEPROM_HPP

#include <Arduino.h>
#include <at24c256.h>

class EEPROM {
    private:
        AT24C256 eeprom;
        
        int address;
        int timeAddress;
        int data[4];
        int dataSize = 4;


    public:
            EEPROM() : eeprom(0x50) {}

            void writeData(int address, float value) {
                byte data[4];
                if (address + 4 > 255000) { // Assuming 32KB EEPROM
                    Serial.println("Error: EEPROM address out of bounds");
                    return; // Don't write beyond the end of EEPROM
                }
                 memcpy(data, &value, sizeof(value));
                eeprom.write(address, data, 4); // Write 4 bytes to EEPROM
            }
            float readData(int address) {
                float value;
                byte data[4];
                eeprom.read(address, data, 4); // Read 4 bytes from EEPROM
            
                memcpy(&value, data, sizeof(value));
                return value;
            }
            void clearEEPROM() {
                byte zeros[32] = {0}; // Buffer of 32 zeros
                 for (int i = 0; i < 100000; i += 32) { 
                 eeprom.write(i, zeros, 32); // Write 32 zeros at a time
                 // Yield to prevent watchdog timer reset
                Serial.println(i);
                if (readData(i) == 0)
                {
                     break;
                }
       
                delay(10);
                }
            }
            int test(float val) {
                writeData(0, val);
                if(readData(0) > 0) {
                    return 1;
                }
                else {
                    return 0;
                }
            }

};


#endif // EEPROM_HPP