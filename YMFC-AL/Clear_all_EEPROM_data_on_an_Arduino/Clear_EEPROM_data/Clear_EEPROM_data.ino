#include <EEPROM.h>

void setup() {
    pinMode(13, OUTPUT);

    // Check if the EEPROM length is valid
    int eeprom_size = EEPROM.length();
    if (eeprom_size > 0) {
        // Clear the entire EEPROM
        for (int i = 0; i < eeprom_size; i++) {
            EEPROM.write(i, 0);  // Write 0 to each EEPROM location
        }
        Serial.begin(9600);
        Serial.println("EEPROM cleared successfully.");
    } else {
        Serial.begin(9600);
        Serial.println("No EEPROM detected on this device.");
    }

    digitalWrite(13, HIGH);  // Turn on the built-in LED
}

void loop() {
    // Nothing to do in the loop
}
