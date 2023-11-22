#include <ArduinoBLE.h>
#include <iostream>
#include <map>
#include <stdio.h>

#define MAC_8012_1 "FF:FF:C1:0E:22:EC"
#define MAC_8012_2 "aa:bb:cc:ee:dd:ff"
#define MAC_8022_1 "aa:bb:cc:ee:dd:ff"
#define MAC_8022_2 "aa:bb:cc:ee:dd:ff"
#define MAC_8022_3 "aa:bb:cc:ee:dd:ff"

std::map<std::string, const char*> MAC;

void setup() {

  MAC["8012_1"] = MAC_8012_1;
  MAC["8012_2"] = MAC_8012_2;
  MAC["8022_1"] = MAC_8022_1;
  MAC["8022_2"] = MAC_8022_2;
  MAC["8022_3"] = MAC_8022_3;

  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
}

void loop() {
  bool findPeripheralDevice = false;
  char address[6];

  Serial.println("BLE Central scan");

  for (const auto& pair : MAC) {
      findPeripheralDevice = BLE.scanForAddress(pair.second);
      BLEDevice peripheral = BLE.available();

      if (peripheral) {
        Serial.println("Connecting ...");

        if (peripheral.connect()) {
          Serial.println("Connected");
          BLE.address().toCharArray(address, sizeof(address));
          Serial.print("Local address is: ");
          Serial.println(address);
          // definir o timestamp
          //peripheral.disconnect()
        } else {
          Serial.println("Failed to connect!");
          return;
        }
      }
  }

  delay(2000);
}
