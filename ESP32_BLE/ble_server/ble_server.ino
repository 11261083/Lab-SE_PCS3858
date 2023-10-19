#include <Arduino_JSON.h>
#include <ArduinoBLE.h>

#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUIDr "beb5483e-36e1-4688-b7f5-ea07361b26a7"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

BLEService pService(SERVICE_UUID);
BLEIntCharacteristic pCharacteristic(CHARACTERISTIC_UUID, BLERead);
BLEIntCharacteristic rCharacteristic(CHARACTERISTIC_UUIDr, BLEWrite | BLENotify);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("BLE_ESP32_PCS3858_SERVER");
  BLE.setAdvertisedService(pService);

  pService.addCharacteristic(pCharacteristic);
  pService.addCharacteristic(rCharacteristic);
  BLE.addService(pService);

  pCharacteristic.setValue(13);
  BLE.advertise(); // Start advertising

  Serial.println("Characteristic defined! Now you can read it on your phone!");
}

void loop() {
  int dataInput;
  Serial.println("Loop start");

  if (BLE.connected()) {
    Serial.println("Device connected");
    rCharacteristic.readValue(dataInput);
    Serial.print("Read value: ");
    Serial.println(dataInput);
    if (dataInput != 0) {
      pCharacteristic.setValue(27);
      Serial.println(pCharacteristic.valueUpdated());
      Serial.println(rCharacteristic.valueUpdated());
    }    
  } else {
    Serial.println("Device not connected");
  }

  //BLE.poll();
  delay(2000);
}
