#include <Arduino_JSON.h>
#include <ArduinoBLE.h>

#define CHARACTERISTIC_UUID_8012_1 "beb5483e-36e1-4688-b7f5-ea073618012a"
#define CHARACTERISTIC_UUID_8012_2 "beb5483e-36e1-4688-b7f5-ea073618012b"
#define CHARACTERISTIC_UUID_8022_1 "beb5483e-36e1-4688-b7f5-ea073618022a"
#define CHARACTERISTIC_UUID_8022_2 "beb5483e-36e1-4688-b7f5-ea073618022b"
#define CHARACTERISTIC_UUID_8022_3 "beb5483e-36e1-4688-b7f5-ea073618022c"
#define CHARACTERISTIC_UUID_RECV "beb5483e-36e1-4688-b7f5-ea0736180fab2c"
#define SERVICE_UUID_8012 "4fafc201-1fb5-459e-8fcc-c5c9c3318012"
#define SERVICE_UUID_8022 "4fafc201-1fb5-459e-8fcc-c5c9c3318022"
#define SERVICE_UUID_RECV "4fafc201-1fb5-459e-8fcc-c5c9c32154ab"


BLEService service_8012(SERVICE_UUID_8012);
BLEService service_8022(SERVICE_UUID_8022);
BLEService service_recv(SERVICE_UUID_RECV);
BLEIntCharacteristic characteristic_8012_1(CHARACTERISTIC_UUID_8012_1, BLERead);
BLEIntCharacteristic characteristic_8012_2(CHARACTERISTIC_UUID_8012_2, BLERead);
BLEIntCharacteristic characteristic_8022_1(CHARACTERISTIC_UUID_8022_1, BLERead);
BLEIntCharacteristic characteristic_8022_2(CHARACTERISTIC_UUID_8022_2, BLERead);
BLEIntCharacteristic characteristic_8022_3(CHARACTERISTIC_UUID_8022_3, BLERead);
BLEBoolCharacteristic characteristic_recv(CHARACTERISTIC_UUID_RECV, BLEWrite | BLENotify);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("BLE_ESP32_PCS3858_SERVER");
  BLE.setAdvertisedService(service_8012);
  BLE.setAdvertisedService(service_8022);
  BLE.setAdvertisedService(service_recv);

  service_8012.addCharacteristic(characteristic_8012_1);
  service_8012.addCharacteristic(characteristic_8012_2);
  service_8022.addCharacteristic(characteristic_8022_1);
  service_8022.addCharacteristic(characteristic_8022_2);
  service_8022.addCharacteristic(characteristic_8022_3);
  service_recv.addCharacteristic(characteristic_recv);
  BLE.addService(service_8012);
  BLE.addService(service_8022);
  BLE.addService(service_recv);

  characteristic_8012_1.setValue(1200);
  characteristic_recv.setValue(false); // como padrão, setamos como false o recebimento dos dados

  BLE.advertise(); // Start advertising

  Serial.println("Characteristic defined! Now you can read it on your phone!");
}

void loop() {
  int dataInput;

  if (BLE.connected()) {
    Serial.println("Device connected");
    characteristic_recv.readValue(dataInput);
    if (dataInput == 1) {
      Serial.print("Read value: ");
      Serial.println(dataInput);
      BLE.disconnect();
      characteristic_recv.setValue(false);
      Serial.println("Device disconnected");
    }    
  } else {
    Serial.println("Device not connected");
  }

  //BLE.poll();
  delay(2000);
}
