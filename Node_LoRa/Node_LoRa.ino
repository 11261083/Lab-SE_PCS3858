#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <ArduinoBLE.h>
#include <map>


/* Endereço I2C do display */
#define OLED_ADDR 0x3c

/* distancia, em pixels, de cada linha em relacao ao topo do display */
#define OLED_LINE1 0
#define OLED_LINE2 10
#define OLED_LINE3 20
#define OLED_LINE4 30
#define OLED_LINE5 40
#define OLED_LINE6 50

/* Configuração da resolucao do display (este modulo possui display 128x64) */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

/* Definicoes para comunicação com radio LoRa */
#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18

#define HIGH_GAIN_LORA     20  /* dBm */
#define BAND               915E6  /* 915MHz de frequencia */


/* Definicaco do Unique ID do dispositivo */
#define StopID              101
#define LineID_A            8012
#define LineID_B            8022
#define LineID_C            8032

/* Configuracao do BLE */
String dispositivoPresente = "0";
#define MAC_8012_1 "FF:FF:C1:0E:1E:60"
#define MAC_8022_1 "FF:FF:C1:0E:22:EC"
std::map<std::string, const char*> MAC_beacon;

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

/* objeto do display */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 16);

bool timestampConfigured = false;


enum PacketTypes {
    TIMESTAMP_PACKET = 1,
    BUSARRIVALDATA_PACKET = 2,
    BUSARRIVALDATA_TO_GATEWAY = 3,
    BUSPREDICTDATA_PACKET = 4,
    // Adicione mais tipos conforme necessário
};

struct MacToId {
    String mac;
    uint32_t id;
};

MacToId macMappings[] = {
    {"FF:FF:C1:0E:1E:60", 60},
    {"FF:FF:C1:0E:22:EC", 80},
    // Adicione mais mapeamentos conforme necessário
};

std::map<int, int> busIdToLineId = {
    {60, 8012},
    {80, 8022},
    // outros mapeamentos
};

typedef struct __attribute__((__packed__))
{
  uint32_t lineId;
  uint8_t packetType;
}LoRaPacketHeader;

struct TimestampPacket : public LoRaPacketHeader
{
  uint32_t timestamp;
} ;

struct BusArrivalDataPacket : public LoRaPacketHeader
{
  uint32_t stopId;
  uint32_t busId;
  uint32_t time;
};

struct BusPredictDataPacket : public LoRaPacketHeader
{
  uint32_t stopId;
  uint32_t busId;
  uint32_t predictTime;
};

#define MAX_HISTORY  8   //
struct BusInfo 
{
  uint32_t busId;
  uint32_t lineId;
  uint32_t stopId;
  uint32_t time;
  bool infoType; // estimativa: false, tempo de chegada: true;
};
BusInfo busHistory[MAX_HISTORY];
BusInfo busPredictions[MAX_HISTORY];
int currentHistoryIndex = 0;
int currentPredictionsIndex = 0;

struct BusInfoToBeDisplayed
{
  uint32_t lineId;
  uint32_t time;
};
BusInfoToBeDisplayed infoToDisplay[2];
BusInfoToBeDisplayed predictToDisplay[2];

int indexOfBusInfo(uint32_t busId)
 {
  for(int i = 0; i < MAX_HISTORY; i++)
  {
    if (busHistory[i].busId == busId)
      return i;
  }
  return -1;
}

uint32_t getIdFromMac(String macAddress) {
    int numMappings = sizeof(macMappings) / sizeof(MacToId);
    for (int i = 0; i < numMappings; i++) {
        if (macMappings[i].mac == macAddress) {
            return macMappings[i].id;
        }
    }
    return -1; // Retorna -1 se não encontrar
}

void busInfoInit()
{
  for(int i = 0; i < MAX_HISTORY; i++)
  {
    busHistory[i].busId = 0;
    busHistory[i].lineId = 0;
    busHistory[i].stopId = 0;
    busHistory[i].time = 0;
    busHistory[i].infoType = false;
  }
}

// Initialize LoRa connection
bool LoRa_init(void)
{
    bool status_init = false;
    Serial.println("[LoRa Node] Tentando iniciar comunicacao com o radio LoRa...");
    SPI.begin(SCK_LORA, MISO_LORA, MOSI_LORA, SS_PIN_LORA);
    LoRa.setPins(SS_PIN_LORA, RESET_PIN_LORA, LORA_DEFAULT_DIO0_PIN);
    
    if (!LoRa.begin(BAND)) 
    {
        Serial.println("[LoRa Node] Comunicacao com o radio LoRa falhou. Nova tentativa em 1 segundo...");        
        delay(1000);
        status_init = false;
    }
    else
    {
      /* Configura o ganho do receptor LoRa para 20dBm, o maior ganho possível (visando maior alcance possível) */ 
      LoRa.setTxPower(HIGH_GAIN_LORA); 
      Serial.println("[LoRa Node] Comunicacao com o radio LoRa ok");
      status_init = true;
    }

    return status_init;
}

// Initialize Display
void display_init()
{
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) 
    {
      Serial.println("[LoRa Node] Falha ao inicializar comunicacao com OLED");        
    }
    else
    {
      Serial.println("[LoRa Node] Comunicacao com OLED inicializada com sucesso");
    
      /* Limpa display e configura tamanho de fonte */
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
    }
}

void setRTCWithUnixTimestamp(time_t unixTimestamp) 
{
  struct tm timeinfo;
  gmtime_r(&unixTimestamp, &timeinfo); // Converte o timestamp para tm struct
  timeinfo.tm_isdst = 0; // Define o horário de verão como desconhecido

  // Configura o RTC com a estrutura tm
  timeval tv;
  tv.tv_sec = unixTimestamp;  // definindo os segundos
  tv.tv_usec = 0;             // e os microsegundos

  timezone tz;
  tz.tz_minuteswest = 180;  // 3 horas = 180 minutos
  tz.tz_dsttime = 0;

  settimeofday(&tv, &tz);

  display.clearDisplay();
  display.setCursor(0, OLED_LINE1);
  display.print("Configuração do timestamp recebida!");
  display.display();

  timestampConfigured = true;
}

void processTimestamp(TimestampPacket timestampData)
{
  uint32_t receivedTimestamp = timestampData.timestamp;
  
  setRTCWithUnixTimestamp(static_cast<time_t>(receivedTimestamp));
}

void PrintTime()  // funcao de teste, TODO: deletar depois
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Falha ao obter o tempo local");
    return;
  }

  Serial.print("Data e Hora: ");
  Serial.print(timeinfo.tm_year + 1900);  // os anos são contados a partir de 1900
  Serial.print('/');
  Serial.print(timeinfo.tm_mon + 1);     // meses vão de 0 a 11
  Serial.print('/');
  Serial.print(timeinfo.tm_mday);
  Serial.print(" ");
  Serial.print(timeinfo.tm_hour - 3);
  Serial.print(':');
  if (timeinfo.tm_min < 10) Serial.print('0'); // para garantir dois dígitos
  Serial.print(timeinfo.tm_min);
  Serial.print(':');
  if (timeinfo.tm_sec < 10) Serial.print('0'); // para garantir dois dígitos
  Serial.print(timeinfo.tm_sec);
  Serial.println();
}

void PrintBusHistory()
{
  int temp = currentHistoryIndex - 1;
  if(temp == -1)
  {
    temp = MAX_HISTORY - 1;
  }
  Serial.println("Bus History:");
  for(int i = 0; i < MAX_HISTORY; i++)
  {
    if(busHistory[temp].busId == 0)
    {
      temp--;
      if(temp == -1)
      {
        temp = MAX_HISTORY -1 ;
      }

      continue;
    }
    Serial.println("Bus ID: " + String(busHistory[temp].busId));
    Serial.println("Line ID: " + String(busHistory[temp].lineId));

    uint32_t unixTime = busHistory[temp].time;
    struct tm timeStruct;
    gmtime_r((const time_t *)&unixTime, &timeStruct);
    
    int currentHour = timeStruct.tm_hour - 3;
    if(currentHour < 0)
    {
      currentHour = currentHour + 24;
    }

    Serial.println("Arrival Time: " + String(currentHour) + " : " + String(timeStruct.tm_min) + " : " + String(timeStruct.tm_sec));

    temp--;
    if(temp == -1)
    {
      temp = MAX_HISTORY - 1;
    }
  }

  temp = currentPredictionsIndex - 1;
  if(temp == -1)
  {
    temp = MAX_HISTORY - 1;
  }
  Serial.println("Bus Predictions:");
  for(int i = 0; i < MAX_HISTORY; i++)
  {
    if(busPredictions[temp].busId == 0)
    {
      temp--;
      if(temp == -1)
      {
        temp = MAX_HISTORY -1 ;
      }

      continue;
    }
    Serial.println("Bus ID: " + String(busPredictions[temp].busId));
    Serial.println("Line ID: " + String(busPredictions[temp].lineId));

    uint32_t unixTime = busPredictions[temp].time;
    struct tm timeStruct;
    gmtime_r((const time_t *)&unixTime, &timeStruct);
    
    int currentHour = timeStruct.tm_hour - 3;
    if(currentHour < 0)
    {
      currentHour = currentHour + 24;
    }

    Serial.println("Prediction Time: " + String(currentHour) + " : " + String(timeStruct.tm_min) + " : " + String(timeStruct.tm_sec));

    temp--;
    if(temp == -1)
    {
      temp = MAX_HISTORY - 1;
    }
  }
}

void DisplayBusHistory()
{
  display.clearDisplay();   

  display.setCursor(0, OLED_LINE1);
  display.print("Arrived: ");

  display.setCursor(0, OLED_LINE2);
  display.print(infoToDisplay[0].lineId);
  uint32_t unixTime = infoToDisplay[0].time;
  struct tm timeStruct;
  gmtime_r((const time_t *)&unixTime, &timeStruct);
  int currentHour = timeStruct.tm_hour - 3;
  if(currentHour < 0)
  {
    currentHour = currentHour + 24;
  }
  display.print(" - ");
  display.print(String(currentHour) + " : " + String(timeStruct.tm_min) + " : " + String(timeStruct.tm_sec));

  display.setCursor(0, OLED_LINE3);
  display.print(infoToDisplay[1].lineId);
  unixTime = infoToDisplay[1].time;
  gmtime_r((const time_t *)&unixTime, &timeStruct);
  currentHour = timeStruct.tm_hour - 3;
  if(currentHour < 0)
  {
    currentHour = currentHour + 24;
  }
  display.print(" - ");
  display.print(String(currentHour) + " : " + String(timeStruct.tm_min) + " : " + String(timeStruct.tm_sec));

  display.setCursor(0, OLED_LINE4);
  display.print("Arrival Predictions");

  display.setCursor(0, OLED_LINE5);
  display.print(predictToDisplay[0].lineId);
  unixTime = predictToDisplay[0].time;
  gmtime_r((const time_t *)&unixTime, &timeStruct);
  currentHour = timeStruct.tm_hour - 3;
  if(currentHour < 0)
  {
    currentHour = currentHour + 24;
  }
  display.print(" - ");
  display.print(String(currentHour) + " : " + String(timeStruct.tm_min) + " : " + String(timeStruct.tm_sec));

  display.setCursor(0, OLED_LINE6);
  display.print(predictToDisplay[1].lineId);
  unixTime = predictToDisplay[1].time;
  gmtime_r((const time_t *)&unixTime, &timeStruct);
  currentHour = timeStruct.tm_hour - 3;
  if(currentHour < 0)
  {
    currentHour = currentHour + 24;
  }
  display.print(" - ");
  display.print(String(currentHour) + " : " + String(timeStruct.tm_min) + " : " + String(timeStruct.tm_sec));

  display.display();      
}

void processBusArrivalPacketData(BusPredictDataPacket predictData)
{
  if(predictData.stopId == StopID)     // OBS: as paradas de onibus numa linha terao seu StopId numa sequencia, logo os dados dos pontos da frente que nao seriam interessantes serao descartados
  {
    UpdatePredictionHistory(predictData, false);
  }
}

void UpdateBusHistory(BusArrivalDataPacket arrivalData, bool isArrival)
{
  BusInfo newInfo;
  newInfo.infoType = isArrival;
  newInfo.busId = arrivalData.busId;
  newInfo.stopId = arrivalData.stopId;
  newInfo.lineId = arrivalData.lineId;
  newInfo.time = arrivalData.time;

  if(isArrival)
  {
    int index = indexOfBusInfo(newInfo.busId);
    if(index == -1)
    {
      busHistory[currentHistoryIndex] = newInfo;
      currentHistoryIndex = (currentHistoryIndex + 1) % MAX_HISTORY;
    }
    else
    {
      busHistory[index] = newInfo;
    }
  }
  PrintBusHistory();
  

  infoToDisplay[1] = infoToDisplay[0];
  infoToDisplay[0].lineId = arrivalData.lineId;
  infoToDisplay[0].time = arrivalData.time;

  DisplayBusHistory();
}

void UpdatePredictionHistory(BusPredictDataPacket predictData, bool isArrival)  //TODO: pensar no caso de dados de onibus que AINDA ESTAO na memoria
{
  BusInfo newInfo;
  newInfo.infoType = isArrival;
  newInfo.busId = predictData.busId;
  newInfo.stopId = predictData.stopId;
  newInfo.lineId = predictData.lineId;
  newInfo.time = predictData.predictTime;

  if(!isArrival)
  {
    busPredictions[currentPredictionsIndex] = newInfo;
    currentPredictionsIndex = (currentPredictionsIndex + 1) % MAX_HISTORY;
  }


  PrintBusHistory();

  switch(busIdToLineId[predictData.busId])
    {
      case 8012:
        characteristic_8012_2.setValue(predictData.predictTime);
        break;

      case 8022:
        characteristic_8022_2.setValue(predictData.predictTime);
        break;

      default:
       break;
    }

  predictToDisplay[1] = predictToDisplay[0];
  predictToDisplay[0].lineId = predictData.lineId;
  predictToDisplay[0].time = predictData.predictTime;

  DisplayBusHistory();
}

void SendBusArrivalData(uint32_t busId)
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) 
    {
      Serial.println("Falha ao obter o tempo local");
      return;
    }

    time_t unix_timestamp = mktime(&timeinfo);

    switch(busIdToLineId[busId])
    {
      case 8012:
        characteristic_8012_1.setValue(static_cast<uint32_t>(unix_timestamp));
        break;

      case 8022:
        characteristic_8022_1.setValue(static_cast<uint32_t>(unix_timestamp));
        break;

      default:
       break;
    }

    BusArrivalDataPacket arrivalData;
    arrivalData.packetType = BUSARRIVALDATA_TO_GATEWAY;
    arrivalData.lineId = busIdToLineId[busId];
    arrivalData.busId = busId;
    arrivalData.stopId = StopID;
    arrivalData.time = static_cast<uint32_t>(unix_timestamp);

    UpdateBusHistory(arrivalData, true);

    LoRa.beginPacket();
    LoRa.write((unsigned char *)&arrivalData, sizeof(BusArrivalDataPacket));
    Serial.println(arrivalData.lineId);
    Serial.println(arrivalData.time);
    LoRa.endPacket();
}

void scanBLE() {
  bool findPeripheralDevice = false;
  for (const auto& pair : MAC_beacon) {
    findPeripheralDevice = BLE.scanForAddress(pair.second);
    BLEDevice peripheral = BLE.available();

    if (peripheral && findPeripheralDevice) {
      dispositivoPresente = pair.second;
    }
  }
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("iniciando");

  Wire.begin(4, 15);
  display_init();
  display.clearDisplay();    
  display.setCursor(0, OLED_LINE1);
  display.print("Aguarde...");
  display.display();

  MAC_beacon["8012_1"] = MAC_8012_1;
  MAC_beacon["8022_1"] = MAC_8022_1;
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


  //characteristic_8012_1.setValue(1702394562);
  characteristic_recv.setValue(false); // como padrão, setamos como false o recebimento dos dados

  BLE.advertise(); // Start advertising
  
  // tenta ate obter sucesso
  while(LoRa_init() == false);

  busInfoInit();
}

/* variaveis utilizados para simular onibus chegando, realiza o trabalho de um contador assincrono que permite nao utilizar delay no loop principal */
unsigned long previousMillis = 0;
const long interval = 100; // 100ms

void loop() 
{
  // put your main code here, to run repeatedly:

  // if statement principal para a leitura do pacote
  int packetSize = 0;
  packetSize = LoRa.parsePacket();
  if(packetSize) 
  {
    // Primeiro, lemos o cabeçalho para identificar o tipo de pacote
    LoRaPacketHeader header;
    LoRa.readBytes((uint8_t*)&header, sizeof(LoRaPacketHeader));
    Serial.println(header.lineId);
    Serial.println("Packcet Type: " + String(header.packetType));

    if(header.lineId == LineID_A || header.lineId == LineID_B || header.packetType == TIMESTAMP_PACKET)
    {
      // Com base no tipo, decidimos como ler o restante
      switch(header.packetType)
      {
        case TIMESTAMP_PACKET:
          TimestampPacket timestampData;
          // Copiamos o cabeçalho já lido
          memcpy(&timestampData, &header, sizeof(LoRaPacketHeader));
          // Lemos o restante do pacote
          LoRa.readBytes(((uint8_t*)&timestampData) + sizeof(LoRaPacketHeader), sizeof(TimestampPacket) - sizeof(LoRaPacketHeader));

          // Agora, timestampData contém os dados completos
          processTimestamp(timestampData);
          break;
        
        // colocar mais case statement conforme o necessario
        case BUSPREDICTDATA_PACKET:
          BusPredictDataPacket predictData;
          memcpy(&predictData, &header, sizeof(LoRaPacketHeader));
          LoRa.readBytes(((uint8_t*)&predictData) + sizeof(LoRaPacketHeader), sizeof(BusPredictDataPacket) - sizeof(LoRaPacketHeader));
          processBusArrivalPacketData(predictData);
          break;
          
        case BUSARRIVALDATA_TO_GATEWAY:
          break;

        default:
          Serial.println("Tipo de pacote desconhecido recebido.");
          break;
      }
    }
  }
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    scanBLE();
    if(dispositivoPresente != "0" && timestampConfigured)
    {
      uint32_t busId = getIdFromMac(dispositivoPresente);

      struct tm timeinfo;
      if (!getLocalTime(&timeinfo)) 
      {
        Serial.println("Falha ao obter o tempo local");
        return;
      }
      time_t unix_timestamp = mktime(&timeinfo);
      uint32_t timestamp = static_cast<uint32_t>(unix_timestamp) - 30; //20min * 60s = 1200s

      int index = indexOfBusInfo(busId);
      if(index == -1 || busHistory[index].time < timestamp)
      {
        SendBusArrivalData(busId); // TODO: atribuir bus ID dependendo do MAC do beacon
      }
      dispositivoPresente = "0";
    }
  }

  int dataInput;

  if (BLE.connected()) {
    characteristic_recv.readValue(dataInput);
    if (dataInput == 1) {
      Serial.print("Read value: ");
      Serial.println(dataInput);
      BLE.disconnect();
      characteristic_recv.setValue(false);
      Serial.println("Device disconnected");
    }    
  }

}
