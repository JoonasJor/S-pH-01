//https://www.seeedstudio.com/RS485-pH-Sensor-S-pH-01A-p-4632.html
//https://files.seeedstudio.com/products/101990666/res/RS485%20&%200-2V%20pH%20Sensor%20(S-pH-01)%20-%20User%20Guide%20v2.0.pdf
//https://files.seeedstudio.com/products/101990666/res/pH_sensor_Calibration.pdf

//modbus esimerkki ph ja lämpö kysely
//master request: 01 04 0000 0002 71CB 
//slave response: 01 04 04 08C3 029E 8910
//temperature = (0x08*256+0xC3)/100=2243/100=22.43°C
//pH = (0x02*256+0x9E)/100=670/100=6.70pH

//AA    1 byte  Slave Address, 0-255
//0x03  1 byte  Function Code 3
//RRRR  2 byte  Starting Register Addr
//NNNN  2 byte  Quantity of Register to read
//CCCC  2 byte  CRC CHECKSUM

//AA      1 byte  Slave Address,0-255
//0x03    1 byte  Function Code 3
//MM      1 byte  Register Data Byte Count
//VV0,VV1 2 byte  Register Value (High8bits first)
//VV2,VV3 2 byte  Register Value (High8bits first)
//…       …       Register Value (High8bits first)
//CCCC    2 byte  CRC CHECKSUM

#include <SoftwareSerial.h>
#include "RunningAverage.h"

EspSoftwareSerial::UART ph_modbus;

byte ph_temperature_inquiry[8] = {0x0C, 0x04, 0x00, 0x00, 0x00, 0x02};

byte calibrate_ph4[8] = {0x0C, 0x06, 0x00, 0x30, 0x7F, 0xFF};
byte calibrate_ph7[8] = {0x0C, 0x06, 0x00, 0x31, 0x7F, 0xFF};
byte calibrate_ph10[8] = {0x0C, 0x06, 0x00, 0x32, 0x7F, 0xFF};

const int response_length = 10;

const int ph_rx_pin = 33;
const int ph_tx_pin = 27;
const int ph_de_re_pin = 12;
const int ph_analog_pin = 36;

const int calibrate_ph4_pin = 15;
const int calibrate_ph7_pin = 32;
const int calibrate_ph10_pin = 14;

const int running_avg_size = 50;
RunningAverage ph_running_avg(running_avg_size); // tähän laitetaan anturin rekisterin kautta saadut arvot
RunningAverage ph_running_avg_analog(running_avg_size); // ja tähän anturin analogisen ulostulon kautta saadut arvot

//crc checksum laskeminen
const uint16_t crc_poly = 0xA001;
const uint16_t crc_initial = 0xFFFF;

uint16_t calculateCRC(const uint8_t *data, size_t length) {
    uint16_t crc = crc_initial;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ crc_poly;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void setup()
{
  Serial.begin(115200);

  pinMode(ph_de_re_pin, OUTPUT);
  ph_modbus.begin(9600, SWSERIAL_8N1, ph_tx_pin, ph_rx_pin, false);
  if (!ph_modbus) { // tarkistetaan onnistuiko EspSoftwareSerial objektin luominen
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // jos ei onnistunut niin ei jatketa ohjelman ajoa
      delay(1000);
    }
  } 

  pinMode(calibrate_ph4_pin, INPUT_PULLDOWN);
  pinMode(calibrate_ph7_pin, INPUT_PULLDOWN);
  pinMode(calibrate_ph10_pin, INPUT_PULLDOWN);

  // lasketaan kyselyiden crc checksum arvot ja lisätään ne kyselyiden taulukoiden loppuun
  uint16_t crc = 0xFFFF;

  crc = calculateCRC(ph_temperature_inquiry, sizeof(ph_temperature_inquiry) - 2);
  ph_temperature_inquiry[6] = lowByte(crc);
  ph_temperature_inquiry[7] = highByte(crc);

  crc = calculateCRC(calibrate_ph4, sizeof(calibrate_ph4) - 2);
  calibrate_ph4[6] = lowByte(crc);
  calibrate_ph4[7] = highByte(crc);

  crc = calculateCRC(calibrate_ph7, sizeof(calibrate_ph7) - 2);
  calibrate_ph7[6] = lowByte(crc);
  calibrate_ph7[7] = highByte(crc);

  crc = calculateCRC(calibrate_ph10, sizeof(calibrate_ph10) - 2);
  calibrate_ph10[6] = lowByte(crc);
  calibrate_ph10[7] = highByte(crc);
}

void loop()
{
  digitalWrite(ph_de_re_pin, HIGH); // asetetaan UART <-> RS485 muunnin lähettävään tilaan (muunnin -> anturi)
  delay(10);
  ph_modbus.write(ph_temperature_inquiry, sizeof(ph_temperature_inquiry)); // lähetään ph/lämpötila kysely anturille
  digitalWrite(ph_de_re_pin, LOW); // asetetaan UART <-> RS485 muunnin vastaanottavaan tilaan (anturi -> muunnin)

  byte response[response_length];

  for (int i = 0; i < response_length; i++)
  {
    response[i] = ph_modbus.read(); // luetaan anturin vastaus taulukkoon
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();  

  // lasketaan vastauksesta lämpötila ja pH arvot
  float temperature = float((response[3]) * 256 + float(response[4])) / 100;
  float ph = float((response[5]) * 256 + float(response[6])) / 100;
  ph_running_avg.add(ph);

  // luetaan ja muunnetaan anturilta tuleva jännite pH arvoksi
  // tämä tapa ei näytä olevan yhtä tarkka kuin modbussilta tulevat arvot
  float output_voltage = -1; // 0-2V 
  output_voltage = analogRead(ph_analog_pin);
  float ph_analog = output_voltage * (3.3 / 4095) * 7;
  ph_running_avg_analog.add(ph_analog);

  Serial.print("pH: ");
  Serial.print(ph, 2);
  Serial.print("/");
  Serial.print(ph_analog, 2);
  Serial.print(" -- pH avg: ");
  Serial.print(ph_running_avg.getAverage());
  Serial.print("/");
  Serial.print(ph_running_avg_analog.getAverage());
  Serial.print(" -- temperature: ");
  Serial.print(temperature, 1);
  Serial.println("C");

  // anturin kalibrointi
  // pH 4
  if (digitalRead(calibrate_ph4_pin) == HIGH) // painonappi
  {
    delay(1000);
    digitalWrite(ph_de_re_pin, HIGH);
    delay(10);
    ph_modbus.write(calibrate_ph4, sizeof(calibrate_ph4));
    digitalWrite(ph_de_re_pin, LOW);

    byte ph4_response[response_length];
    Serial.print("\nResponse: ");
    for (int i = 0; i < response_length; i++)
    {
      ph4_response[i] = ph_modbus.read();
      Serial.print(ph4_response[i], HEX);
      Serial.print(" ");
    }
    // tähän voisi tehdä tarkistuksia että onnistuiko kalibrointi oikeasti
    Serial.print("\n\npH 4 calibrated");
    delay(1500);
  }
  // pH 7
  else if (digitalRead(calibrate_ph7_pin) == HIGH)
  {
    delay(1000);
    digitalWrite(ph_de_re_pin, HIGH);
    delay(10);
    ph_modbus.write(calibrate_ph7, sizeof(calibrate_ph7));
    digitalWrite(ph_de_re_pin, LOW);
  
    byte ph7_response[response_length];
    Serial.print("\nResponse: ");
    for (int i = 0; i < response_length; i++)
    {
      ph7_response[i] = ph_modbus.read();
      Serial.print(ph7_response[i], HEX);
      Serial.print(" ");
    }
    Serial.println("\n\npH 7 calibrated");
    delay(1500);
  }
  // pH 10
  else if (digitalRead(calibrate_ph10_pin) == HIGH)
  {
    delay(1000);
    digitalWrite(ph_de_re_pin, HIGH);
    delay(10);
    ph_modbus.write(calibrate_ph10, sizeof(calibrate_ph10));
    digitalWrite(ph_de_re_pin, LOW);

    byte ph10_response[response_length];
    Serial.print("\nResponse: ");
    for (int i = 0; i < response_length; i++)
    {
      ph10_response[i] = ph_modbus.read();
      Serial.print(ph10_response[i], HEX);
      Serial.print(" ");
    }
    Serial.println("\n\npH 10 calibrated");
    delay(1500);
  }
  delay(250);
}