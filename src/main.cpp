//_____________________________LIBRERIAS_______________________________________
#include <Arduino.h>
#include "user-variables.h"
#include <SPI.h>
#include <LoRa.h>    // https://github.com/sandeepmistry/arduino-LoRa
//#include <U8g2lib.h> // https://github.com/olikraus/U8g2_Arduino
// #include <U8x8lib.h>

#define OFF 0 // For LED
#define ON 1
#define SensorPin 38 //SENSOR

// SPI LoRa Radio
#define LORA_SCK 5   // GPIO5 - SX1276 SCK
#define LORA_MISO 19 // GPIO19 - SX1276 MISO
#define LORA_MOSI 27 // GPIO27 -  SX1276 MOSI
#define LORA_CS 18   // GPIO18 -   SX1276 CS
#define LORA_RST 14  // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

// I2C OLED Display works with SSD1306 driver
//#define OLED_SDA 4
//#define OLED_SCL 15
//#define OLED_RST 16

/* Pick One. Hardware I2C does NOT work! This article helped: https://robotzero.one/heltec-wifi-kit-32/
  TTGo boards similar to Heltec boards, LED_BUILTIN = 2 instead of pin 25
  Some OLED displays don't handle ACK correctly so SW I2C works better. Thank you Olikraus!
  TTGo OLED has pin 16 reset unlike other OLED displays
*/

// UNCOMMENT one of the constructor lines below
//U8X8_SSD1306_128X64_NONAME_SW_I2C Display(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Unbuffered, basic graphics, software I2C
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Page buffer, SW I2C
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/OLED_SCL, /* data=*/OLED_SDA, /* reset=*/OLED_RST); // Full framebuffer, SW I2C

const int blueLED = LED_BUILTIN;
bool confirmation = false;
int reenvio = 0;
float sensor1 = 17;
int waitCount = 0;
int waitLapse = 30000; //Cuanto tiempo esperamos la confirmación de recibido
String rssi = "";
bool packet = false;

void sendSensorValue();
void waitingResponse();
void readSensor();

// Start Subroutines
#include <go-to-deep-sleep.h>
//______________________________SET-UP_______________________________________
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("LoRa Sender");

  //Display.begin();
  //Display.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
  //Display.setFont(u8g2_font_ncenB10_tr);

  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  // Very important for LoRa Radio pin configuration!
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  pinMode(blueLED, OUTPUT); // For LED feedback

  if (!LoRa.begin(915E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  LoRa.setSpreadingFactor(12); //oscila entre 6-12, el valor predeterminado es 7 ver documentos de API
  // Cambia la potencia de transmisión de la radio
  // El valor predeterminado es LoRa.setTxPower (17, PA_OUTPUT_PA_BOOST_PIN);
  // La mayoría de los módulos tienen el pin de salida PA conectado a PA_BOOST, ganan 2-17
  // TTGO y algunos módulos están conectados a RFO_HF, ganancia 0-14
  // ¡Si su receptor RSSI es muy débil y poco afectado por una antena mejor, cambie esto!
  LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN);
}

//_______________________________LOOP_________________________________________
void loop()
{
  readSensor();
  sendSensorValue();

  if (confirmation)
  {
    Serial.println("LISTO, me voy a dormir un rato");
    //IR A DORMIR
    delay(1000);
    goToDeepSleep();
  }
  else
  {
    Serial.println("Vuelvo a intentar, quizá no escuchó");
    reenvio++;
    if (reenvio > 3)
    {
      Serial.println("Algo Pasó, intentaré luego");
      //IR A DORMIR
      delay(1000);
      goToDeepSleep();
    }
  }
}

//_____________________________LEER_SENSOR________________________________
void readSensor()
{
  uint8_t samples = 120;
  uint32_t hum = 1;
  uint16_t array[120];

  for (int i = 0; i < samples; i++)
  {
    array[i] = analogRead(SensorPin);
    Serial.println(array[i]);
    delay(5);
  }
  std::sort(array, array + samples);
  for (int i = 0; i < samples; i++)
  {
    if (i == 0 || i == samples - 1)
      continue;
    hum += array[i];
  }
  hum /= samples - 2;
  sensor1 = map(hum, 900, 4095, 100, 0);
  Serial.print("El valor del sensor es" + (String)sensor1);
}

//____________________ENVIA_VALOR_DEL_SENSOR_VIA_LORA_____________________
void sendSensorValue()
{
  Serial.print("Enviando...: ");
  Serial.println(sensor1);
  digitalWrite(blueLED, ON); // Turn blue LED on
 // sensor1 = random(1, 100);

  LoRa.beginPacket();
  LoRa.print(sensor1);
  LoRa.endPacket();
  digitalWrite(blueLED, OFF); // Turn blue LED off

  // Display Info
  //Display.clearBuffer();
  //Display.setCursor(0, 12);
  //Display.print("Enviando");
  //Display.setCursor(0, 30);
  //Display.print("Paquete:");
  //Display.setCursor(0, 48);
  //Display.print("value: " + (String)sensor1);
  //Display.sendBuffer();
  delay(5000);
  waitingResponse();
}

//____________________ESPERAR_CONFIRMACIÓN_DE_RECIBIDO_________________________
void waitingResponse()
{
  Serial.println("Esperando...");
  bool waiting = true;
  while (waiting)
  {
    Serial.println(millis());
    if (millis() - waitCount > waitLapse)
    {
      Serial.println("Ya pasó mucho tiempo");
      confirmation = false;
      waiting = false;
      waitCount = millis();
    }
    else
    {
      waitCount++;
      int packetSize = LoRa.parsePacket();
      if (packetSize)
      {
        Serial.println("Recibiendo confirmación...'");
        digitalWrite(blueLED, ON); // Turn blue LED on
        packet = false;            // Clear packet
        while (LoRa.available())
        {
          packet += (char)LoRa.read(); // Assemble new packet
        }
        rssi = LoRa.packetRssi();

        // Display Info
        //Display.clearBuffer();
        //Display.setCursor(0, 12);
        //Display.print("Confirmado");
        //Display.setCursor(0, 42);
        //Display.print("    '" + (String)packet + "'");
        //Display.setCursor(0, 58);
        //Display.print("RSSI " + rssi);
        //Display.sendBuffer();

        digitalWrite(blueLED, OFF); // Turn blue LED off
        Serial.println(packet + "' con RSSI " + rssi);

        confirmation = packet;
        waiting = false;
      }
    }
  }
}