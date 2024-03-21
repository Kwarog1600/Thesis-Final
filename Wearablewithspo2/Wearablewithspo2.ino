#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>

// Section for necessary Variables and Initialization
#define HES D3
#define Ready D7
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;
HTTPClient http;

bool WearControl;

const char* ssid = "Your Network SSID here";
const char* password = "Your Network Password";
const char* serverName = "http://192.168.1.3/DataInsert.php"; //Your Corresponding ip address for the sql database

typedef struct Control
{
    bool WearStatControl;
    bool dataCollect;
}
Control;

typedef struct dataMsg
{
    double temperature;
    int BPM;
    int SPO2;
    bool wearStat = true;
    const char* identifier = "a0:76:4e:45:56:30";
}
dataMsg;

Control ctrl;
dataMsg wearableData;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;
double temperature;

float Percentage;

void sendDataToPHP(dataMsg data)
{
    WiFiClient client;

    // Prepare your HTTP POST request data
    String httpRequestData = "temperature=" + String(data.temperature) +
                             "&pulseRate=" + String(data.BPM) +
                             "&spo2=" + String(data.SPO2) +
                             "&wearStatus=" + String(data.wearStat) +
                             "&DevID=" + String(data.identifier) +
                             "&Battery=" + String(Percentage);
    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);

    http.begin(client, serverName);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpResponseCode = http.POST(httpRequestData);

    if (httpResponseCode > 0)
    {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
    }
    else
    {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    http.end();
}

void calculateSPO2(uint32_t redValue, uint32_t irValue)
{
    float R = (float)redValue / irValue;
    wearableData.SPO2 = 104 - 17 * R;
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_AP_STA);

    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());

    http.begin(serverName);
    int httpCode = http.GET();
    if (httpCode > 0)
    {
        Serial.printf("Connected to server: %d\n", httpCode);
    }
    else
    {
        Serial.printf("Failed to connect to server: %d\n", httpCode);
    }
    http.end();
    while (!mlx.begin())
    {
        Serial.println("Error connecting to MLX sensor. Check wiring.");
        delay(250);
    }

    Serial.print("Emissivity = ");
    Serial.println(mlx.readEmissivity());
    Serial.println("================================================");

    while (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        delay(250);
    }
    Serial.println("Place your index finger on the sensor with steady pressure.");

    particleSensor.setup();
    particleSensor.enableDIETEMPRDY();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
}

void loop()
{
    uint32_t Vbatt = 0;

    for (int i = 0; i < 4; i++) {
      
      Vbatt += analogReadMilliVolts(A0);  
    }
    float Vbattf = Vbatt / 4;
    Serial.println(Vbattf);
float Percentage = ((Vbattf) / 3300) * 100; // Assuming 0 - 2600mV measurement range with 11 dB attenuation

    Serial.print("Battery= ");
Serial.print(Percentage, 2);
Serial.print(", HES= ");
Serial.println(analogRead(HES));
noHandReset:
    if (analogRead(HES) < 3000)
    {
      
        wearableData.wearStat = true;
        int i = 0;
        lastBeat = 0;
        beatsPerMinute = 0;
        beatAvg = 0;
        temperature = mlx.readObjectTempC();

        while (i < 4)
        {
            long irValue = particleSensor.getIR();
            if (irValue < 50000)
            {
              Serial.print("IR = ");
                Serial.print(irValue);
                Serial.println(" No finger?");
                goto noHandReset;
            }
            else
            {
                long irValue = particleSensor.getIR();
                if (checkForBeat(irValue) == true)
                {
                    if (lastBeat == 0)
                    {
                        lastBeat = millis();
                    }
                    else
                    {
                        i++;
                        long delta = millis() - lastBeat;
                        lastBeat = millis();
                        beatsPerMinute = 60 / (delta / 1000.0);
                        if (beatsPerMinute < 255 && beatsPerMinute > 20)
                        {
                            rates[rateSpot++] = (byte)beatsPerMinute;
                            rateSpot %= RATE_SIZE;
                            beatAvg = 0;
                            for (byte x = 0; x < RATE_SIZE; x++)
                                beatAvg += rates[x];
                                beatAvg /= RATE_SIZE;
                        }
                    }
                }
                Serial.print("IR = ");
                Serial.print(irValue);
                Serial.print(", BPM = ");
                Serial.print(beatsPerMinute);
                Serial.print(", Avg BPM = ");
                Serial.println(beatAvg);
            }
        }


        calculateSPO2(particleSensor.getRed(), particleSensor.getIR());
        Serial.print("SPO2 = ");
        Serial.println(wearableData.SPO2);
        Serial.print("Temperature = ");
        Serial.println(temperature);
        wearableData.temperature = temperature;
        wearableData.BPM = beatAvg;
        sendDataToPHP(wearableData);
        delay(2000);
    }
    else
    {
        wearableData.wearStat = false;
        wearableData.temperature = 0.0;
        wearableData.BPM = 0;
        wearableData.SPO2 = 0;
        sendDataToPHP(wearableData);
        delay(2000);
    }
}
