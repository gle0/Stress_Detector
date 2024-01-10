#include <WiFi.h>              //Wifi library
#include <MQTT.h>              //IoT library
#include <PeakDetection.h>     //FC library
#include <Adafruit_GFX.h>      //display library
#include <Adafruit_SSD1306.h>  //display library
#include <Wire.h>              // I2C support library

#define OLED_WIDTH 128  // OLED display width, in pixels
#define OLED_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
#define N 20

//pin sensors
int FC = 32;
int GSR = 35;
//pin led
int LED = 26;

// Operating voltage
float Vcc = 3.3;

//delay for sampling of gsr sensor
int delay_gsr = 50;
//gsr sensor
float gsr = 0.0;
float voltage_GSR = 0.0;
float R_dito = 0.0;
float R_fix = 200000;  //200k ohm: two 100k ohm resitors in series.

float soglia_Relax;
float soglia_Stress;
float R_Relax;
float R_Stress;

String string;

int Ns = 2400;  // number of samples in order to have the two tasks of 2 min each. 2400*50=120000ms

//bpm sensor
float tb = 0; // time of last beat
float fc = 0.0;
int last_peak = 0;
float start;
float end;
int Nb = 30; // number of beats to update the BPM value.
int BPM = 0;
PeakDetection peakDetection;  // create PeakDetection object
int count = 0;

// publish time
float t = 0;

//wifi
const char* ssid = "OnePlusX"; // insert wifi username
const char* pass = "minimo8caratteri"; // insert wifi password

int status = WL_IDLE_STATUS;
WiFiClient net;
MQTTClient client;

//output subscriber
int flag;

//iot functions
void connect() {

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  while (!client.connect("arduino")) {
    delay(100);
  }
  client.subscribe("stressdetector/sub");
}

// the payload, that indicates the stress, controls the led light
void messageReceived(String& topic, String& payload) {
  Serial.println("Alert: " + topic + " - " + payload);
  if (payload == "0") {
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
  }
}
// function for printing on the OLED display
void newdisplay(String s, int j){
  display.clearDisplay();
  display.setCursor(1, 0);
  display.setTextSize(j);
  display.println(s);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.setRotation(2);
  display.setTextColor(WHITE);

  newdisplay("Hello, ESP32!",2);

  delay(5000);

  peakDetection.begin(20, 3, 0.6);  // sets the lag, threshold and influence
  
  pinMode(GSR, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(FC, INPUT);

  digitalWrite(LED, LOW);

  //initialization wifi and client
  WiFi.begin(ssid, pass);  // virtual wifi, for real ones use WiFi.begin("username", "passwd");
  client.begin("test.mosquitto.org", 1883, net);
  client.onMessage(messageReceived);
  
  //connection wifi and mqtt server
  connect();

  //start calibration
  Serial.print("calibrazione");
  newdisplay("CALIBRAZIONE",2);
  delay(10000);

  newdisplay("Concentrati sulla respirazione: 4in - 6out",1);

  for (int i = 0; i < Ns; i++) {
    gsr = gsr + analogRead(GSR);
        delay(delay_gsr);
   }
  soglia_Relax = gsr / Ns;
  gsr=0;

  newdisplay("Rest period",2); // rest period of 1 min
  delay(60000);

  newdisplay("Stress task",2);

  for (int i = 0; i < Ns; i++) {
   gsr = gsr + analogRead(GSR);
   delay(delay_gsr);
  }
  soglia_Stress = gsr / (Ns);
  gsr=0;

  // //converto il livello di soglia in r di soglia
  soglia_Relax = soglia_Relax * (Vcc / 4095.0);                    // 12 bit converter, we have 4095 levels
  R_Relax = (Vcc - soglia_Relax) / (1000 * soglia_Relax / R_fix);  //calculation of R having the gsr voltage (kohm)

  soglia_Stress = soglia_Stress * (Vcc / 4095.0);
  R_Stress = (Vcc - soglia_Stress) / (1000 * soglia_Stress / R_fix);  //calculation of R having the gsr voltage (kohm)

  newdisplay("FINE CALIBRAZIONE",2);
  Serial.println("fine calibrazione");
  delay(2000);

  newdisplay("R_soglia_Stress in Kohm :  "+String(R_Stress)+" \nR_soglia_Relax in Kohm :  "+String(R_Relax),1);
  Serial.println("R_soglia_Stress in Kohm :  "+String(R_Stress)+" \nR_soglia_Relax in Kohm :  "+String(R_Relax));
  delay(10000); // show the thresholds

  newdisplay("INIZIO MONITORAGGIO:",2);
  delay(2000);
}

void loop() {
  display.clearDisplay();

  for (uint8_t i = 0; i < N; i++) {
    gsr = gsr + analogRead(GSR);
    fc = fc + analogRead(FC);
    delay(1);
  }
  gsr = gsr / N;
  fc = fc / N;
  voltage_GSR = gsr * (Vcc / 4095.0);
  R_dito = (Vcc - voltage_GSR) / (1000 * voltage_GSR / R_fix);  //calculation of R_dito having the gsr voltage (kohm)

  double data = (double)fc; // reads the value of the sensor and converts to a range between -1 and 1
  peakDetection.add(data); // adds a new data point
  int peak = peakDetection.getPeak(); // returns 0, 1 or -1
  
  if (peak == 1 && last_peak == 0) {
    if (millis()-tb>300){
      count++;
      tb=millis();
      }
    if (count == 1) {
      start = millis();
      }
    else if (count == Nb) {
      end = millis();
      BPM = (count / (end - start)) * 60000;
      count = 0;
      // control on the BPM   
      Serial.print("BPM: "+ String(BPM)+ ", GSR (R_dito): " + String(R_dito));
    } 
  }
  //light control
      if (R_dito <= (R_Stress)) {
        flag=0; }// 0 means stress
      else {
        flag=1; }// 1 means relax

 if(millis()-t>10000) {
    client.loop();
    delay(1);  // <- fixes some issues with WiFi stability
    if (!client.connected()) {
      connect(); }
    client.publish("stressdetector", String(BPM) + "," + String(R_dito) + "," + String(flag));
    t=millis();
    newdisplay("BPM: "+ String(BPM)+ "\nGSR (R_dito): " + String(R_dito),1);
    Serial.println(String(BPM)+","+String(R_dito));
  }
  // reset values before the restart of the loop
  last_peak = peak;
  fc = 0;
  gsr = 0;
}