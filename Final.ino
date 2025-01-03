#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <Hash.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;
StaticJsonDocument<256> doc;
String docReply;

int currServoValues[6] = { 180, 60, 180, 80, 90, 90 };

struct ServoPins {
  int servoPin;
  int initPosition;
};

std::vector<ServoPins> servoPins = {
  { 0, 180 },
  { 1, 60 },
  { 2, 180 },
  { 3, 80 },
  { 4, 90 },
  { 5, 90 },
};

#define SERVOMIN 125  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 575  // this is the 'maximum' pulse length count (out of 4096)
#define USE_SERIAL Serial
// #define bt D8

const int flashButtonPin = 0; 

// server address, port
const char* host = "192.168.22.22";
const int port = 5000;

void initServo() {
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  // pwm.setPWM(0, 0, angleToPulse(120));  // sv1: 90-> 180 (phai sang trai, 180 chinh giua)
  // pwm.setPWM(3, 0, angleToPulse(80));   // sv2: 90-> ... (sau ra trước)
  // pwm.setPWM(4, 0, angleToPulse(180));  // sv3: 0-> 180 (ngẩng lên)
  // pwm.setPWM(7, 0, angleToPulse(60));   // sv4: 0 -> 180  (cúi thấp xuống)
  // pwm.setPWM(8, 0, angleToPulse(90));   // sv5: (90: ngang, 180: quay phải, 0: quay trái)
  // pwm.setPWM(11, 0, angleToPulse(50));  // sv6: 50 -> 120 (rộng -> hẹp)
  for (int i = 0; i < servoPins.size(); i++) {
    pwm.setPWM(servoPins[i].servoPin, 0, angleToPulse(servoPins[i].initPosition));
  }
}

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
  // Serial.print("Angle: ");
  // Serial.print(ang);
  // Serial.print(" pulse: ");
  // Serial.println(pulse);
  return pulse;
}

void setUpWifi() {
  pinMode(flashButtonPin, INPUT);

  WiFiManager wifiManager;

  if (!wifiManager.autoConnect("MyESP8266AP")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.reset();
    delay(1000);
  }
  Serial.println("Connected to Wi-Fi!");

  // server address, port and URL
  socketIO.begin(host, port, "/socket.io/?EIO=4");

  // event handler
  socketIO.onEvent(socketIOEvent);
}

void socketIOEvent(socketIOmessageType_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case sIOtype_DISCONNECT:
      USE_SERIAL.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      socketIO.sendEVENT(docReply);
      break;
    case sIOtype_EVENT:
      USE_SERIAL.printf("[IOc] get event: %s\n", payload);
      controller(payload);
      break;
    case sIOtype_ACK:
      USE_SERIAL.printf("[IOc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      USE_SERIAL.printf("[IOc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      USE_SERIAL.printf("[IOc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
  }
}

void controller(uint8_t* payload) {
  doc.clear();

  // Giải mã JSON
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  const char* command = doc[0];
  // Serial.println(command);
  if (!strcmp(command, "control")) {
    if (doc[1]["servoId"] >= 0 && doc[1]["servoId"] <= 6) {
      int id = doc[1]["servoId"];
      int angle = doc[1]["angle"];
      Serial.println(id);
      Serial.println(angle);
      // if (id == 1) {
      //   angle += 90;
      // }
      pwm.setPWM(id, 0, angleToPulse(angle));
    }
    // Serial.println("lenh control");
  } else {
    Serial.println("ko phai lenh control");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  JsonArray array = doc.to<JsonArray>();
  array.add("ESP-reply");
  JsonObject param1 = array.createNestedObject();
  JsonArray nestedArray = param1.createNestedArray("currServoValue"); // Key chứa mảng
  for (int i = 0; i < 6; i++) {
    nestedArray.add(currServoValues[i]);
  }
  serializeJson(doc, docReply);

  initServo();
  delay(10);
  setUpWifi();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(flashButtonPin) == LOW) {
    WiFiManager wifiManager;
    wifiManager.resetSettings();
    ESP.reset();
    Serial.println("WiFi settings reset");
    delay(1000);
  }

  socketIO.loop();
}
