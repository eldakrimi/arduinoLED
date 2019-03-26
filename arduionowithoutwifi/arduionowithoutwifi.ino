#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <Hash.h>
# include <QList.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

//#define WIFI true
#define WIFI false
//#define LEDs false
#define LEDs true
#define BAUD_RATE 9600
//#define BAUD_RATE 115200
//#define BAUD_RATE 4800

#define flag_body 0
#define flag_hand 0
#define flag_face 1
#define flag_face_deg 1
#define flag_age 0
#define flag_gend 0
#define flag_gaze 1
#define flag_blink 0
#define flag_expression 0
#define flag_ID 0
#define flag_image 0
#define HVC_SYNCBYTE 0xFE
#define HVC_SET_FACE 0x09
#define HVC_SET_FACE_THERSHOLD 0x05
#define HVC_READ_DATA 0x04
#define HVC_MESG_SIZE 2
#define HVC_DATA 0x00
#define HVC_PROTOCOL 1
#define HVC_FACE_POS 8
#define HVC_HAND_POS 7
#define HVC_BODY_POS 6
#define FILTER_MODE 0
#define HVC_NUM 2
#define COMMON_ANODE
#define PIN 2
#define NUM_LEDS 1
#define BRIGHTNESS 25
#define EYE_THRESHOLD 8


#ifdef LEDs
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRBW + NEO_KHZ800);
#endif

byte neopix_gamma[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

SoftwareSerial swSer(14, 12, false, 256); // RX, TX

byte msgInBuf[162];
byte msgInPos = 0;
uint32_t msgSize = 0;
byte msgSizeBytes[4];
bool msgStarted = false;
bool new_message = false;
int num_faces;
unsigned int t;
bool cam_waiting = false;
bool web_com[] = {false, false, false, false, false, false, false, false};
///********************//
String gestureArray[4];
bool gestureComplete = false;
int middleCounter = 0;
int upCounter = 0;
int downCounter = 0;
QList <int> gestureQueue;


bool CONNECTED = false;

byte read1 = flag_body + 2 * flag_hand + 4 * flag_face + 8 * flag_face_deg + 16 * flag_age + 32 * flag_gend + 64 * flag_gaze + 128 * flag_blink;
byte read2 = flag_expression + 2 * flag_ID;
byte com_setface[] = {0xFE, 0x09, 0x02, 0x00, 0x00, 0x00};
byte com_setTH[] = {0xFE, 0x05, 0x08, 0x00, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01};//set face detection threshold 500
byte com_read[] = {0xFE, 0x04, 0x03, 0x00, read1, read2, flag_image}; //command to read data from HVC-P2

struct FACE {//structure for detected face
  int pos[4];
  int deg[4];
  int age[2];
  char gender[2];
  int gaze[2];
  int blink_eyes[2];
  int expression[6];
  byte face_d[8];
  byte face_deg_d[8];
  byte face_age_d[3];
  byte face_gend_d[3];
  signed char face_gaze_d[2];
  byte face_blink_d[4];
  byte face_expression_d[6];
  signed char face_ID_d[4];
};

FACE face[8];//face structure array

StaticJsonBuffer<200> jsonBuffer;
StaticJsonBuffer<400> *myArray[10];

#define USE_SERIAL Serial

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

}

void setup() {
  USE_SERIAL.begin(115200);
  if (LEDs) {
    strip.setBrightness(BRIGHTNESS);
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    colorWipe(strip.Color(255, 0, 0)); // Red
    delay(100);
    colorWipe(strip.Color(0, 255, 0)); // Green
    delay(100);
    colorWipe(strip.Color(0, 0, 255)); // Blue
    delay(100);
    colorWipe(strip.Color(0, 0, 0, 255)); // None
  }

  USE_SERIAL.setDebugOutput(true);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  USE_SERIAL.println();

  for (uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
    yield();
  }

  swSer.begin(BAUD_RATE);
  delay(2000);
  swSer.write(com_setface, sizeof(com_setface));
  swSer.write(com_setTH, sizeof(com_setTH));

  for (int i = 0; i < 10; i++) myArray[i] = new StaticJsonBuffer<400>;

  t = millis();
}

void loop() {
  yield();

  if (((millis()) - t > 3000) && (!CONNECTED)) {
    swSer.write(com_read, sizeof(com_read));
    t = millis();
  }
  while (swSer.available())
  {
    update();
    yield();
  }
  if (new_message) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(10);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    JsonObject& root = jsonBuffer.createObject();
    JsonArray& face_json = root.createNestedArray("face");

    new_message = false;

    bool found_middle = false;
    bool found_up = false;
    bool found_down = false;

    for (int i = 0; i < num_faces; i++) {
      JsonObject& aux_json = myArray[i]->createObject();
      JsonObject& face_data = aux_json.createNestedObject("face");
      face_data["x"] = face[i].pos[0];
      face_data["y"] = face[i].pos[1];
      face_data["size"] = face[i].pos[2];
      face_data["confidence"] = face[i].pos[3];

      JsonObject& direction_ = aux_json.createNestedObject("direction");
      direction_["yaw"] = face[i].deg[0];
      int dirYaw = face[i].deg[0];
      direction_["pitch"] = face[i].deg[1];
      int dirPitch = face[i].deg[1];
      direction_["roll"] = face[i].deg[2];
      direction_["confidence"] = face[i].deg[3];

      JsonObject& gaze_ = aux_json.createNestedObject("gaze");
      gaze_["yaw"] = face[i].gaze[0];
      int yaw = face[i].gaze[0];
      gaze_["pitch"] = face[i].gaze[1];
      int pitch = face[i].gaze[1];


      //if gaze is in the middel - add 0
      if (yaw > -7 && yaw < 7 && pitch > -7 && pitch < 7) {
        gestureQueue.push_front(0);
      }
      else if (
        //if gaze is up to the right  - add 1
        yaw < -13 && pitch > 12) {
        gestureQueue.push_front(1);
      }
      else if (
        //If gaze is down to the left - add 2
        yaw > 8 && pitch < -8) {
        gestureQueue.push_front(2);
      }
      //If gestureQueue is too big, remove one. 
      if (gestureQueue.size() > 8)    {
        gestureQueue.pop_back();
      }



      printGestureQueueSize();

      //START forloop adding to gestueCounters
      for (int i = 0; i < gestureQueue.size(); i++) {
        if (gestureQueue[i] == 0) {
          middleCounter++;
        }else if(gestureQueue[i] == 1){
          upCounter++;
        }else if(gestureQueue[i] == 2){
          downCounter++;
        }

        if (middleCounter >= 4) {
          found_middle = true;
        } else if (upCounter >= 4) {
          found_up = true;
        }
        else if (downCounter >= 4) {
          found_down = true;
        }
      }
       //END forloop adding to gestureCounters


     printCounters(); 
     resetCounters();

      // no it changed the color of the the color of the LED
      face_json.add(aux_json);
      yield();
    }

    if (LEDs) {
      if (num_faces > 0) {
        if (found_middle) {
          // Grön, gaze in the middle
          colorWipe(strip.Color(0, 255, 0 ));
        }
        else if (found_up) {
          // Blue, gaze is up,right
          colorWipe(strip.Color(0, 0, 255));
        }
        else if (found_down) {
          // RED, gaze is down,left
          colorWipe(strip.Color(255, 0, 0));
        }
        else {
          //No LED-light if only face is detected
          colorWipe(strip.Color(0, 0, 0)); 
          gestureQueue.push_front(3);
        }
      }
      // No, LED if face is not detected
      else   colorWipe(strip.Color(0, 0, 0, 0));
    }
    String text_tx;
    root.printTo(text_tx);


    if (CONNECTED) {
    } else {
      swSer.write(com_read, sizeof(com_read));
      t = millis();
    }
    for (int i = 0; i < num_faces; i++) myArray[i]->clear();
    jsonBuffer.clear();
    USE_SERIAL.println(text_tx);
  }
}
void update()
{
  byte t = swSer.read();

  if (msgInPos > 162) {
    msgStarted = false;
    msgInPos = 0;
    msgSize = 0;
  }
  if (!msgStarted && (t == HVC_SYNCBYTE)) {
    msgStarted = true;
  }
  if (msgStarted) {
    msgInBuf[msgInPos] = t;
    if (msgInPos == HVC_MESG_SIZE) {
      msgSizeBytes[0] = t;
    } else if (msgInPos == HVC_MESG_SIZE + 1) {
      msgSizeBytes[1] = t;
    } else if (msgInPos == HVC_MESG_SIZE + 2) {
      msgSizeBytes[2] = t;
    } else if (msgInPos == HVC_MESG_SIZE + 3) {
      msgSizeBytes[3] = t;
      msgSize = (uint32_t) msgSizeBytes[3] << 24;
      msgSize |=  (uint32_t) msgSizeBytes[2] << 16;
      msgSize |= (uint32_t) msgSizeBytes[1] << 8;
      msgSize |= (uint32_t) msgSizeBytes[0];
      msgSize = msgSize + 6;
    }
    msgInPos++;
    if (msgInPos == msgSize) {
      switch (msgInBuf[HVC_PROTOCOL]) {
        case HVC_DATA: {
            num_faces = msgInBuf[HVC_FACE_POS];
            if (flag_face && msgInBuf[HVC_FACE_POS] > 0) {//face detection
              int message_pos = HVC_FACE_POS + 2;
              for (int z = 0; z < msgInBuf[HVC_FACE_POS]; z++) {
                if (flag_face) {
                  for (int j = 0; j < 8; j++) {
                    face[z].face_d[j] = msgInBuf[message_pos];
                    message_pos++;
                    face[z].pos[0] = (int)face[z].face_d[0] + 256 * (int)face[z].face_d[1];
                    face[z].pos[1] = (int)face[z].face_d[2] + 256 * (int)face[z].face_d[3];
                    face[z].pos[2] = (int)face[z].face_d[4] + 256 * (int)face[z].face_d[5];
                    face[z].pos[3] = (int)face[z].face_d[6] + 256 * (int)face[z].face_d[7];
                  }
                }
                if (flag_face_deg) {
                  for (int j = 0; j < 8; j++) {
                    face[z].face_deg_d[j] = msgInBuf[message_pos];
                    message_pos++;
                    face[z].deg[0] = (int)face[z].face_deg_d[0] + 256 * (int)face[z].face_deg_d[1];
                    face[z].deg[1] = (int)face[z].face_deg_d[2] + 256 * (int)face[z].face_deg_d[3];
                    face[z].deg[2] = (int)face[z].face_deg_d[4] + 256 * (int)face[z].face_deg_d[5];
                    face[z].deg[3] = (int)face[z].face_deg_d[6] + 256 * (int)face[z].face_deg_d[7];
                  }
                }
                if (flag_age) {
                  for (int j = 0; j < 3; j++) {
                    face[z].face_age_d[j] = msgInBuf[message_pos];
                    message_pos++;
                    face[z].age[0] = (int)face[z].face_age_d[0] + 256 * (int)face[z].face_age_d[1];
                    face[z].age[1] = (int)face[z].face_age_d[2] + 256 * (int)face[z].face_age_d[3];
                  }
                }
                if (flag_gend) {
                  for (int j = 0; j < 3; j++) {
                    face[z].face_gend_d[j] = msgInBuf[message_pos];
                    message_pos++;
                    face[z].gender[0] = (int)face[z].face_gend_d[0] + 256 * (int)face[z].face_gend_d[1];
                    face[z].gender[1] = (int)face[z].face_gend_d[2] + 256 * (int)face[z].face_gend_d[3];
                  }
                }
                if (flag_gaze) {
                  for (int j = 0; j < 2; j++) {
                    face[z].face_gaze_d[j] = msgInBuf[message_pos];
                    message_pos++;
                    face[z].gaze[0] = (int)face[z].face_gaze_d[0];
                    face[z].gaze[1] = (int)face[z].face_gaze_d[1];
                  }
                }
                if (flag_blink) {
                  for (int j = 0; j < 4; j++) {
                    face[z].face_blink_d[j] = msgInBuf[message_pos];
                    message_pos++;
                    face[z].blink_eyes[0] = (int)face[z].face_blink_d[0] + 256 * (int)face[z].face_blink_d[1];
                    face[z].blink_eyes[1] = (int)face[z].face_blink_d[2] + 256 * (int)face[z].face_blink_d[3];
                  }
                }
                if (flag_expression) {
                  for (int j = 0; j < 6; j++) {
                    face[z].face_expression_d[j] = msgInBuf[message_pos];
                    message_pos++;
                    face[z].expression[0] = (int)face[z].face_expression_d[0];
                    face[z].expression[1] = (int)face[z].face_expression_d[1];
                    face[z].expression[2] = (int)face[z].face_expression_d[2];
                    face[z].expression[3] = (int)face[z].face_expression_d[3];
                    face[z].expression[4] = (int)face[z].face_expression_d[4];
                    face[z].expression[5] = (int)face[z].face_expression_d[5];
                  }
                }
                if (flag_ID) {
                  for (int j = 0; j < 4; j++) {
                    face[z].face_ID_d[j] = msgInBuf[message_pos];
                    message_pos++;

                  }
                }
              }

            }
            break;
          }
        default: {
            //////unknownResponseCallback(message,messageSize);
          }
      }
      msgStarted = 0;
      msgInPos = 0;
      msgSize = 0;
      new_message = true;
    }
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
  }
}

void resetCounters() {
      middleCounter = 0;
      upCounter = 0;
      downCounter = 0;
 
}

void printCounters(){
  
      USE_SERIAL.print("middleCounter:  ");
      USE_SERIAL.print(middleCounter);
      USE_SERIAL.print("\t");
      USE_SERIAL.print("upCounter:  ");
      USE_SERIAL.print(upCounter);
      USE_SERIAL.print("\t");
      USE_SERIAL.print("downCounter:  ");
      USE_SERIAL.print(downCounter);
      USE_SERIAL.print("\t");
}

void printGestureQueueSize(){
      USE_SERIAL.print("Kö storlek:  ");
      USE_SERIAL.print(gestureQueue.size());
      USE_SERIAL.print("\t"); 
}
