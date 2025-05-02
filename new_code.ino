//Includes
//Motor
#include <PID_v1.h>
#include "driver/pcnt.h"

//State Indicator
#include <Adafruit_NeoPixel.h>

//Depth Sensor
#include "MS5837.h"
#include <Wire.h>

//Communication
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>




//Defines + Declares
//Macros
#define abs(x) ((x) < 0 ? -(x) : (x))

//States
#define SURFACING_AND_COMMUNICATING 0
#define COLLECTING_DATA 1
uint8_t mode = SURFACING_AND_COMMUNICATING;

//Communication
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
#define GET_DATA 0
#define INITIATE_PROFILE 1
#define SET_DESIRED_DEPTH 2
#define SET_ALLOWED_DEPTH_ERROR 3
#define SET_DATA_COLLECTION_TIME 4
#define SET_DATA_COLLECTION_INTERVAL 5

//Motor
#define M2 10
#define M1 9
#define ANALOG_SPEED_PIN A0

#define STOP 0
#define UP 1
#define DOWN -1

#define Kp_PISTON 0.1

//Rotary Controller
#define ENCODER_A_PIN GPIO_NUM_17  //Rotary encoder A signal (Labled A0)
#define ENCODER_B_PIN GPIO_NUM_16  //Rotary encoder B signal (Labled A1)
#define PCNT_UNIT PCNT_UNIT_0      //Using PCNT unit 0
int32_t trueCount;

//Depth Sensors
MS5837 sensor;
double depth;
double depth_data[64];
size_t depthWriteIndex;

//Depth PID
#define Kp_DEPTH 2
#define Ki_DEPTH 5
#define Kd_DEPTH 1
double desiredDepth = 2.5;  //meters
double targetCount;
PID depthPID(&depth, &targetCount, &desiredDepth, Kp_DEPTH, Ki_DEPTH, Kd_DEPTH, DIRECT);

//Hall Effect Sensors
#define HALL_EFFECT_TOP_PIN 5     //Top
#define HALL_EFFECT_BOTTOM_PIN 6  //Bottom

//Timing
//(milliseconds)
#define DEPTH_CHECK_INTERVAL 50
#define PISTON_CHECK_INTERVAL 100
uint32_t timeNow;

//Indicator LED
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

//Program Details
uint8_t dataCollectionTime = 45;     //seconds
uint8_t dataCollectionInterval = 5;  //seconds
float allowedDepthError = 0.5;       //meters
bool needsToCollectMoreData = true;
#define MAX_PROGRAM_DURATION 2.5  //Minutes
uint32_t programStartTime;

//State
#define COLLECTING_DATA_COLOR 0x02e33e             //Green
#define SURFACING_AND_COMMUNICATING_COLOR 0xe6d40e  //Yellow




void clearDepthData() {
  Serial.println("Clearing depth data!");
  memset(depth_data, 0, sizeof(depth_data));
  depthWriteIndex = 0;
}


void sendCurrentModeToClients() {
  // Notify all WebSocket clients of new mode (cause why not with multi-client support)
  for (AsyncWebSocketClient &client : ws.getClients()) {
    if (client.status() == WS_CONNECTED) {
      client.text(String("mode:") + String(mode));
    }
  }
}


void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t data_len) {
  Serial.println("Received WebSocketEvent!");
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("Connected to client!");
      break;

    case WS_EVT_DATA:
      {  //C is stupid and we need to make a new scope so that it doesn't get confused and think we might possibly skip the initilization of the command variable.
        if (data_len < 1) {
          Serial.println("No command given!");
          return;  //ensure that the payload contains at least a command
        }
        uint8_t command = data[0];  //read the first byte as a command.
        switch (command) {
          case GET_DATA:
            Serial.println("Reading and transmitting depth data!");
            //If we've gathered data, send it.
            if (needsToCollectMoreData == false) {
              String depthDataPayload = "depth_data:";
              for (size_t i = 0; i < depthWriteIndex; i++) {
                depthDataPayload += depth_data[i];
                if (i < depthWriteIndex - 1) {  //Don't add ", " to the last value
                  depthDataPayload += ", ";
                }
              }

              client->text(depthDataPayload);  //Send depth data to client
              delay(10);                       //Avoid Flooding the client
              Serial.println("Transmission Complete!");
            } else {
              Serial.println("Not enough data has been collected.");
            }
            break;

          case INITIATE_PROFILE:
            needsToCollectMoreData = true;  //Reset Data Collection
            clearDepthData();
            depth = 0;  //Keeps the last depth sample from carrying over into the new mission in rare scenarios with specific depth check and data collection intervals.
            mode = COLLECTING_DATA;
            sendCurrentModeToClients();
            programStartTime = millis();

            Serial.println("Initiating a new profile!");
            //Update Indicator LED
            pixels.fill(COLLECTING_DATA_COLOR);  //Green
            pixels.show();
            break;

          case SET_DESIRED_DEPTH:
            if (data_len >= 1 + sizeof(double)) {
              memcpy(&desiredDepth, data + 1, sizeof(double));  //Copy the data (skipping the command stored in the first byte) into the desiredDepth.
              Serial.printf("Set desiredDepth to %f\n", desiredDepth);
            } else {
              Serial.println("Invalid new desiredDepth!");
            }
            break;

          case SET_ALLOWED_DEPTH_ERROR:
            if (data_len >= 1 + sizeof(float)) {
              memcpy(&allowedDepthError, data + 1, sizeof(float));  //Copy the data (skipping the command stored in the first byte) into the allowedDepthError.
              Serial.printf("Set allowedDepthError to %f\n", allowedDepthError);
            } else {
              Serial.println("Invalid new allowedDepthError!");
            }
            break;

          case SET_DATA_COLLECTION_TIME:
            if (data_len >= 2) {
              dataCollectionTime = data[1];  //Copy the data (skipping the command stored in the first byte) into the dataCollectionTime.
              Serial.printf("Set dataCollectionTime to %u\n", dataCollectionTime);
            } else {
              Serial.println("Invalid new dataCollectionTime!");
            }
            break;

          case SET_DATA_COLLECTION_INTERVAL:
            if (data_len >= 2) {
              dataCollectionInterval = data[1];  //Copy the data (skipping the command stored in the first byte) into the dataCollectionInterval.
              Serial.printf("Set dataCollectionInterval to %u\n", dataCollectionInterval);
            } else {
              Serial.println("Invalid new dataCollectionInterval!");
            }
            break;

          default:
            Serial.println("Invalid Command!");
            break;
        }
        break;
      }

    case WS_EVT_DISCONNECT:
      Serial.println("Client disconnected");
      break;
  }
}


void drive(int8_t dir, uint8_t speed = 255) {
  static bool hallEffectTriggered = false;

  // Check the Hall Effect Sensors
  if (!digitalRead(HALL_EFFECT_TOP_PIN) && dir == UP) {
    Serial.println("Hall Effect Sensor Trip (Top)!");
    hallEffectTriggered = true;
    dir = STOP;
  } else if (!digitalRead(HALL_EFFECT_BOTTOM_PIN) && dir == DOWN) {
    Serial.println("Hall Effect Sensor Trip (Bottom)!");
    hallEffectTriggered = true;
    dir = STOP;
  } else {
    hallEffectTriggered = false;  // Reset the flag if no sensor is triggered
  }

  // If a Hall Effect sensor was triggered, prevent further movement in that direction
  if (hallEffectTriggered && dir != STOP) {
    return;
  }

  analogWrite(ANALOG_SPEED_PIN, speed);  //Change the speed, defaulting to full.

  switch (dir) {
    case (DOWN):  //Motor Drives Down Float Goes Up
      digitalWrite(M2, HIGH);
      digitalWrite(M1, LOW);
      break;
    case (UP):  //Motor Drives Up Float Goes Down
      digitalWrite(M2, LOW);
      digitalWrite(M1, HIGH);
      break;
    case (STOP):
      digitalWrite(M2, LOW);
      digitalWrite(M1, LOW);
      break;
  }
}


void update_piston_count() {
  static int16_t previousRegisterCount;
  static int16_t registerCount;

  if (!digitalRead(HALL_EFFECT_BOTTOM_PIN)) {
    trueCount = 0;
    Serial.println("Reset Count!");
    return;
  }

  previousRegisterCount = registerCount;
  pcnt_get_counter_value(PCNT_UNIT, &registerCount);

  int16_t diff = registerCount - previousRegisterCount;
  // If the jump is too big, adjust for wrap-around.
  if (diff < -16384) {
    diff += 65536;
  } else if (diff > 16384) {
    diff -= 65536;
  }
  trueCount += diff;
  Serial.println(trueCount);
}


void update_depth() {
  static uint32_t timeSinceLastDepthCheck;
  static uint32_t timeSinceLastDepthLog;

  //If the time elapsed since our last check is greater than our minimum interval
  if ((timeNow - timeSinceLastDepthCheck) > DEPTH_CHECK_INTERVAL) {
    depth = sensor.depth();              //Clock Depth Sensor
    timeSinceLastDepthCheck = millis();  //update last checked time
    // Serial.println(depth);
  }
  if ((timeNow - timeSinceLastDepthLog) > dataCollectionInterval * 1000) {  //Convert Seconds to Miliseconds
    depth_data[depthWriteIndex] = depth;                                    //Write at the next available index
    depthWriteIndex++;                                                      //Increment Write Index
    timeSinceLastDepthLog = millis();                                       //update last checked time
  }
}


void process_rotary_encoder() {
  static uint32_t timeSinceLastPistonCheck;

  if ((timeNow - timeSinceLastPistonCheck) > PISTON_CHECK_INTERVAL) {
    update_piston_count();                //Update
    timeSinceLastPistonCheck = millis();  //update last checked time
  }
}


void approach_target_piston_count() {
  int32_t optimalSpeed = Kp_PISTON * (static_cast<int32_t>(targetCount) - trueCount);
  uint8_t trueSpeed;


  //Clamp optimal speed
  if (abs(optimalSpeed) > 255) {
    trueSpeed = 255;
  } else {
    trueSpeed = abs(optimalSpeed);
  }

  if (optimalSpeed < 0) {
    drive(DOWN, trueSpeed);
  } else {
    drive(UP, trueSpeed);
  }
}


void evaluate_program() {
  if (timeNow - programStartTime > MAX_PROGRAM_DURATION * 60000) {  //convert minutes to milliseconds
    needsToCollectMoreData = false;                                 //abort
    return;
  }

  if (needsToCollectMoreData == false) {
    return;
  }

  uint16_t samplesNeeded = (dataCollectionTime / dataCollectionInterval) + 1;  //Integer division essentialy has builtin floor(), but we want to be safe so we add 1 to make it ceil()

  if (depthWriteIndex < samplesNeeded) {
    return;
  }
  for (size_t i = depthWriteIndex - samplesNeeded; i < depthWriteIndex; i++) {
    if (depth_data[i] < desiredDepth - allowedDepthError || depth_data[i] > desiredDepth + allowedDepthError) {
      return;
    }
  }

  needsToCollectMoreData = false;  //No more samples needed, and data is correct.
}



void setup() {
  //Communication
  Serial.begin(115200);  //start Serial Communication
  WiFi.softAP("JONA_Float");
  ws.onEvent(onWebSocketEvent);  //Call the onWebSocketEvent function, after a websocket event occurs.
  server.addHandler(&ws);        //Add the websocket as a handler
  server.begin();                //Start the server


  //Motor
  pinMode(M2, OUTPUT);
  pinMode(M1, OUTPUT);

  //Motor Controller
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_A_PIN;  // Count pulses on A
  pcnt_config.ctrl_gpio_num = ENCODER_B_PIN;   // Direction control on B
  pcnt_config.unit = PCNT_UNIT;
  pcnt_config.channel = PCNT_CHANNEL_0;

  // Increment on both rising and falling edges of A
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;

  // Control signal determines counting direction
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;  // Reverse direction if B is low
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;     // Keep counting direction if B is high

  // Initialize PCNT unit
  if (pcnt_unit_config(&pcnt_config) != ESP_OK) {
    Serial.println("PCNT FAILED TO INITILIZE!");
  }

  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

  //Enable Pins
  pinMode(ENCODER_A_PIN, INPUT);  //A = Count Pulses
  pinMode(ENCODER_B_PIN, INPUT);  //B = Direction Control


  //Depth Sensor
  Wire.begin();
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.init();
  sensor.setFluidDensity(998);  //Aprox Density of water (998 for pool water, 997 for freshwater, 1024 for saltwater)


  //Depth PID
  depthPID.SetMode(AUTOMATIC);  //Enable the PID


  //Hall Effect Sensors
  pinMode(HALL_EFFECT_TOP_PIN, INPUT_PULLUP);     //Top
  pinMode(HALL_EFFECT_BOTTOM_PIN, INPUT_PULLUP);  //Bottom


  pixels.begin();
  delay(100);
  pixels.fill(SURFACING_AND_COMMUNICATING);
  pixels.show();


  Serial.println("Finished Setup!");
}




void loop() {
  timeNow = millis();

  switch (mode) {
    case SURFACING_AND_COMMUNICATING:
      drive(DOWN);
      break;
    case COLLECTING_DATA:
      update_depth();
      evaluate_program();
      if (needsToCollectMoreData == true) {
        depthPID.Compute();  //Update PID
        process_rotary_encoder();
        approach_target_piston_count();
      } else {
        mode = SURFACING_AND_COMMUNICATING;
        sendCurrentModeToClients();

        //Update Indicator LED
        pixels.fill(SURFACING_AND_COMMUNICATING_COLOR);  //Yellow
        pixels.show();
      }
  }

  delay(10);
}