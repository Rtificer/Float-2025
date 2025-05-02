// Float 2024-2025 - Miles Hilliard
// include the required libraries...
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "MS5837.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "SPIFFS.h"
#include <PID_v1.h>
#include "driver/pcnt.h"

// define eeprom parameters
#define EEPROM_SIZE 256
#define COUNTER_ADDR 0   // store where to write next data to
#define CURRENT_STATE 1  // store what state it is in for error recovery
#define DATA_START 2     // store data starting here and ending at end of EEPROM

// company number and float statistics
#define COMPANY_NUMBER "38"    // Company number to be sent to the client.
#define FLOAT_HEIGHT 61        // height of float in cm, used to convert depth from measuring at top to measuring at bottom

#define PIN_NEOPIXEL 8
#define BLINK_FREQ 300

#define M2 10
#define M1 9
//Hall Effect Sensors
#define HT 5 //Top
#define HB 6 //Bottom


#define UP 0
#define DOWN 1
#define STOP 2

#define SINK 0
#define FLOAT 1
#define WAIT 2
#define COMMUNICATE 3
#define HOME 4
#define MAINTAIN_DEPTH 5

#define TRIP 0
#define DEPTH_INTERVAL 50
#define PISTON_INTERVAL 200
#define WAIT_INTERVAL 15000

//Rotary Controller
#define ENCODER_A_PIN GPIO_NUM_17  //Rotary encoder A signal (Labled A0)
#define ENCODER_B_PIN GPIO_NUM_16  //Rotary encoder B signal (Labled A1)
#define PCNT_UNIT PCNT_UNIT_0      //Using PCNT unit 0

//Depth PID defines
#define TARGET_DEPTH 200
#define Kp_DEPTH 2
#define Ki_DEPTH 5
#define Kd_DEPTH 1
#define COUNT_ERROR_THRESHOLD 8000

//Piston P defines
#define Kp_PISTON 0.2 //keep positive
//absolute value macro
#define abs(x) ((x) < 0 ? -(x) : (x))

uint8_t state;
uint32_t time_now;
uint32_t depth_check;
uint32_t piston_check;
uint32_t wait_check;
uint8_t depth_counter;
bool recovering_from_error = false;
unsigned long time_since_start;
unsigned long past_blink_time;

//Rotary Controller stuff
int16_t previousRegisterCount = 0;
int16_t registerCount = 0;
int32_t trueCount = 0;

//PID stuff
double targetDepth = TARGET_DEPTH;
double PIDDepth;
double targetCount;
PID PIDController(&PIDDepth, &targetCount, &targetDepth, Kp_DEPTH, Ki_DEPTH, Kd_DEPTH, DIRECT);

const char* ssid = "sunk";          // put SSID here
const char* password = "sunksunk";  // put password here

const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <style>
    body { font-family: Arial, sans-serif; }
    .container { max-width: 600px; margin: auto; text-align: center; }
    .button { margin: 10px; padding: 10px 20px; font-size: 16px; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Sunk Robotics</h1>
    <button class="button" onclick="sendCommand('profile')">Profile</button>
    <button class="button" onclick="sendCommand('get_info')">Get Info</button>
    <button class="button" onclick="sendCommand('get_depth')">Get Depth</button>
    <button class="button" onclick="sendCommand('get_pressure')">Get Pressure</button>
    <button class="button" onclick="sendCommand('clear_data')">Clear Data</button>
    <button class="button" onclick="sendCommand('help')">Help</button>
  </div>
  <script>
    function sendCommand(cmd) {
      fetch('/' + cmd)
        .then(response => response.text())
        .then(data => alert(data));
    }
  </script>
</body>
</html>
)rawliteral";

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
MS5837 sensor;


WebServer webServer(80);

void drive(uint8_t dir) {
  analogWrite(A0, 255); //set motor to full speed
  switch (dir) {
    case(DOWN): //Motor Drives Down Float Goes Up
      digitalWrite(M2, HIGH);
      digitalWrite(M1, LOW);
      break;
    case(UP): //Motor Drives Up Float Goes Down
      digitalWrite(M2, LOW);
      digitalWrite(M1, HIGH);
      break;
    case(STOP):
      digitalWrite(M2, LOW);
      digitalWrite(M1, LOW);
      break;
  }
}

void drive_to_count(int32_t targetcount) {
  int32_t optimalSpeed = Kp_PISTON * (targetcount - trueCount);
  uint8_t trueSpeed;
  if (abs(optimalSpeed) > 255) {
    trueSpeed = 255;
  }
  else if (abs(optimalSpeed) < 0) {
    trueSpeed = 0;
  }
  else {
    trueSpeed = optimalSpeed;
  }
  if (optimalSpeed < 0) {
    drive(DOWN);
  }
  else {
    drive(UP);
  }
  analogWrite(A0, trueSpeed);
}

void update_rotary_count() {
  previousRegisterCount = registerCount;
  pcnt_get_counter_value(PCNT_UNIT, &registerCount);

  int16_t diff = registerCount - previousRegisterCount;
  // If the jump is too big, adjust for wrap-around.
  if(diff < -16384) {
      diff += 65536;
  } else if(diff > 16384) {
      diff -= 65536;
  }
  trueCount += diff;
}

void connect_to_wifi() {
  WiFi.setHostname("Sunk Robotics Float");
  WiFi.begin(ssid, password);

  // Wait some time to connect to wifi
  for (int i = 0; i < 15 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  //You can get IP address assigned to ESP32
}

void store_depth() {
  sensor.read();
  float raw_data = sensor.depth();  // get data in cm
  if (raw_data < 0) {
    raw_data = 0;
  }
  uint8_t data = uint8_t(((raw_data * 100) + FLOAT_HEIGHT) / 5);
  Serial.println(raw_data * 5);
  Serial.println(data);

  EEPROM.write(depth_counter, data);
  depth_counter++;
  EEPROM.write(COUNTER_ADDR, depth_counter);
  EEPROM.commit();
}

void handleRoot() {
  webServer.send(200, "text/html", htmlPage);
}

void profile() {
    drive(UP);
    state = SINK;
}

String get_depth() {
    String message = "[";
    depth_counter = EEPROM.read(COUNTER_ADDR);
    message = message + String(EEPROM.read(DATA_START) * 5);
    for (int i = DATA_START + 1; i < depth_counter; i++) {
      message = message + ",";
      message = message + String(EEPROM.read(i) * 5);
    }
    message = message + "]";

    return message;
}

String get_pressure() {
    String message = "[";
    depth_counter = EEPROM.read(COUNTER_ADDR);

    // EEPROM Stores depth, multiply by the density of freshwater and force of gravity in order to calculate the pressure.
    message = message + String(float((EEPROM.read(DATA_START) * 5)) * 997.0474 * 9.80665);
    for (int i = DATA_START + 1; i < depth_counter; i++) {
      message = message + ",";
      message = message + String(float((EEPROM.read(i) * 5)) * 997.0474 * 9.80665);
    }
    message = message + "]";
    return message;
}

void clear_data() {
    depth_counter = DATA_START;
    EEPROM.write(COUNTER_ADDR, depth_counter);
    EEPROM.commit();
}

void handleCommand(String cmd) {
  switch (cmd) {
    case("profile"):

      profile();
      
      String json_message = "{\"status\": \"ok\", \"message\": \"Going Down!\"}";
      webServer.send(200, "application/json", jsonResponse);  
      break;
    
    case("get_depth"):

      String json_message = "{\"status\": \"ok\", \"message\": \"" + get_depth() + "\"}";
      webServer.send(200, "application/json", jsonResponse);
      break;

    case("get_pressure"):

      String json_message = "{\"status\": \"ok\", \"message\": \"" + get_pressure() + "\"}";
      webServer.send(200, "application/json", jsonResponse);
      break;

    case("clear_data"):
      clear_data();
  
      String json_message = "{\"status\": \"ok\", \"message\": \"Data Cleared!\"}";
      webServer.send(200, "application/json", jsonResponse);
      break;
    
    case("get_info"):

      message = COMPANY_NUMBER + " | " + "Uptime (Seconds): " + String(millis() / 1000) + " | " + "Time Recording (Seconds): " + String(EEPROM.read(COUNTER_ADDR) * 5);
      String json_message = "{\"status\": \"ok\", \"message\": \"" + message + "\"}";
      webServer.send(200, "application/json", jsonResponse);
      break;

    case("help"):

      String helpMessage = "JENA - the Sunk Robotics vertically profiling float\nCommands:\n\t* profile - float will do a vertical profile\n\t* get_info - returns uptime and company number\n\t* get_depth - Send depth data stored in EEPROM in centimeters (accurate to 5cm)\n\t* get_pressure - return pressure data stored in EEPROM\n\t* clear_data - Clear data in EEPROM and collect fresh data\n\t* break - exit gracefully without profile\n\t* help - print this message";
      String json_message = "{\"status\": \"ok\", \"message\": \"" + helpMessage + "\"}";
      webServer.send(200, "application/json", jsonResponse);
      break;

    default:
      String json_message = "{\"status\": \"fail\", \"message\": \"Invalid Command.\"}";
      webServer.send(200, "application/json", jsonResponse);
      break;

  }
}

void setup() {
  Serial.begin(115200);
  //Motor
  pinMode(M2, OUTPUT);
  pinMode(M1, OUTPUT);
  //Hall Effect
  pinMode(HT, INPUT); //Top
  pinMode(HB, INPUT); //Bottom

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);

  if (EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM Initalized");
  }

  depth_counter = EEPROM.read(COUNTER_ADDR);

  pixels.begin();
  // Connect to wifi
  Wire.begin();

  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)

  time_since_start = millis();

  //Rotary Controller
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_A_PIN;  // Count pulses on A
  pcnt_config.ctrl_gpio_num = ENCODER_B_PIN;   // Direction control on B
  pcnt_config.unit = PCNT_UNIT;
  pcnt_config.channel = PCNT_CHANNEL_0;

  // Increment on both rising and falling edges of A
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;

  // Control signal determines counting direction
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse direction if B is low
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep counting direction if B is high

  pcnt_config.counter_h_lim = 10000; // Arbitrary upper limit
  pcnt_config.counter_l_lim = -10000; // Arbitrary lower limit

  // Initialize PCNT unit
  pcnt_unit_config(&pcnt_config);

  // Enable PCNT counter
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

  //Enable Pins
  pinMode(ENCODER_A_PIN, INPUT); //A = Count Pulses
  pinMode(ENCODER_B_PIN, INPUT); //B = Direction Control

  //Enable the PID
  depthPID.SetMode(AUTOMATIC);

  // if it crashes in a floating, sinking, waiting, maintaining depth resume
  // otherwise home
  switch (EEPROM.read(CURRENT_STATE)) {
    case SINK:
      recovering_from_error = true;
      state = SINK;
      drive(UP);
      depth_check = millis();
      piston_check = millis();
      break;
    case FLOAT:
      recovering_from_error = true;
      state = FLOAT;
      drive(DOWN);
      depth_check = millis();
      piston_check = millis();
      break;
    case WAIT:
      recovering_from_error = true;
      state = WAIT;
      depth_check = millis();
      piston_check = millis();
      drive(STOP);
      break;
    case MAINTAIN_DEPTH:
      recovering_from_error = true;
      state = MAINTAIN_DEPTH;
      depth_check = millis();
      piston_check = millis();
      break;
    default:
      recovering_from_error = false;
      connect_to_wifi();
      state = HOME;
      drive(DOWN);
      break;
  }

  webServer.on("/", handleRoot);
  webServer.onNotFound([]() {
    webServer.send(404, "text/plain", "404: Not found");
  });

  const char* commands[] = {"profile", "get_info", "get_depth", "get_pressure", "clear_data", "help"};
  for (auto cmd : commands) {
    webServer.on(String("/") + cmd, [cmd]() { handleCommand(cmd); });
  }

  webServer.begin();
}

void loop() {
  webServer.handleClient();

  if (EEPROM.read(CURRENT_STATE) != state) {
    EEPROM.write(CURRENT_STATE, state);
    EEPROM.commit();
  }

  time_now = millis();

  if (time_now - past_blink_time > BLINK_FREQ) {
    past_blink_time = time_now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
  }

  pixels.fill(0xFFFFF);
  pixels.show();
  switch (state) {
    case HOME:
      //Serial.println("Home");
      drive(DOWN);
      if (digitalRead(HB) == TRIP) {
        drive(STOP);
        state = COMMUNICATE;
      }
      break;
    case COMMUNICATE:
      if (recovering_from_error) {
        connect_to_wifi();
        recovering_from_error = false;
      }
      //Serial.println("Communicate");
      pixels.fill(0x0000FF);
      pixels.show();
      break;
    case FLOAT:
      //Serial.println("Float");
      if (time_now - depth_check >= DEPTH_INTERVAL) {
        //read sensor
        store_depth();
        depth_check = time_now;
      }
      if (digitalRead(HB) == TRIP) {
        drive(STOP);
        state = COMMUNICATE;
        for (int i = 0; i < 5000; i++) {
          if (time_now - depth_check >= DEPTH_INTERVAL) {
            //read sensor
            store_depth();
            depth_check = time_now;
          }
          delay(1);
        }
      }
      break;
    case SINK:
      //Serial.println("Sink");
      if (time_now - depth_check >= DEPTH_INTERVAL) {
        //read sensor
        store_depth();
        depth_check = time_now;
      }
      if (digitalRead(HT) == TRIP) {
        drive(STOP);
        wait_check = time_now;
        state = WAIT;
      }
      break;
    case WAIT:
      //Serial.println("Wait");
      if (time_now - depth_check >= DEPTH_INTERVAL) {
        //read sensor
        store_depth();
        depth_check = time_now;
      }
      if (time_now - wait_check >= WAIT_INTERVAL) {
        drive(DOWN);
        state = FLOAT;
      }
      break;
    case MAINTAIN_DEPTH:
    //Serial.println("Maintain Depth");
      if (time_now - depth_check >= DEPTH_INTERVAL) {
        //read sensor
        store_depth();
        depth_check = time_now;
      }
      if (time_now - piston_check >= PISTON_INTERVAL) {
        //read sensor
        update_rotary_count();
        piston_check = time_now;
      }

      //Update PIDDepth
      PIDDepth = EEPROM.read(EEPROM.read(COUNTER_ADDR));
      //Run PID
      depthPID.Compute();

      if (digitalRead(HT) == TRIP && (targetCount - trueCount) > 0) {
        drive(STOP);
        break;
      }
      if (digitalRead(HB) == TRIP && (targetCount - trueCount) < 0) {
        drive(STOP);
        break;
      }

      drive_to_count(targetCount);
  }
}
