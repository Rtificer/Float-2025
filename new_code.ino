#include <PID_v1.h>
#include "driver/pcnt.h"
#include <Adafruit_NeoPixel.h>
#include "MS5837.h"
#include <Wire.h>
#include <EEPROM.h>

//Macros
#define abs(x) ((x) < 0 ? -(x) : (x))

//States
#define COMMUNICATE 0
#define PROGRAM 1
uint8_t mode;

//Motor
#define M2 10
#define M1 9
#define ANALOG_SPEED_PIN A0

#define STOP 0
#define UP 1
#define DOWN -1

#define Kp_PISTON 0.2

//Rotary Controller
#define ENCODER_A_PIN GPIO_NUM_17  //Rotary encoder A signal (Labled A0)
#define ENCODER_B_PIN GPIO_NUM_16  //Rotary encoder B signal (Labled A1)
#define PCNT_UNIT PCNT_UNIT_0      //Using PCNT unit 0
int32_t trueCount;

//Depth Sensors
MS5837 sensor;
double depth;
#define EEPROM_SIZE 512
size_t writeIndex;

//Depth PID
#define Kp_DEPTH 2
#define Ki_DEPTH 5
#define Kd_DEPTH 1
double targetDepth;
double targetCount;
PID depthPID(&depth, &targetCount, &targetDepth, Kp_DEPTH, Ki_DEPTH, Kd_DEPTH, DIRECT);

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

//Program Details REPLACE WITH DATA EXCHANGED WITH THE SERVER
#define DATA_COLLECTION_TIME 45  //(seconds)
#define DESIRED_DEPTH 2.5        //Meters
#define ALLOWED_DEPTH_ERROR 0.5  //Meters
#define SAMPLES_NEEDED DATA_COLLECTION_TIME * 1000 / DEPTH_CHECK_INTERVAL
bool needsToCollectMoreData = true;
// #define MAX_PROGRAM_DURATION 2.5  //Minutes
// uint32_t programStartTime;




void drive(int8_t dir, uint8_t speed = 255) {
  //Check the Hall Effect Sensors
  if ((digitalRead(HALL_EFFECT_TOP_PIN) && dir == UP) || (digitalRead(HALL_EFFECT_BOTTOM_PIN) && dir == DOWN)) {
    dir = STOP;
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
}


void update_depth() {
  static uint32_t timeSinceLastDepthCheck;

  //If the time elapsed since our last check is greater than our minimum interval
  if ((timeNow - timeSinceLastDepthCheck) > DEPTH_CHECK_INTERVAL) {
    depth = sensor.depth();                          //Clock Depth Sensor
    EEPROM.put(writeIndex * sizeof(double), depth);  //Write at the next available index, (multiplied by the size of the data)
    EEPROM.commit();                                 //comit to EEPROM
    writeIndex++;                                    //Increment Write Index
    timeSinceLastDepthCheck = millis();              //update last checked time
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

  if (abs(optimalSpeed) > 255) {
    trueSpeed = 255;
  } else if (abs(optimalSpeed) < 0) {
    trueSpeed = 0;
  } else {
    trueSpeed = optimalSpeed;
  }

  if (optimalSpeed < 0) {
    drive(DOWN, trueSpeed);
  } else {
    drive(UP, trueSpeed);
  }
}


void evaluate_program() {
  // if (timeNow - programStartTime > MAX_PROGRAM_DURATION * 60000) {  //convert minutes to milliseconds
  //   return false;
  // }
  if (needsToCollectMoreData == false) {
    return;
  }

  if (writeIndex < SAMPLES_NEEDED) {
    return;
  }

  for (size_t i = writeIndex - SAMPLES_NEEDED; i < writeIndex; i++) {
    double data;
    EEPROM.get(i * sizeof(double), data);

    if (data < DESIRED_DEPTH - ALLOWED_DEPTH_ERROR || data > DESIRED_DEPTH + ALLOWED_DEPTH_ERROR) {
      return;
    }
  }

  needsToCollectMoreData = false;  //No more samples needed, and data is correct.
}




void setup() {
  Serial.begin(115200);  //start Serial Communication


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

  pcnt_config.counter_h_lim = 10000;   // Arbitrary upper limit
  pcnt_config.counter_l_lim = -10000;  // Arbitrary lower limit

  // Initialize PCNT unit
  pcnt_unit_config(&pcnt_config);

  // Enable PCNT counter
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

  //Depth Data
  if (EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM Initalized");
  }

  //Depth PID
  depthPID.SetMode(AUTOMATIC);  //Enable the PID

  //Hall Effect Sensors
  pinMode(HALL_EFFECT_TOP_PIN, INPUT);     //Top
  pinMode(HALL_EFFECT_BOTTOM_PIN, INPUT);  //Bottom


  //Indicator LED
  pixels.begin();
}





void loop() {
  timeNow = millis();

  switch (mode) {
    case COMMUNICATE:
      //Update Indicator LED
      pixels.fill(0xf2f204);  //Yellow
      pixels.show();
      break;
    case PROGRAM:
      // programStartTime = millis();

      //Update Indicator LED
      pixels.fill(0x0bc902);  //Green
      pixels.show();

      update_depth();
      evaluate_program();

      if (needsToCollectMoreData = true) {
        depthPID.Compute();  //Update PID
        process_rotary_encoder();
        approach_target_piston_count();
      } else {
        drive(UP);
        mode = COMMUNICATE;
      }
  }

  // put your main code here, to run repeatedly:

  delay(10);
}
