#include <PID_v1.h>
#include "driver/pcnt.h"
#include <Adafruit_NeoPixel.h>

//Macros
#define abs(x) ((x) < 0 ? -(x) : (x))

//States
#define IDLE 0
#define SINK 1
#define FLOAT 2
#define MAINTAIN_DEPTH 3

//Motor
#define M2 10
#define M1 9

#define STOP 0
#define UP 1
#define DOWN -1

#define Kp_PISTON 0.2

//Rotary Controller
#define ENCODER_A_PIN GPIO_NUM_17  //Rotary encoder A signal (Labled A0)
#define ENCODER_B_PIN GPIO_NUM_16  //Rotary encoder B signal (Labled A1)
#define PCNT_UNIT PCNT_UNIT_0      //Using PCNT unit 0
//Rotary Controller stuff
int16_t previousRegisterCount;
int16_t registerCount;
int32_t trueCount;

//Depth PID
#define Kp_DEPTH 2
#define Ki_DEPTH 5
#define Kd_DEPTH 1
double targetDepth;
double targetCount;
PID depthPID(&depth, &targetCount, &targetDepth, Kp_DEPTH, Ki_DEPTH, Kd_DEPTH, DIRECT);

//Hall Effect Sensors
#define HT 5 //Top
#define HB 6 //Bottom

//Depth Sensors
double depth;

//Misc
uint32_t timeNow;
uint32_t timeSinceLastDepthCheck;
uint32_t timeSinceLastPistonCheck;
uint32_t timeSinceLastDepthCheck;


//Indicator LED
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200); //start Serial Communication
  

  //Motor
  pinMode(M2, OUTPUT);
  pinMode(M1, OUTPUT);


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

  //Indicator LED
  pixels.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  pixels.fill(0xff1100);
  pixels.show();

  delay(100);
}
