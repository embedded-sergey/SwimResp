////////////////////
// USER INTERFACE //
////////////////////

// Type which method for switching between flush and measurement phases: 
// i.e. uncomment one of the two options below i.e (by default OPTION 1).

// OPTION 1: Set the constant measurement and flush phase periods (in seconds)
// String PHASE_SWITCH_METHOD = "constant_phase_period"; // (UN)COMMENT THIS LINE!
unsigned long FLUSH = 60;
unsigned long MEASUREMENT = 240;

// OPTION 2: Choose the % air saturation high threshold for DO to start 
//           measurement phase and low threshold to start flushing the tunnel
String PHASE_SWITCH_METHOD = "saturation_threshold"; // (UN)COMMENT THIS LINE!
const float LOW_SATURATION_THRESHOLD = 75;  // to trigger the flush pump ON
const float HIGH_SATURATION_THRESHOLD = 90;  // to trigger the flush pump OFF


// Flow velocity (in cm/s) and length (in seconds) of increment steps
// of the Ucrit protocol (typically, the have similar length)
float VELOCITY[] = {2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0};

unsigned int LENGTH[] = {300, 300, 300, 300, 300, 300, 300, 300, 300};

// Motor calibration
// Record the flow of die solution or neutrally buoyant particlesa using a 
// slow-motion camera at one motor speed starting from lowest posssible.
// Calculate flow velocity based on your record and add it in  the arrays 
// below with the corresponding PWM value for that  motor speed (0...255). 
// Repeat the process with other motor speeds to get at least three reference
//  points for motor calibration (more reference points -> better precision).

// Note that the values in an array must ascend progressively!
float in[]  = {0.8, 1.2, 3.2, 3.7, 4.3, 6.2}; // flow velocity in cm/s
float out[] = {15, 18, 25, 40, 80, 225}; // raw data: 0...255



/////////////////////////
// IMPLEMENTATION CODE //
/////////////////////////
#include <SoftwareSerial.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// pinout map
const byte PICO_RX = 2; // DO meter
const byte PICO_TX = 3; // DO meter
const byte LED = 4;
const byte enA = 5; // PWM for a motor
const byte in1 = 6; // motor
const byte in2 = 7; // motor
const byte in3 = 8; // pump
const byte in4 = 9; // pump
const byte enB = 10; // PWM for a pump
const byte BUTTON_GREEN = 11;
const byte BUTTON_RED = 12;
const byte I2C_CLOCK = A4; // display
const byte I2C_DATA = A5; // display

// display
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiAvrI2c oled;
unsigned long previousTime_display = 0;

// DO logger
SoftwareSerial mySerial(PICO_RX, PICO_TX); // RX, TX
char charArr[255];
char *strings[25];
char *ptr = NULL;
unsigned long previousTime_logger = 0;
float DO;
float SATURATION;
float TEMPERATURE;
float PRESSURE;
bool HIGH_SATURATION_REACHED = true; // flag to know when to stop flushing

// buttons
long buttonTimer = 0;
long buttonTime = 1000;
boolean buttonActive = false;
boolean longPressActive = false;
boolean button1Active = false;
boolean button2Active = false;
boolean reverseMotor1 = false; //motor IN1
boolean reverseMotor2 = true; //motor IN2

// motor & pump
int PERIOD = 1;
float TIMER;
int i = 0;
String PHASE = "M0", F, M, f = "F", m = "M";
unsigned long previousTime_pump = 0;
unsigned long previousTime_motor = 0;

// For autosaving
unsigned long lastSave;


void setup(){
  // Display
  #if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0
  oled.clear();
  oled.setFont(Adafruit5x7);

  // Button setup
  pinMode(BUTTON_GREEN, INPUT_PULLUP);
  pinMode(BUTTON_RED, INPUT_PULLUP);
  
  // LED for a pump
  pinMode(LED, OUTPUT);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  pinMode(LED, OUTPUT);

  // Kick starter for the motor
  analogWrite(enA, 255);
  delay(20);
  
  // For autosaving
  lastSave = millis(); // inital start

  Serial.begin(9600);
  Serial.println("CLEARRANGE,A,2,H,1000000"); // trick to get rid of AM-PM
  mySerial.begin(19200); // for data streaming from DO logger 
}


void loop(){
  pumpControl(); // to control a pump
  motorControl(); // to regulate motor speed
  buttonEvents(); // to identify button actions
  displayUpdate(); // to show motor information
  loggerStream(); // to receive data from logger
  autoSave(); // to save data in Excel every minute
}
 

void pumpControl(){
  if (PHASE_SWITCH_METHOD == "constant_phase_period"){
    unsigned long currentTime_pump = millis();
    if(currentTime_pump - previousTime_pump <= FLUSH*1000UL){
      // set pin 3 OFF to turn off the relay,
      F = f + PERIOD;
      analogWrite(enB, 255);   // so pumps start working again.
      digitalWrite(LED, HIGH);
      PHASE = F;
    }
    else if(currentTime_pump - previousTime_pump > FLUSH*1000UL && 
            currentTime_pump - previousTime_pump <= MEASUREMENT*1000UL 
                                                      + FLUSH*1000UL){
      digitalWrite(enB, 0); // set pin 3 ON to activate the relay,
                                // so pumps stop working
        // blinking LED during Measurement Phase
        M = m + PERIOD;
        if(currentTime_pump % 1000UL <= 800UL){
          digitalWrite(LED, LOW);
        }
        else{
          digitalWrite(LED, HIGH);
        }
        PHASE = M;
    }
    else{
      previousTime_pump = currentTime_pump;
      PERIOD++;
    }
  }

  else if(PHASE_SWITCH_METHOD == "saturation_threshold"){
    unsigned long currentTime_pump = millis();

    if((SATURATION <= LOW_SATURATION_THRESHOLD || HIGH_SATURATION_REACHED == false) && DO != 0){
      F = f + PERIOD;
      analogWrite(enB, 255);   // so pumps start working again.
      digitalWrite(LED, HIGH);
      String previous_phase = PHASE;
      PHASE = F;  
      if(PHASE[0] != previous_phase[0]){
        previousTime_pump = currentTime_pump;
      }
      if(SATURATION > HIGH_SATURATION_THRESHOLD){
        HIGH_SATURATION_REACHED = true;
      }
      else{
        HIGH_SATURATION_REACHED = false;
      }
    }
    else if((SATURATION > LOW_SATURATION_THRESHOLD && HIGH_SATURATION_REACHED == true) && DO != 0){
      digitalWrite(enB, 0); // set pin 3 ON to activate the relay,
                              // so pumps stop working
      // blinking LED during Measurement Phase
      M = m + PERIOD;
      if(currentTime_pump % 1000UL <= 800UL){
        digitalWrite(LED, LOW);
      }
      else{
        digitalWrite(LED, HIGH);
      }
      String previous_phase = PHASE;
      PHASE = M;
      if(PHASE[0] != previous_phase[0]){
        previousTime_pump = currentTime_pump;
        PERIOD++;
      }
    }
    else{
      Serial.println("SOMETHING IS WRONG HERE !!!");
    }
  }

  else{
    Serial.println("Choose the phase switch method in USER INTERFACE: lines 5-17");
  }
}


void motorControl(){
  // map actual values (cm/s) to raw (bits 0...255)
  float raw = FmultiMap(VELOCITY[i], in, out, sizeof in/sizeof *in);

  unsigned long currentTime_motor = millis();
  if(currentTime_motor - previousTime_motor <= LENGTH[i]*1000UL){
    analogWrite(enA, raw);
    TIMER = (currentTime_motor - previousTime_motor)/1000UL;
  }
  else{
    i++;
    previousTime_motor = currentTime_motor;
    TIMER = 0UL;
  }
  
  // Number of elements in an array
  if(i >= sizeof LENGTH/sizeof *LENGTH){
    i = 0;
  }
}


void buttonEvents(){
  // note, the logic is inversed here due to an internal pull-up resistor
  if (digitalRead(BUTTON_GREEN) != HIGH) {
		if (buttonActive == false) {
			buttonActive = true;
			buttonTimer = millis();
		}
    button1Active = true;
	}

	if (digitalRead(BUTTON_RED) != HIGH) {
		if (buttonActive == false) {
			buttonActive = true;
			buttonTimer = millis();
		}
		button2Active = true;
	}
  delay(50);

	if ((buttonActive == true) && (digitalRead(BUTTON_GREEN) != LOW) && (digitalRead(BUTTON_RED) != LOW)) {
		if(longPressActive == true) {
			longPressActive = false;
		} 
    else{
			if((button1Active == true) && (button2Active == true)) {
				Serial.println("Short press custom mode: programm it yourself");
        } 
      else if((button1Active == true) && (button2Active == false)) {
		  // CASE1: short press -> reverse motor for a several seconds
        unsigned long previousTime_motor = millis();
        TIMER = 0;
        i = i - 1;
        }
			else if((button1Active == false) && (button2Active == true)) {
		  // CASE2: short press -> reverse motor for a several seconds
        unsigned long previousTime_motor = millis();
        TIMER = 0;
        i = i + 1;
			}
      else{
      }
		}
		buttonActive = false;
		button1Active = false;
		button2Active = false;
	}

  if ((buttonActive == true) && (millis() - buttonTimer > buttonTime) && (longPressActive == false)) {
		longPressActive = true;
		if ((button1Active == true) && (button2Active == true)) {
			Serial.println("Long press custom mode: programm it yourself");
      }
		else if((button1Active == true) && (button2Active == false)) {
      // CASE3: long press -> reverse motor
      reverseMotor1 = !reverseMotor1; //motor IN1
      reverseMotor2 = !reverseMotor2; //motor IN2
      for (int j = VELOCITY[i]; j >= 0; j = j-10){  //soft reverse
          analogWrite(enA, j);
          delay(5);
        }
      digitalWrite(in1, reverseMotor1);
      digitalWrite(in2, reverseMotor2);

      analogWrite(enA, 123);
      delay(50);
      }
    else if((button1Active == false) && (button2Active == true)) {
      // CASE4: long press -> 2.5 hour pause
      for (int j = VELOCITY[i]; j >= VELOCITY[0]; j = j-1){  //soft reverse
        analogWrite(enA, j);
        delay(20);
        }
      i = 0;
      VELOCITY[0] = {5};
      LENGTH[0] = {9999};
      }
		else{
		}
	}
}


void displayUpdate() {
  unsigned long currentTime_display = millis();
  if(currentTime_display - previousTime_display > 1000UL){
    oled.setCursor(2,0);
    oled.set2X();
    oled.print("Velocity");
    if(reverseMotor1 == true){
      oled.setCursor(115,0);
      oled.print("R");
    }
    else{
      oled.setCursor(115,0);
      oled.print("   ");
    }

    oled.setCursor(2,3);
    oled.print((VELOCITY[i]));
    oled.setCursor(66,3);
    oled.print(" cm/s");

    oled.setCursor(2,6);
    oled.set1X();
    oled.print("NOW  ");
    oled.print(LENGTH[i]);
    oled.print("s   ");
    oled.setCursor(66,6);
    int percent_bar = (TIMER)/LENGTH[i]*100;
    int dash_bar = percent_bar/10;
    for(int k = 0; k <= dash_bar; k++){
      oled.print("#");
    }
    for(int j = dash_bar; j >= dash_bar && j < 10; j++){
      oled.print(" ");
    }

    oled.setCursor(2,7);
    oled.set1X();
    oled.print("NEXT ");
    oled.print(LENGTH[i+1]);
    oled.print("s   ");
    oled.setCursor(66,7);
    oled.print((VELOCITY[i+1]));
    oled.setCursor(102,7);
    oled.print("cm/s");

    previousTime_display = currentTime_display;
    }
  }


void loggerStream(){
  unsigned long currentTime_logger = millis();
  if(currentTime_logger - previousTime_logger > 1000UL){
    mySerial.println("MEA 1 7");

    int k = 0;
    while (mySerial.available() > 0) {
      int inByte = mySerial.read();
      Serial.write(inByte);
      charArr[k] = inByte;
      k++;
    }
    Serial.println();

    byte index = 0;
    ptr = strtok(charArr, " ");  // delimiters space and comma
    while (ptr != NULL)
    {
        strings[index] = ptr;
        index++;
        ptr = strtok(NULL, " ");
    }

    String a = (strings[5]); // (umol*32)/1000) -> mg/L
    DO = (32 * a.toFloat())/1000000;

    String b = (strings[7]); // %
    SATURATION = (b.toFloat())/1000;

    String c = (strings[8]); // C
    TEMPERATURE = (c.toFloat())/1000;

    String d = (strings[12]); // mbar
    PRESSURE = (d.toFloat());
    // quick fix for a randomly cut pressure value due to a buffer 
    // limitation in the UART communication (i.e. 64 bytes)
    if (PRESSURE > 500 && PRESSURE < 1500){
      PRESSURE = (d.toFloat());
    }
    else if (PRESSURE > 5000 && PRESSURE < 15000){
      PRESSURE = (d.toFloat())/10;
    }
    else if (PRESSURE > 50000 && PRESSURE < 150000){
      PRESSURE = (d.toFloat())/100;
    }
    else if (PRESSURE > 500000 && PRESSURE < 1500000){
      PRESSURE = (d.toFloat())/1000;
    }
    else{
    }

    // Parse to PLX-DAQ
    if (TEMPERATURE != 0 && DO != 0 && PHASE != "M0"){
      Serial.println((String) "DATA,DATE,TIME," + (String) PHASE + (",") + TEMPERATURE + (",")
                            + DO + (",") + (VELOCITY[i]) + (",") + PRESSURE + (",") + SATURATION);
    }

    previousTime_logger = currentTime_logger;
  }
}


void autoSave(){
  if (millis() - lastSave >= 60000) { // one minute auto save
    Serial.println((String) "SAVEWORKBOOK");
    lastSave = millis();
    //Serial.println((String) "DONE"); // flushing the serial port in Excel
  }
}


// Originally from: https://playground.arduino.cc/Main/MultiMap
float FmultiMap(float val, float * _in, float * _out, uint8_t size){
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / 
          (_in[pos] - _in[pos-1]) + _out[pos-1];
}


/* REFERENCES:
1. L298n: https://dronebotworkshop.com/dc-motors-l298n-h-bridge/
2. Multimap: https://playground.arduino.cc/Main/MultiMap/
3. Buttons: https://www.instructables.com/Arduino-Dual-Function-Button-Long-PressShort-Press/
   (a mistake in schematics: the orange wire should be in line #5)
4. Event-based programming: ... //// ADD HERE!
5. Display library, i.e. 'AvrI2c128x64.ino' example: https://github.com/greiman/SSD1306Ascii
6. Serial protocol for Pyroscience: ... //// ADD HERE!
7. SoftwareSerial Library documentation: https://docs.arduino.cc/learn/built-in-libraries/software-serial/ */
