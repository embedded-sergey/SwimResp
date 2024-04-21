////////////////////
// USER INTERFACE //
////////////////////

// Set the measurement and flush phases (in seconds)
unsigned long FLUSH = 20;
unsigned long MEASUREMENT = 5;

// Speed (e.g. cm/s) and length (in seconds) of increment steps 
// of the Ucrit protocol (typically, the have similar length)
float SPEED[] = {5, 10, 15, 17.5, 20, 
                 22.5, 25, 27.5, 30, 32.5, 
                 35, 37.5, 40, 45, 50}; 

unsigned long LENGTH[] = {4, 4, 4, 4, 4, 
                          4, 4, 4, 4, 4, 
                          4, 4, 4, 4, 4};

// Motor calibration (the arrays should have increasing values)
float in[]  = {5,50}; // in cm/s
float out[] = {15, 33}; // raw data: 0...255

/////////////////////////
// IMPLEMENTATION CODE //
/////////////////////////
const byte PICO_RX = 2;
const byte PICO_TX = 3;
const byte LED = 4;

// Pinout for L298n controlling swimm-tunnels #1 and #2 
const byte enA = 5; // PWM for a motor
const byte in1 = 6;
const byte in2 = 7;
const byte in3 = 8;
const byte in4 = 9;
const byte enB = 10; // PWM for a pump

const byte BUTTON_REVERSE = 10; // longpress to step down in Ucrit
const byte BUTTON_STOP = 11;    // longpress to step up in Ucrit

const byte I2C_CLOCK = A4; // display
const byte I2C_DATA = A5; // display

int PERIOD = 1;
int i = 0;


const byte button = A2;
const byte LED_TEST = A3; /// only fore testing *REMOVE
boolean LED_TEST_State = false; /// only fore testing *REMOVE
boolean buttonActive = false;
boolean longPressActive = false;
long buttonTimer = 0; /// MOVE TO the section 'Time variables'
long longPressTime = 3000; /// MOVE TO the section 'Time variables'

// Time variables
unsigned long previousTime_pump = 0;
unsigned long previousTime_measurement = 0;
unsigned long previousTime_motor = 0;

void setup(){
  pinMode(LED, OUTPUT);

  // Button setup
  pinMode(LED_TEST, OUTPUT);
  pinMode(button, INPUT);

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

  // Starter for a motor
  analogWrite(enA, 255);
  delay(20);
  
  Serial.begin(9600);
}

void loop(){
  pumpControl(); // to control a pump
  motorControl(); // to regulate motor speed
  //buttonEvents(); // to identify button actions
}

void pumpControl(){
  unsigned long currentTime_pump = millis();

  if(currentTime_pump - previousTime_pump <= FLUSH*1000UL){
   // set pin 3 OFF to turn off the relay,
    analogWrite(enB, 255);   // so pumps start working again.
    digitalWrite(LED, HIGH);
  }

  else if(currentTime_pump - previousTime_pump > FLUSH*1000UL && 
          currentTime_pump - previousTime_pump <= MEASUREMENT*1000UL 
                                                    + FLUSH*1000UL){
    digitalWrite(enB, 0); // set pin 3 ON to activate the relay,
                               // so pumps stop working
    // blinking LED during Measurement Phase
      if(currentTime_pump % 1000UL <= 800UL){
        digitalWrite(LED, LOW);
      }
      else{
        digitalWrite(LED, HIGH);
      }
  }

  else{
    previousTime_pump = currentTime_pump;
    PERIOD++;
  }
}

void motorControl(){
  // map actual values (cm/s) to raw (bits 0...255)
  float raw = FmultiMap(SPEED[i], in, out, sizeof in/sizeof *in);
  Serial.print(SPEED[i]);
  Serial.print(" cm/s \t");
  Serial.print(raw);
  Serial.println(" bits");

  // kind of for loop
  unsigned long currentTime_motor = millis();
  if(currentTime_motor - previousTime_motor <= LENGTH[i]*1000){
    analogWrite(enA, raw);
  }
  else{
    i++;
    previousTime_motor = currentTime_motor;
  }
  
  // Number of elements in an array
  if(i >= sizeof LENGTH/sizeof *LENGTH){
    i = 0;
  }
}

void buttonEvents(){
  if(digitalRead(button) == true){
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
      
    if((millis() - buttonTimer) > longPressTime && longPressActive == false){
      longPressActive == true;
      // CASE1: long press -> minimum value in the Ucrit protocol
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);  
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW); 
    }
    
    else{
      // CASE2: short press -> reverse motor for a several seconds
      digitalWrite(LED_TEST, true);
    }
    
  }
  else{
    if (buttonActive == true) {
      buttonActive = false;
    }
    
    if (longPressActive == true) {
      longPressActive = false;
    }
    else {
      // CASE3: no press -> Ucrit protocol is running normally
      digitalWrite(LED_TEST, false);
    }
  }
}

// Voltage values?
// Remove units
// connect to DAQ-PLX2
// form a nice table
// have a nice graph

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

/*for (int i = 30; i < 40; i++)
  {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(1000);
  } 
  
  // Decelerate from maximum speed to zero
  for (int i = 40; i >= 30; --i)
  {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(10);
  } 
 
  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
}*/

// unsigned long SPEED[] = {75, 60, 75, 70, 60, 235, 255};
// unsigned long LENGTH[] = {20, 10, 20, 30, 80, 20, 20};

/* REFERENCES:
1. L298n: https://dronebotworkshop.com/dc-motors-l298n-h-bridge/
2. Multimap: https://playground.arduino.cc/Main/MultiMap/
3. Buttons: https://www.instructables.com/Arduino-Dual-Function-Button-Long-PressShort-Press/
   (a mistake in schematics: the orange wire should be in line #5)
4. Event-based programming: ... //// ADD HERE!
*/
