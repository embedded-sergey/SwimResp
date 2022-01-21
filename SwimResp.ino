////////////////////
// USER INTERFACE //
////////////////////
# include <Controllino.h>
// Set the measurement and flush phases (in seconds)
unsigned long FLUSH = 2;
unsigned long MEASUREMENT = 2;

// Speed (e.g. cm/s) and length (in seconds) of increment steps 
// of the Ucrit protocol (the arrays should have similar length)
float SPEED[] = {5, 10, 15, 17.5, 20, 
                 22.5, 25, 27.5, 30, 32.5, 
                 35, 37.5, 40, 45, 50}; 

unsigned long LENGTH[] = {4, 4, 4, 4, 4, 
                          4, 4, 4, 4, 4, 
                          4, 4, 4, 4, 4};

// Motor calibration (the arrays should have increasing values)
float in[]  = {5, 10, 15, 25, 35, 50}; // in cm/s
float out[] = {28, 38, 47, 98, 160, 255}; // raw data: 0...255

/////////////////////////
// IMPLEMENTATION CODE //
/////////////////////////

const byte LED = 13;
const byte RELAY = A3;
int PERIOD = 1;
int i = 0;

// Pinout for L298n controlling swimm-tunnels #1 and #2 
const byte enA = 6;
const byte in1 = 4;
const byte in2 = 5;
const byte enB = 9;
const byte in3 = 7;
const byte in4 = 8;

// Time variables
unsigned long previousTime_flush = 0;
unsigned long previousTime_measurement = 0;
unsigned long previousTime_motor = 0;

void setup(){
  pinMode(LED, OUTPUT);
  pinMode(RELAY, OUTPUT);

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

  // Starter for motors
  analogWrite(enA, 127); 
  analogWrite(enB, 127); 
  delay(20);
  
  Serial.begin(9600);
}

void loop(){
  flushControl(); // to control flush pumps
  motorControl(); // to regulate swim speed
  buttonEvents(); // to identify button actions
}

void flushControl(){
  unsigned long currentTime_flush = millis();

  if(currentTime_flush - previousTime_flush <= FLUSH*1000UL){
    digitalWrite(RELAY, LOW); // set pin 3 OFF to turn off the relay,
                              // so pumps start working again.
    digitalWrite(LED, HIGH);
  }

  else if(currentTime_flush - previousTime_flush > FLUSH*1000UL && 
          currentTime_flush - previousTime_flush <= MEASUREMENT*1000UL + FLUSH*1000UL){
    digitalWrite(RELAY, HIGH); // set pin 3 ON to activate the relay,
                               // so pumps stop working
    // blinking LED during Measurement Phase
      if(currentTime_flush % 1000UL <= 800UL){
        digitalWrite(LED, LOW);
      }
      else{
        digitalWrite(LED, HIGH);
      }
  }

  else{
    previousTime_flush = currentTime_flush;
    PERIOD++;
  }
}

void motorControl(){
  // map actual values (cm/s) to raw (bits 0...255)
  float raw = FmultiMap(SPEED[i], in, out, sizeof in/sizeof *in);
  Serial.print(sizeof in/sizeof *in);
  Serial.print("\t");
  Serial.print(SPEED[i]);
  Serial.print(" cm/s \t");
  Serial.print(raw);
  Serial.println(" bits");

  // kind of for loop
  unsigned long currentTime_motor = millis();
  if(currentTime_motor - previousTime_motor <= LENGTH[i]*1000){
    analogWrite(enA, raw);
    analogWrite(enB, raw);
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
  /*
  a. if a button pressed, the motor reverse for a couple of seconds
     and get back to the previous speed
  b. if a button pressed for long time (3 sec), then it will go back
     to the default initial speed
  c. check out the A5 and A6 buttons.
  */
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
