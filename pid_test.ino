#include <Servo.h>
Servo myservo;

#define PIN_SERVO 10 
#define PIN_IR A0 

#define SEQ_SIZE 10
#define _ITERM_MAX 40
//
#define _INTERVAL_DIST 30
#define DELAY_MICROS  1500
float filtered_dist; 
float samples_num = 3;
//
// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100 
#define _DIST_MAX 410 
// Distance sensor
#define _DIST_ALPHA 0.35
 

// Servo range
#define _DUTY_MIN 1600   
#define _DUTY_NEU 1450     
#define _DUTY_MAX 1300     


// Servo speed control
#define _SERVO_ANGLE 30.0  
#define _SERVO_SPEED 500.0

// Event periods
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 2
#define a 69.24
#define b 385.28
#define _KD 65
#define _KI 0.005
// Distance sensor
float dist_target; 
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//
//
float under_noise_filter(void){ 
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance_sequence();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float ir_distance_filtered(void){ 
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  //
  dist_ema = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*dist_ema;
  return dist_ema;
}
//
//
float ir_distance_sequence(void){
  float value, real_value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;

   // sensor values
  float x[SEQ_SIZE] = {88.07, 141.77, 179.55, 234.88, 288.25, 337.77, 385.28, 1000.0};
  float y[SEQ_SIZE] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 8000.0};

 
  int s = 0, e = SEQ_SIZE - 1, m;
  // binary search
  while(s <= e){
    m = (s + e) / 2;
    if(value < x[m]) {
      e = m - 1;}
    else if(value > x[m+1]){
      s = m + 1;}
    else{
      break;}
  } 
   if(s > e){
    if(value > 2000.0 || value < -2000.0) real_value = _DIST_TARGET;
    else if(s == 0) real_value = _DIST_MIN; 
    else real_value = _DIST_MAX;}


  // calculate real values
  real_value = (y[m+1] - y[m]) / (x[m+1] - x[m] ) * (value - x[m]) + y[m];
  return real_value;}

void setup() {
// initialize GPIO pins for LED and attach servo 
myservo.attach(PIN_SERVO); // attach servo

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);
duty_curr = _DUTY_NEU;

// initialize serial port
Serial.begin(57600);

pterm = 0;
iterm = 0;
dterm = 0;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / 180 ) * (_INTERVAL_SERVO / 1000.0);
}
  

void loop() {

unsigned long time_curr = millis();
if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
}

if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
}

if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
}

if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
     dist_ema = ir_distance_filtered();

  // PID control logic
    error_curr = dist_ema - _DIST_TARGET;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = -(pterm + dterm + iterm);
    
    duty_target = _DUTY_NEU + control;


  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    else if(duty_target < _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }

  }
  
  if(event_servo) {
    event_servo=false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
  duty_curr += duty_chg_per_interval;
  if(duty_curr > duty_target) duty_curr = duty_target;
     }
    else {
  duty_curr -= duty_chg_per_interval;
  if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
     error_prev = error_curr;
  }   


if(event_serial) {
  event_serial = false;
  Serial.print("IR:");
  Serial.print(dist_ema);
  Serial.print(",T:");
  Serial.print(255);
  Serial.print(",P:");
  Serial.print(map(pterm,-1000,1000,510,610));
  Serial.print(",D:");
  Serial.print(map(dterm,-1000,1000,510,610));
  Serial.print(",I:");
  Serial.print(map(iterm,-1000,1000,510,610));
  Serial.print(",DTT:");
  Serial.print(map(duty_target,1000,2000,410,510));
  Serial.print(",DTC:");
  Serial.print(map(duty_curr,1000,2000,410,510));
  Serial.println(",-G:245,+G:265,m:0,M:800");
}
} 
