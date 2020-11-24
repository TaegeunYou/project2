#include <Servo.h>
Servo myservo;

#define PIN_SERVO 10 
#define PIN_IR A0 

#define SEQ_SIZE 8

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100 
#define _DIST_MAX 410 
// Distance sensor
#define _DIST_ALPHA 0.5 
 

// Servo range
#define _DUTY_MIN 1600   
#define _DUTY_NEU 1450     
#define _DUTY_MAX 1300     


// Servo speed control
#define _SERVO_ANGLE 30.0  
#define _SERVO_SPEED 500.0

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 0.05
#define a 87.83
#define b 321.60

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

float ir_distance(void){ // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (value    - a) + 100;
}
//[3099]

float ir_distance_filtered(void){ // return value unit: mm
  dist_raw = ir_distance();
  return _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
}
float ir_distance_sequence(void){
  float value, real_value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;

  // sensor values
float x[SEQ_SIZE] = {87.83, 141.29, 174.67, 206.24, 239.42, 274.77, 314.77};
float y[SEQ_SIZE] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 450.0};

 
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
    pterm = error_curr;
    iterm = 0;
    dterm = 0;
    control = - _KP * pterm + iterm + dterm;
    duty_target = _DUTY_NEU + control * ((control>0)?(_DUTY_MIN - _DUTY_NEU):(_DUTY_NEU - _DUTY_MAX))  * _SERVO_ANGLE / 180;


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
  }   
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
} 
