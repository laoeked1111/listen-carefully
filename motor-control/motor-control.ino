// stepper control
#define PUL 19
#define DIR 18
#define EN 17 

#define STEPPER_ENABLED 0
#define STEPPER_DISABLED !STEPPER_ENABLED
#define DIR_FORWARD 0
#define DIR_BACKWARD !DIR_FORWARD

#define MICROSTEPS 32
#define STEPS_PER_REV 200
#define PULSES_PER_REV MICROSTEPS * STEPS_PER_REV 

#define MM_PER_REV 40

// trigger
#define TRIGGER 16
#define TRIGGER_ACTIVE 1
#define TRIGGER_INACTIVE !TRIGGER_ACTIVE

// ultrasonic sensor
#define SENSOR_TRIG 3
#define SENSOR_ECHO 4

// speaker
#define SPEAKER 14

// lenses
#define F1 30.0 // cm
#define F2 30.0 // cm

#define MIN_DELTA 5.0 // cm
#define MAX_D 50.0 // cm
#define MIN_D 17.0 // cm

#define FREQ 5000 // Hz

float F;
float D;

float prev_dist = 0;

int debounce(int pin) {
  if(digitalRead(pin)){
    delay(20);
    if(digitalRead(pin)){
      return 1;
    }
  }
  return 0;
}

void moveNSteps(int steps, int dir) {
  for (int i = 0; i < steps * MICROSTEPS; i ++) {
    digitalWrite(DIR, dir);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);
  }
}

void moveDist_cm(float dist_cm, int dir) {
  moveNSteps((int) 10 * dist_cm * MM_PER_REV, dir);
}

float findDist() { // returns distance in centimeter
  digitalWrite(SENSOR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_TRIG, LOW);

  int timeout = 10000;

  float duration = pulseIn(SENSOR_ECHO, HIGH, timeout);
  if(duration == 0.0f){
    return prev_dist;
  }
  
  float distance   = duration * .0343 / 2;
  prev_dist = distance;
  return distance;
}

float calcD(float F_needed) {
  return F1 * F2 * (1/F1 + 1/F2 - 1/F_needed);
}
float dist = 0.0;

void setup() {
  pinMode(EN, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  
  digitalWrite(EN, STEPPER_ENABLED);

  pinMode(TRIGGER, INPUT);

  pinMode(SENSOR_TRIG, OUTPUT);
  pinMode(SENSOR_ECHO, INPUT);

  Serial.begin(9600);

  // home stepper --> must move the carriage to the back before use
  D = MIN_D;
  F = 1/(1/F1 + 1/F2 - D/F1/F2);
}

void loop() {
  F = 1/(1/F1 + 1/F2 - D/F1/F2);

  dist = (findDist() + MAX_D)*0.5 + 0.5*dist;
  Serial.println("Distance: " + String(dist) + " cm.");

  float D_needed = calcD(dist);
  if(D_needed > MAX_D) D_needed = MAX_D;
  if(D_needed < MIN_D) D_needed = MIN_D;

  Serial.println("D_needed: " + String(D_needed) + " cm.");

  float delta = D_needed - D;
  float abs_delta = (delta >= 0) ? delta : (-1 * delta);

  if (D_needed >= MIN_D && D_needed <= MAX_D){
    if (abs_delta >= 1) {

      int this_dir = (delta > 0) ? DIR_FORWARD : DIR_BACKWARD;
      int sign = (delta > 0) ? 1 : -1;

      moveDist_cm(0.1, this_dir);
      
      D = D + 1.0f * (sign);
    }
    else{
      delay(100);
    }
  } else if (D_needed < MIN_D) {
    // moveDist_cm(0.1, DIR_BACKWARD);
    D = MIN_D;
    Serial.println("Minimum D reached.");
  } else {
    // moveDist_cm(0.1, DIR_FORWARD);
    D = MAX_D;
    Serial.println("Maximum D reached.");
  }

  if (debounce(TRIGGER)) {
    Serial.println("Trigger pressed.");
    tone(SPEAKER, FREQ, 1000);
  }
}


