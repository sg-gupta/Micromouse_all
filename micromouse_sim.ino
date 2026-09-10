#define IR_LEFT 25
#define IR_FRONT_LEFT 24
#define IR_FRONT_RIGHT 23
#define IR_RIGHT 22

#define TRIG_LEFT 28
#define ECHO_LEFT 29
#define TRIG_RIGHT 30
#define ECHO_RIGHT 31

#define MOTOR_L_PWM 6
#define MOTOR_R_PWM 11
#define MOTOR_L_IN1 8
#define MOTOR_L_IN2 7
#define MOTOR_R_IN1 12
#define MOTOR_R_IN2 10

#define ENCODER_L_A 3
#define ENCODER_R_A 2
#define PPR 1050
#define WHEEL_DIAMETER 4.4
#define WHEELBASE 9.0

#define MAX_DISTANCE 200
#define SPEED 120

volatile uint16_t pulsesL = 0;
volatile uint16_t pulsesR = 0;

float Kp = 3.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 1.2;  // Derivative gain
float previousError = 0;
float integral = 0;

float leftDistance, rightDistance, error, adjustment;

void encoderISR_L() { pulsesL++; }
void encoderISR_R() { pulsesR++; }

void setup() {
    // Encoder Setup
    pinMode(ENCODER_L_A, INPUT_PULLUP);
    pinMode(ENCODER_R_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderISR_L, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderISR_R, RISING);

    // Motor Pins Setup
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);

    // IR Sensor Setup
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_FRONT_LEFT, INPUT);
    pinMode(IR_FRONT_RIGHT, INPUT);
    pinMode(IR_RIGHT, INPUT);

    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);

    Serial.begin(9600);
    Serial.println("Setup complete.");
}

void loop() {
    int wallStates[3];
    detectWalls(wallStates);

    if (wallStates[1]) { // Front wall detected
        Serial.println("Front wall detected.");
        if (wallStates[0] && wallStates[2]) {
            Serial.println("Left and Right walls detected. Turning around.");
            turnAround();
        } else if (!wallStates[0]) {
            Serial.println("No Left wall. Turning left.");
            turnLeft();
        } else if (!wallStates[2]) {
            Serial.println("No Right wall. Turning right.");
            turnRight();
        }
    } else {
        Serial.println("No front wall. Moving forward.");
        moveForwardWithPID();
    }
}

inline void detectWalls(int* states) {
    states[0] = (!(digitalRead(IR_LEFT)));   // Left
    states[1] = (!(digitalRead(IR_FRONT_LEFT)) || !(digitalRead(IR_FRONT_RIGHT))); // Front
    states[2] = (!(digitalRead(IR_RIGHT)));  // Right
    Serial.print("Wall states: ");
    Serial.print(states[0]);
    Serial.print(", ");
    Serial.print(states[1]);
    Serial.print(", ");
    Serial.println(states[2]);
}

void moveForwardWithPID() {

    while ((digitalRead(IR_FRONT_LEFT) || digitalRead(IR_FRONT_RIGHT)) &&
       !(digitalRead(IR_LEFT) && digitalRead(IR_RIGHT))) {

        leftDistance = getDistance(TRIG_LEFT, ECHO_LEFT);
        rightDistance = getDistance(TRIG_RIGHT, ECHO_RIGHT);

        error = leftDistance - rightDistance;
        integral += error;
        float derivative = error - previousError;

        adjustment = Kp * error + Ki * integral + Kd * derivative;
        previousError = error;

        Serial.print("Error: ");
        Serial.print(error);
        Serial.print(" Adjustment: ");
        Serial.println(adjustment);

        moveMotors(SPEED - adjustment, SPEED + adjustment, true, true);
    }
    stopMotors();
}

float getDistance(int trigPin, int echoPin) {
    long duration;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343) / 2.0;
    Serial.print("Distance (cm) from trigPin ");
    Serial.print(trigPin);
    Serial.print(": ");
    Serial.println(distance);
    return distance;
}

void moveMotors(int speedL, int speedR, bool forwardL, bool forwardR) {
    // Left Motor
    if (forwardL) {
        digitalWrite(MOTOR_L_IN1, HIGH);
        digitalWrite(MOTOR_L_IN2, LOW);
    } else {
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, HIGH);
    }
    analogWrite(MOTOR_L_PWM, abs(speedL)); // Ensure speed is non-negative

    // Right Motor
    if (forwardR) {
        digitalWrite(MOTOR_R_IN1, HIGH);
        digitalWrite(MOTOR_R_IN2, LOW);
    } else {
        digitalWrite(MOTOR_R_IN1, LOW);
        digitalWrite(MOTOR_R_IN2, HIGH);
    }
    analogWrite(MOTOR_R_PWM, abs(speedR)); // Ensure speed is non-negative
}

inline void turnLeft() {
    Serial.println("Turning left.");
    turnAngle(40, SPEED+20); // Turn left by 90 degrees
}

inline void turnRight() {
    Serial.println("Turning right.");
    turnAngle(-40, SPEED+20); // Turn right by 90 degrees
}

inline void turnAround() {
    Serial.println("Turning around.");
    turnAngle(80, SPEED+20); // Turn around by 180 degrees
}

void turnAngle(float angle, int speed) {
    pulsesL = 0;
    pulsesR = 0;
    float wheelbaseCircumference = PI * WHEELBASE;
    float turnDistance = (abs(angle) / 360.0) * wheelbaseCircumference;
    int targetPulses = abs((turnDistance / (PI * WHEEL_DIAMETER)) * PPR);

    Serial.print("Turning ");
    Serial.print(angle);
    Serial.println(" degrees.");
    Serial.print("Target pulses: ");
    Serial.println(targetPulses);

    if (angle > 0) {
        // Turn left
        moveMotors(speed, speed, false, true); // Left motor backward, Right motor forward
    } else {
        // Turn right
        moveMotors(speed, speed, true, false); // Left motor forward, Right motor backward
    }

    while (pulsesL < targetPulses || pulsesR < targetPulses) {
        Serial.print("PulsesL: ");
        Serial.print(pulsesL);
        Serial.print(" / PulsesR: ");
        Serial.println(pulsesR);

        if (pulsesL >= targetPulses) moveMotors(0, speed, angle > 0 ? false : true, angle > 0 ? true : false); // Stop Left Motor
        if (pulsesR >= targetPulses) moveMotors(speed, 0, angle > 0 ? false : true, angle > 0 ? true : false); // Stop Right Motor
    }
    stopMotors();
    Serial.println("Turn complete.");
}

void stopMotors() {
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
    Serial.println("Motors stopped.");
}
