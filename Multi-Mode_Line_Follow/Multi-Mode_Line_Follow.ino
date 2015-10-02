/*
 * Note that this example requires the Sparkfun RedBot library as well as the 
 * Pololu QTR Sensors library.  I like the QTRSensors library because it provides
 * some useful functions for utilizing IR reflectance sensors.
 *
 * This example shows how you can use a switch case statement to identify
 * multiple modes of following a line which can be switched by pressing the
 * button on the redbot board.  This is an example using the redbot but can
 * be used by many different boards and configurations.
 */

#include <QTRSensors.h>
#include <RedBotSoftwareSerial.h>
#include <RedBot.h>


/******************************* Constants ***********************************/
#define NUM_SENSORS             3   // The number of IR Sensors
#define NUM_SAMPLES_PER_SENSOR  4   // Take the average of four sample readings
#define BUTTON_PIN              12  // The pin attached to the mode switching button
#define BASIC_MODE              1   // Basic mode is a simple if-else line follow
#define P_MODE                  2   // P mode is a simple proportional based line follow
#define PD_MODE                 3   // PD mode is enhanced P mode utilizing D gain.
#define STRAIGHT_MODE           4   // Straight mode isn't a line follow, it simply drives motors forward
#define NUM_MODES               4   // Define the number of modes so we know when to reset
#define LINETHRESHOLD           800 // The value at which basic mode sees the line under a sensor
#define LED_PIN                 13  // The pin attached to an LED
#define BASE_SPEED              150 // The base speed, this is the value with no adjustments
#define MAX_SPEED               225 // The maximum speed, don't ever let a motor exceed this speed

/*************************** PID Value Variables ******************************/
float kP = 0.1f;
float kD = 0.05f;
float lastError = 0.0f;
float error = 0.0f;
float goal = 1000.0f;

// Create the sensors object
QTRSensorsAnalog sensors((unsigned char[]) { 3, 6, 7 }
                , NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
unsigned int sensorValues[NUM_SENSORS];  // An array to hold sensor values

/*************************** Motor & Sensor Objects *****************************/
RedBotMotors motors;
RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7

byte driveMode = 1;  // The current drive mode (start in mode 1)

void setup()
{
  // Calibrate the sensors
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);        // Turn on the LED to indicate we are calibrating
  for (int i = 0; i < 400; i++) {     // make the calibration take about 10 seconds
      sensors.calibrate();            // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);              // turn off the LED when we're done with calibration

  // Set the button pin as an input and identify as a pullup
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // INPUT_PULLUP utilizes an internal pull up resistor

  // Get Serial ready
  Serial.begin(9600);
}

/*
 * In the loop we check for button presses, changing modes as necessary, then
 * check to see what drive mode to use, finally, apply the motor speed and adjustment.
 */
void loop()
{

  // Check for a state change
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    // Turn the motors off
    motors.leftMotor(0);
    motors.rightMotor(0);
    
    // Don't do anything while the button is held down
    while (digitalRead(BUTTON_PIN) == LOW) {}
    
    // Increase the drive mode state
    driveMode++;
    
    // Fix the state (reset back to mode 1 if necessary)
    if (driveMode > NUM_MODES) {
      driveMode = 1;
    }
    
    // Now flash the LED to indicate our current mode
    for (int i = 0; i < driveMode; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }


  // Determine drive strategy and get adjustment value
  double adj;
  switch (driveMode) {
    case BASIC_MODE:
      adj = basicFollow();
      break;
    case P_MODE:
      adj = pdFollow(kP, 0.0);
      break;
    case PD_MODE:
      adj = pdFollow(kP, kD);
      break;
    case STRAIGHT_MODE:
      adj = 0.0;
      break;
  }

  // Write the speed to the motors, constrain ensures that the 
  // value never drops below 0 or exceeds max speed.
  motors.leftMotor(constrain((BASE_SPEED + adj), 0, MAX_SPEED));
  motors.rightMotor(constrain((BASE_SPEED - adj), 0, MAX_SPEED));

}

/*
 * Basic follow prints raw sensor values, then read each sensor.
 * If the left sensor is over the line
 */
double basicFollow() {
  Serial.print(left.read());
  Serial.print("\t");
  Serial.print(center.read());
  Serial.print("\t");
  Serial.print(right.read());
  Serial.println();

  // Find where the line is and react appropriately
  // If the sensor is under the right line, turn left,
  // if the sensor is under the left, turn right and
  // in any other case drive straight
  if (right.read() > LINETHRESHOLD) {
    return 50.0;
  } else if (left.read() > LINETHRESHOLD) {
    return -50.0;
  }
  
  // Always return 0 if left or right sensors were not tripped.
  // The case will be either that it's under the center sensor
  // or that no sensors were tripped.  This keeps the bot from
  return 0.0;
}

/*
 * This method allows the utilization of the PD algorithm, it
 * is designed in a way to allow setting _d to zero so that the
 * function can perform both P and PD algorithm line following.
 */
double pdFollow(double _p, double _d) {
  // Read the position of the line, this handy function from the 
  // QTRSensors library returns a value in increments of 1000 to
  // indicate the sensor (0->1, 1000->2, 2000->3, etc)
  unsigned int position = sensors.readLine(sensorValues);

  // Compute the error, how far the line is from the goal
  error = ((float) position) - goal;

  // Compute the adjustment value
  float value = _p * error + _d * (error - lastError);

  // Record the last error for the next time around
  lastError = error;

  // Print some information
  Serial.print("P");
  Serial.print(_p);
  Serial.print(" D");
  Serial.print(_d);
  Serial.print("\t");
  Serial.print(position);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.println(value);

  return value;
}
