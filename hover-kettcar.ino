#include <SoftwareSerial.h>
// Build on top of demo code from https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
// #define DEBUG_RX                 // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define HOVER_RX_PIN 4
#define HOVER_TX_PIN 5
#define SPEED_POTI_PIN A7
#define DIRECTION_SWITCH_PIN 3

SoftwareSerial HoverSerial(HOVER_RX_PIN, HOVER_TX_PIN); // RX, TX

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(SPEED_POTI_PIN, INPUT);
  pinMode(DIRECTION_SWITCH_PIN, INPUT_PULLUP);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available())
  {
    incomingByte = HoverSerial.read();                                  // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");
      Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");
      Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");
      Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");
      Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");
      Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");
      Serial.println(Feedback.cmdLed);
    }
    else
    {
      Serial.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

int speed = 0;
int lastSpeed = 0;

#define FWD 1
#define BWD -1

#define FWD_ACCELERATE 2
#define FWD_SLOWDOWN 1
#define DRIVING_FAST 550           // if the speed does not exceed this value since last stop, brake always with power Slowdown
#define POWER_SLOWDOWN_TRIGGER 300 // Value between 0 and 1023, if speed drops UNDER this value,  POWER_SLOWDOWN_STEP will be used for slowdown instead of normal FWD_SLOWDOWN
#define POWER_SLOWDOWN_STEP 3
#define BWD_ACCELERATE 2
#define BWD_SLOWDOWN 5

#define POTI_MIN 180
#define POTI_MAX 850
#define HOVER_MAX 1023
#define HOVER_MIN 0

int direction = FWD;
byte wasFast = 0;

void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  // Receive();
  if (lastSpeed >= DRIVING_FAST)
  {
    wasFast = 1;
  }

  int speedRaw = analogRead(SPEED_POTI_PIN); // read the input pin
  speedRaw = max(speedRaw, POTI_MIN);
  speedRaw = min(speedRaw, POTI_MAX);

  speed = map(speedRaw, POTI_MIN, POTI_MAX, HOVER_MIN, HOVER_MAX);
  int speedGoal = speed;

  // Direction read
  int directionRaw = digitalRead(DIRECTION_SWITCH_PIN);
  if (lastSpeed == 0) // Change Direction only if speed is 0
  {
    direction = directionRaw == HIGH ? FWD : BWD;
    wasFast = 0; // reset wasFast when stopping
  }
  speed *= direction;

  if (direction == FWD)
  {

    if (speed < lastSpeed) // Slowdown Forward
    {
      if (!wasFast || lastSpeed <= POWER_SLOWDOWN_TRIGGER || directionRaw == LOW)
      { // Power Slowdown if under SLOWDOWN_TRIGGER or if direction switch was set to LOW (=Backwards) or was not fast
        speed = max(lastSpeed - POWER_SLOWDOWN_STEP, HOVER_MIN);
      }
      else
      {
        speed = max(lastSpeed - FWD_SLOWDOWN, HOVER_MIN);
      }
    }

    if (speed > lastSpeed) // Accelerate Forward
    {
      speed = min(lastSpeed + FWD_ACCELERATE, HOVER_MAX);
    }
  }
  else if (direction == BWD)
  {
    if (speed < lastSpeed) // Accelerate Backwards
    {
      speed = max(lastSpeed - BWD_ACCELERATE, -HOVER_MAX);
    }

    if (speed > lastSpeed) // Slowdown Backwards
    {
      speed = min(lastSpeed + BWD_SLOWDOWN, -HOVER_MIN);
    }
  }

  Send(0, speed);
  lastSpeed = speed; // save last speed

  Serial.print("Read speed: ");
  Serial.print(speedRaw);
  Serial.print(" >>> ");
  Serial.print(speed);
  Serial.print(" | ");
  Serial.print(speedGoal);
  Serial.print(" Direction: ");
  Serial.println(direction);
  delay(2);

}
