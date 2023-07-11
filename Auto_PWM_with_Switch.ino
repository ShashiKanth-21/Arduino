#include <Servo.h>

#define THROTTLE_PIN 9      // Pin connected to the throttle signal input
#define BUTTON_PIN 2        // Pin connected to the button for start/stop

Servo throttle;
volatile bool throttleActive = false;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  
  throttle.attach(THROTTLE_PIN);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggleThrottle, FALLING);
}

void loop() 
{
  for (int pwmValue = 1000; pwmValue <= 2000; pwmValue += 50)
  {
    if (throttleActive)
    {
      setThrottle(pwmValue);
    }
    else
    {
      setThrottle(1000);
      break;
    }
    delay(1000);  // Delay for 1 second before changing throttle
    Serial.println(pwmValue); // Print PWM value to serial monitor
  }
}

void setThrottle(int pwmValue) {
  throttle.writeMicroseconds(pwmValue);
}

void toggleThrottle() {
  throttleActive = !throttleActive;
}
