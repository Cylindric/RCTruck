#include "Truck.h"
#include <esp32-hal-gpio.h>

Truck::Truck() {
    pinMode(DRIVE_MOTOR_PIN1, OUTPUT);
    pinMode(DRIVE_MOTOR_PIN2, OUTPUT);

    // Init steering motor
    Servo steeringServo = Servo();
    steeringServo.attach(STEERING_SERVO_PIN);
}

void Truck::Update() {}

void Truck::SetSteeringAngle(int32_t angle) {
    // angle: Controller angle is from -512 to +512

    if (INVERT_STEERING) {
        angle = angle * -1;
    }

    // Remap the controller input from the controller's range to the servo's range
    int32_t adjustedSteeringValue = map(angle, -512, 512, 40, 140);

    // Clamp the steering range to the allowable extents
    adjustedSteeringValue = constrain(adjustedSteeringValue, 40, 140);// constrain(adjustedSteeringValue, 40, 140);

    // Apply trim settings
    //   adjustedSteeringValue = adjustedSteeringValue + steeringTrim;

    // Serial.print("adjustedSteeringValue: ");
    // Serial.println(adjustedSteeringValue, 0);
    //   steeringServo.write(adjustedSteeringValue);
}

void Truck::SetThrottle(int32_t throttle) {
  // Controller throttle is from -512 to +512
  // Motor throttle is from -255 to 255
  if (INVERT_THROTTLE) {
    throttle = throttle * -1;
  }

  int adjustedThrottleValue = map(throttle, -512, 512, -255, 255);
//   set_motor_pwm(adjustedThrottleValue, DRIVE_MOTOR_PIN1, DRIVE_MOTOR_PIN2);
}


// int servoDelay = 0;
// float adjustedSteeringValue = 90;
// float steeringAdjustment = 0;
// int steeringTrim = 0;

// // pwm - duty cycle from -255 to 255
// void set_motor_pwm(int pwm, int PIN1, int PIN2) {
//   Serial.print("Motor ");

//   int power_value = abs(pwm);
//   if (abs(power_value) < DRIVE_MOTOR_MIN_PWM) {
//     power_value = 0;
//   }
//   // int power_value = map(pwm, DRIVE_MOTOR_MIN_PWM, 255);

//   // if (power_value > -220 && power_value < 220) {
//   //   // stopped
//   //   digitalWrite(PIN1, LOW);
//   //   digitalWrite(PIN2, LOW);
//   //   Serial.print("Stopped  ");
//   // } else {

//   if (pwm < 0) {
//     // backwards
//     analogWrite(PIN1, power_value);
//     digitalWrite(PIN2, LOW);
//     Serial.print("Backward ");
//   } else {
//     // forwards
//     digitalWrite(PIN1, LOW);
//     analogWrite(PIN2, power_value);
//     Serial.print("Forward  ");
//   }

//   // }

//   Serial.print("Power: ");
//   Serial.print(power_value);
//   Serial.print(" PWM: ");
//   Serial.println(pwm);

// }

// void attach_steering() {
//   if (steeringServo.attached() == false) {
//     Serial.print("Enabling steering servo.");
//     steeringServo.attach(STEERING_SERVO_PIN);
//   }
// }

// void detach_steering() {
//   Serial.print("Disabling steering servo.");
//   steeringServo.detach();
// }

// // Arduino setup function. Runs in CPU 1
// void setup() {
//   // Configure the drive motor
//   pinMode(DRIVE_MOTOR_PIN1, OUTPUT);
//   pinMode(DRIVE_MOTOR_PIN2, OUTPUT);
//   digitalWrite(DRIVE_MOTOR_PIN1, LOW);
//   digitalWrite(DRIVE_MOTOR_PIN2, LOW);
  
//   // Configure the Steering servo
//   attach_steering();
//   processSteering(0);

//   Serial.begin(115200);
//   Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
//   const uint8_t *addr = BP32.localBdAddress();
//   Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

//   // Setup the Bluepad32 callbacks
//   BP32.setup(&onConnectedController, &onDisconnectedController);

//   // "forgetBluetoothKeys()" should be called when the user performs
//   // a "device factory reset", or similar.
//   // Calling "forgetBluetoothKeys" in setup() just as an example.
//   // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
//   // But it might also fix some connection / re-connection issues.
//   BP32.forgetBluetoothKeys();

//   // Enables mouse / touchpad support for gamepads that support them.
//   // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
//   // - First one: the gamepad
//   // - Second one, which is a "virtual device", is a mouse.
//   // By default, it is disabled.
//   BP32.enableVirtualDevice(false);
//   // You could add additional error handling here,
//   // such as logging the error or attempting to recover.
//   // For example, you might attempt to reset the MCP23X17
//   // and retry initialization before giving up completely.
//   // Then, you could gracefully exit the program or continue
//   // running with limited functionality.
// }



// // Arduino loop function. Runs in CPU 1.
// void loop() {
//   // This call fetches all the controllers' data.
//   // Call this function in your main loop.
//   bool dataUpdated = BP32.update();
//   if (dataUpdated) {
//     processControllers();
//   }
//   // The main loop must have some kind of "yield to lower priority task" event.
//   // Otherwise, the watchdog will get triggered.
//   // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
//   // Detailed info here:
//   // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

//   //     vTaskDelay(1);
//   else { vTaskDelay(1); }
// }
