#include <cstdint>
#include "ESP32Servo/ESP32Servo.h"

#ifndef TRUCK_CONTROLLER_H
#define TRUCK_CONTROLLER_H

class Truck {
   public:
        // Physical pin mappings
        static const uint8_t STEERING_SERVO_PIN = 13;
        static const uint8_t DRIVE_MOTOR_PIN1 = 26;
        static const uint8_t DRIVE_MOTOR_PIN2 = 25;

        // Motor configuration
        static const bool INVERT_STEERING = false;
        static const bool INVERT_THROTTLE = false;
        static const uint8_t DRIVE_MOTOR_MIN_PWM = 50;

        Truck();

        void Update();
        void SetSteeringAngle(int32_t angle);
        void SetThrottle(int32_t throttle);

    private:
        Servo steeringServo;
        
        long map(long x, long in_min, long in_max, long out_min, long out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        // int32_t constrain(int32_t val, int32_t min, int32_t max) {
        //     if (val < min) {
        //         return min;
        //     }
        //     if (val > max) {
        //         return max;
        //     }
        //     return val;
        // }
};

typedef Truck* TruckPtr;

#endif  // TRUCK_CONTROLLER_H
