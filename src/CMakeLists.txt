set(srcs
        "main.c"
        "sketch.cpp"
        "Truck.h"
        "Truck.cpp"
        "ESP32Servo/ESP32Servo.h"
        "ESP32Servo/ESP32Servo.cpp"
        "ESP32Servo/ESP32PWM.h"
        "ESP32Servo/ESP32PWM.cpp"
)

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
