# Engineering Materials for WRO Future Engineers 2025

This repository contains the engineering documentation for UneCirciut's self-driving vehicle, designed for the World Robot Olympiad (WRO) Future Engineers 2025 competition. Our project showcases a fully autonomous vehicle capable of navigating challenges through innovative mobility, power management, sensing, and obstacle management strategies.

## Team Introduction

UneCirciut consists of three members: Aysun Abdullayeva, Zubeyda Hasanli, and Aysu Huseynova. We are students from Azerbaijan State University of Economics, passionate about robotics and autonomous systems. Our goal was to design a robust vehicle that balances performance, reliability, and creativity, inspired by real-world self-driving technology.

See our team photos in the t-photos/ directory (Team photos will be added to the t-photos/ directory):

- **`team-official.jpg`**: Formal team photo (to be uploaded).
- **`team-fun.jpg`**: Fun team photo with all members (to be uploaded).

## Vehicle Design Overview

Our vehicle is designed to excel in the WRO Future Engineers challenges, with a focus on autonomous navigation, obstacle avoidance, and efficient power usage. Below, we detail the key aspects of our design.

### Mobility

The vehicle uses a drive system powered by a single 6V 150 RPM DC motor with an attached wheel, driving the rear wheels via a gear system. The front wheels are controlled by a Surpass Hobby 9g Digital Plastic Lightweight Servo for steering. This configuration allows precise control, enabling the vehicle to make sharp turns and maintain stability on uneven surfaces. The drive system is controlled by a single TB6612FNG motor driver interfaced with a Raspberry Pi 4, ensuring smooth acceleration and maneuverability.

### Power Management

The vehicle is powered by [(e.g. **three 3.7V 18650 Li-ion batteries)]** connected in series to provide approximately **[(e.g. 11.1V with a 2200mAh)]** capacity, housed in dedicated battery holders. Power is distributed through **[(e.g. a step-down voltage regulator (HW-083) providing 5V)]\*\* to the Raspberry Pi and sensors. We included a battery management system (BMS) to prevent over-discharge, ensuring reliability during competition runs. The system is optimized for up to 20 minutes of continuous operation, sufficient for both open and game challenges.

Our motivation was to maximize runtime while maintaining a lightweight design. We faced challenges with voltage drops under heavy motor load, resolved by adding capacitors to stabilize the supply.

### Sensing

The vehicle employs multiple sensors for navigation and obstacle detection:

- **HC-SR04 Ultrasonic Sensors (x3)**: Used for front, left, and right obstacle detection up to 2 meters, enabling precise distance measurement for navigation.
- **TCS3200 Color Sensors (x1)**: Provide color detection for identifying track markers and obstacles (red and green cubes).
- **Raspberry Pi Camera Module V1.3**: Enables line-following and visual obstacle recognition with real-time image processing.

These sensors are integrated with a Raspberry Pi 4, processing data in real-time. We selected these sensors for their cost-effectiveness and compatibility with our microcontroller. Calibration was a challenge, addressed by filtering noise in sensor data using a moving average filter.

### Obstacle Management

Our obstacle management strategy combines sensor data and algorithms to navigate challenges. When an obstacle is detected, the vehicle pauses, scans surroundings with ultrasonic sensors, and calculates an alternative path using a rule-based algorithm. The software prioritizes avoiding collisions while minimizing deviation from the intended path. For dynamic obstacles (e.g., red and green cubes), we implemented real-time updates to the path planner, using color sensor data to determine whether to pass left (green) or right (red).

This approach was chosen because rule-based navigation ensures reliable performance in the structured WRO environment. We overcame issues like false positives from sensor noise by averaging multiple sensor readings.

## Electromechanical Schematics

Detailed schematics will be added to the schemes/ directory:

- **`wiring-diagram.png`**: Will show connections between the Raspberry Pi 4, TB6612FNG motor driver, HC-SR04 ultrasonic sensors, TCS3200 color sensor, Raspberry Pi Camera, MG90 servo motor, and battery system.

These diagrams clarify how hardware components are wired and mounted, ensuring reproducibility.

## Code Structure and Integration

The source code will be located in the **`src/`** directory, written in Python for the Raspberry Pi 4. Below is an overview of the planned code modules and their relation to hardware.

### Code Modules

- **`main.py`**: Orchestrates the main control loop, coordinating sensor inputs and motor outputs.
- **`sensors.py`**: Interfaces with HC-SR04 ultrasonic sensors, TCS3200 color sensors, and the Raspberry Pi Camera, processing raw data into usable formats.
- **`motors.py`**: Controls DC motors via PWM signals through the TB6612FNG drivers and the MG90 servo for steering.
- **`navigation.py`**: Implements rule-based pathfinding and line-following algorithms based on color and distance sensor inputs.
- **`utils.py`**: Contains helper functions for data filtering and logging.

Each file includes detailed comments explaining functionality, as judges may not have access to a Raspberry Pi environment.

### Relation to Hardware

- **`sensors.py`** connects to HC-SR04 sensors via GPIO pins (e.g., 17, 18, 27 for trigger/echo), TCS3200 sensors via GPIO pins (e.g., 22, 23), and the Raspberry Pi Camera via the CSI interface.
- **`motors.py`** interfaces with TB6612FNG drivers via GPIO pins (e.g., 24, 25 for PWM) and the MG90 servo via GPIO pin 12.
- **`navigation.py`** uses sensor data to adjust motor speeds for obstacle avoidance and line-following.
  Component Photos
  Below are images of the components used in our vehicle, corresponding to the Bill of Materials, located in the component-photos/ directory.

## **Component Photos**

Below are images of the components used in our vehicle, corresponding to the Bill of Materials.

**Raspberry Pi 4 Model 4Gb**

![image.png](component-photos\raspberry_pi_4.webp)

**Motor and Wheel Set (6V, 150 RPM)**

![image.png](component-photos\motor_wheel_150rpm.webp)

**TB6612FNG Motor Driver**

![image.png](component-photos\tb6612fng_driver.webp)

**HC-SR04 Ultrasonic Sensor**

![image.png](component-photos\hc_sr04_sensor.webp)

**Raspberry pi camera module**

![image.png](component-photos\raspberry_pi_4.webp)

**Servo motor**

![image.png](component-photos\surpass_hobby_servo.webp)

**TCS3200 Color Sensor**

![image.png](component-photos\tcs3200_sensor.webp)

## Photos and Videos

- **Vehicle Photos**: See **`v-photos/`** for images from all angles (to be uploaded):
  - vehicle-front.jpg, vehicle-back.jpg, vehicle-left.jpg, vehicle-right.jpg, vehicle-top.jpg, vehicle-bottom.jpg.
- **Team Photos** (to be uploaded): See **`t-photos/`** for **team-official.jpg** and **team-fun.jpg**.
- **Videos** (to be uploaded): Links to YouTube demonstrations (minimum 30 seconds of autonomous driving per challenge) are in **`video/video.md`**.

## Additional Resources

- **`models/`** contains STL files for 3D-printed chassis parts.
- **`other/`** includes datasheets for sensors and motors.

We encourage other teams to explore this repository for inspiration, as per the WRO’s open-sharing ethos. For questions, contact us at [unecirciut@gmail.com](mailto:unecirciut@gmail.com).
