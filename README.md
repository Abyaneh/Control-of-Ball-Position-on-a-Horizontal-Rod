# Control of Ball Position on a Horizontal Rod

This project demonstrates a dynamic control system that maintains the position of a ball on a horizontal rod. Using an STM32 microcontroller, ultrasonic sensors, and a PID controller, the system dynamically adjusts the motor to keep the ball in the desired location, even when external disturbances are introduced.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Project Layers](#project-layers)
  - [Control System Layer](#control-system-layer)
  - [Sensor Feedback Layer](#sensor-feedback-layer)
  - [Motor Actuation Layer](#motor-actuation-layer)
  - [MATLAB Analysis Layer](#matlab-analysis-layer)
- [Explain more about the PID Control](#explain-more-about-the-pid-control)
- [My Dataset](#my-dataset)
- [Technologies & Tools Used](#technologies--tools-used)
- [Video Demonstration](#video-demonstration)
- [How to Run the Project](#how-to-run-the-project)
  - [STM32 Setup](#stm32-setup)
  - [MATLAB Visualization](#matlab-visualization)
  - [Adjust PID Parameters](#adjust-pid-parameters)
- [Team Members](#team-members)
- [Contributing](#contributing)
- [License](#license)
- [Back to Top](#table-of-contents)

---

## Introduction
This project is a practical demonstration of closed-loop control using a PID controller to maintain the position of a ball on a horizontal rod. It combines mechanical design with real-time control algorithms and feedback from sensors to achieve precise positioning.

The system consists of:
- A horizontal rod actuated by a servo motor.
- Two ultrasonic sensors that detect the ball's position on the rod.
- A PID controller implemented in an STM32 microcontroller to dynamically adjust the rod's angle.

This README explains the structure, operation, and technical details of the project, along with how to set it up and run the system.

[Back to Top](#table-of-contents)

---

## Features
- **Real-time Ball Position Control:** The system can maintain the position of the ball using PID feedback control.
- **Dynamic PID Adjustment:** Users can tune the PID controller to optimize system performance, minimizing overshoot, rise time, and steady-state error.
- **Ultrasonic Sensors for Feedback:** Two ultrasonic sensors are used for precise position detection of the ball on the rod.
- **MATLAB Visualizations:** MATLAB is used for graphing and analyzing the system response under different PID configurations.

[Back to Top](#table-of-contents)

---

## Project Layers

### Control System Layer
This layer is responsible for the real-time control of the rod using the STM32 microcontroller. The control system is built around a PID algorithm that adjusts the motor's position based on the error between the ball's current position and the desired position.

### Sensor Feedback Layer
The ball's position is detected using ultrasonic sensors. The sensors send distance data to the STM32, which processes it to compute the position error. This error is then used by the PID controller to adjust the rod's angle.

### Motor Actuation Layer
The motor is driven by a servo driver that receives commands from the STM32 microcontroller. The STM32 sends PWM signals to control the motor's angle and adjust the rod in response to ball movement.

### MATLAB Analysis Layer
The data collected during the system's operation is plotted in MATLAB. MATLAB is used to visualize the response of the system with different PID parameters and analyze the system’s performance, including overshoot, rise time, and settling time.

[Back to Top](#table-of-contents)

---

## Explain more about the PID Control
The PID (Proportional-Integral-Derivative) controller is the heart of this system. It continuously computes an error value as the difference between the desired and actual ball position and adjusts the motor position to minimize this error.

- **Proportional (P):** Determines the reaction to the current error.
- **Integral (I):** Accounts for past errors to eliminate steady-state error.
- **Derivative (D):** Predicts future error based on the rate of change, helping reduce overshoot and oscillations.

Different PID coefficients were tested, and MATLAB was used to visualize the system's response under different configurations. You can find detailed plots in the `plots/` folder.

[Back to Top](#table-of-contents)

---

## My Dataset
The dataset includes real-time data recorded from the system during various test scenarios. The data consists of ball position readings and motor angle data. These datasets are stored in the `text-data/` directory and are used for further analysis and plotting in MATLAB.

Data includes:
- **Ball Position Data:** Recorded in real-time from ultrasonic sensors.
- **Motor Angle Data:** Collected from the motor driver during system operation.

[Back to Top](#table-of-contents)

---

## Technologies & Tools Used
- **Microcontroller:** STM32F407ZET6
- **Programming Languages:** Embedded C for STM32, MATLAB for data analysis
- **Control Algorithms:** PID control implemented on the STM32 board
- **Hardware Components:** Ultrasonic sensors (SRF05), AC servo motor (Delta Asda A2), planetary gearbox
- **Software Tools:** Keil IDE for STM32 development, MATLAB for analysis and graphing

[Back to Top](#table-of-contents)

---

## Video Demonstration
Check out the video demonstration of the project in action:

[**Watch the Video**](#) (Insert your video link here)

[Back to Top](#table-of-contents)

---

## How to Run the Project

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/YourUsername/Control_of_Ball_Position.git
