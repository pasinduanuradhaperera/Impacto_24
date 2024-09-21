# Aruco Marker Detection and Robotic Navigation System

<img src="/images/Raspberry_Pi-Logo.wine.svg" alt="raspi"  height="80"><img src="/images/opencv.svg" alt="opencv"  height="80">

<img src="/images/ArUco-markers-with-different-matrix-sizes-4x4-5x5-6x6-7x7-matrices.ppm" alt="aruco marckers"  height="80">

## Overview
This project is an advanced robotic navigation system designed to detect and track Aruco markers in an environment. It utilizes a combination of computer vision, sensor calibration, and motor control to autonomously navigate through the arena, detecting markers, measuring distances, and correcting its path.

## Features
1. **Aruco Marker Identification**: The system can accurately detect Aruco markers and retrieve their unique IDs.
2. **Distance Measurement**: Using camera calibration, the system can estimate distances to the markers accurately.
3. **Camera Calibration**: The robot automatically calibrates its camera based on the detected marker position.
4. **Rotation Detection**: The system uses the MPU6050 sensor to detect and adjust for rotation during navigation.
5. **Encoder-based Distance Measurement**: Motors are equipped with encoders to measure and control distances during movement.
6. **Path Correction**: The robot can realign itself based on detected markers to ensure accurate navigation.
7. **Marker Search Algorithm**: If a marker is missed, the system employs an intelligent search pattern to find it.
8. **Flexible Movement**: The system can move forward, backward, and rotate in search patterns to optimize marker detection.

## Getting Started

### Prerequisites
- Raspberry Pi with RPi.GPIO library
- OpenCV for Aruco marker detection
- MPU6050 for rotation tracking
- Motor encoders for distance tracking 
- motor driver (any motor driver i used L298N motor driver)
- 12v to 5v buckconverter to powerup Raspberry Pi it need to be stable 5v ex:- atleast 5A 
- Coolin fan (recomended otherwise it overheats)

### Installation
1. Clone the repository:
    ```bash
    git clone https://github.com/pasinduanuradhaperera/Impacto_24.git
    ```
2. Install required dependencies:
    check the code and all depends on os and versions 
    check by own 
    if any issue contact me

    basic requirements
    - opencv-python
    - RPi.GPIO
    - mpu6050-raspberrypi


3. Run the game:
    make sure all the files in a one folder run on that folder
    ```bash
    python3 game.py
    ```

### Usage
Once the game starts, the robot will begin detecting markers and navigating the arena. The system can be recalibrate using a physical switch connected to the Raspberry Pi GPIO pins.

- Press the switch to recalibrate when recalibrate robot need to in a non moving possition the robot.
- The robot will detect markers, adjust its path, and calibrate itself based on the marker's position.

## Contributing
Please see the [CONTRIBUTION](CONTRIBUTION.md) file for details on contributing to this project.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Additional
### additional codes and things in this folder [Additional](/Additional)
   - service file for run the program on startup [autoScript](/Additonal/autoScript.service)
   - encoder file for identifies backwards and forwards running [encoder](/Additonal/encoder.py)
   - some previous codes for take some idea ! [prev](/Additonal/prev/)
