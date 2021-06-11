
# EE240500 Homework 4

## About The Project

This is a project for Embedded System Lab's homework.  
About XBee, BB Car, OpenMV. 

### Built With

* C++
* Python

### Equipment List

* PC or notebook
* B_L4S5I_IOT01A
* XBee chips
* PING
* Encoder
* Boe Bot Car
* OpenMV H7 Plus board

## Homework Description

### Part 1 (XBee Controlled BB Car)

1. PC/Python sends a RPC command though XBee (with position parameters) to reverse park a BB car.
2. The BB Car will park automatically according to the position parameters.
3. The width of parking space is no more than 4cm of BB car width.
4. BB car could start at different initial positions. Each position can be identified by the relative distance between BB car and the parking space and the car forward-facing direction. For example, in the following figure, we have set the BB car at (d1, d2, west) position. This parameter will be sent from PC to mbed for the reverse parking.

### Part 2 (Line Following BB Car)

1. Use OpenMV to detect a straight line (printed on a paper).
2. Send the parameters of the detected line (end points of lines as x1, y1, x2, y2) to mbed through UART connections.
3. Steer BB car to go forward and follow the line.

### Part 3 (BB Car Position Calibration)

1. Print or show an AprilTag (from mbed lab 14).
2. Please use OpenMV to scan the above AprilTag codes. We can use the scanned AprilTag to determine the viewing angle from OpenMV to the AprilTag surface.
3. Please steer BB Car such that the car is facing directly and perpendicular to the AprilTag surface.
4. Also use PING to measure the distance between BB Car and AprilTag surface.
5. Please show and verify (use a ruler) the distance measured above.

## Run Locally

Clone the project

```bash
  git clone https://github.com/SYJINTW/NTHU240500_hw4.git
```

Go to the project directory

```bash
  cd NTHU240500_hw4
```

## Running Tests

To run tests, run the following command

### Part 1

#### Setting

* Setting d1 and d2 in car.py
    
    ```python
    # Setting d1 and d2
    s.write("/parking/run d1 d2 west\n".encode())
    ```

#### Running & compile

  ```bash
    cd part1
    mbed add https://gitlab.larc-nthu.net/ee2405_2021/bbcar
    sudo mbed compile --source . --source ~/ee2405/mbed-os-build/ -m B_L4S5I_IOT01A -t GCC_ARM -f
    sudo python3 car.py
  ```

### Part 2

#### Running & compile

  ```bash
    cd part2
    mbed add https://gitlab.larc-nthu.net/ee2405_2021/bbcar
    sudo mbed compile --source . --source ~/ee2405/mbed-os-build/ -m B_L4S5I_IOT01A -t GCC_ARM -f
  ```

### Part 3

#### Running & compile

  ```bash
    cd part3
    mbed add https://gitlab.larc-nthu.net/ee2405_2021/bbcar
    sudo mbed compile --source . --source ~/ee2405/mbed-os-build/ -m B_L4S5I_IOT01A -t GCC_ARM -f
    sudo screen /dev/ttyACM0
  ```
  
## Demo

### Part 1 Video

  [![Part one demo](https://img.youtube.com/vi/NePzGIxofaE/0.jpg)](https://www.youtube.com/watch?v=NePzGIxofaE)

### Part 2 Video

  [![Part two demo1](https://img.youtube.com/vi/vPuVp3WOvOo/0.jpg)](https://www.youtube.com/watch?v=vPuVp3WOvOo)

  [![Part two demo2](https://img.youtube.com/vi/1ElQ3R_WQgg/0.jpg)](https://www.youtube.com/watch?v=1ElQ3R_WQgg)

### Part 3 Video

  [![Part three demo](https://img.youtube.com/vi/uFNfxIMsOAA/0.jpg)](https://www.youtube.com/watch?v=uFNfxIMsOAA)


  
## Acknowledgements

 - [Electronics Tutorials](https://www.electronics-tutorials.ws/filter/filter_2.html)
 - [w3schools.com](https://www.w3schools.com/python/)
 - [tensorflow](https://github.com/tensorflow/models/tree/master/research/slim?fbclid=IwAR3CeDa2WRadJT7cvaZa723IJGDV72QhXXCNj4NXJc41U0Of6PkQaa5EG5c#preparing-the-datasets)
 - [openmv-cam-h7-plus](https://openmv.io/products/openmv-cam-h7-plus)
 - [apriltag image](https://github.com/AprilRobotics/apriltag-imgs/)