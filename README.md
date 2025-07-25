# Wobble, the Self Balancing Robot

## Overview
`onboard-software`
- Contains all the files relating to real time embedded software running on the robot's ESP32
- Leverages the Arduino Hardware Abstraction Layer to read/write to digital/analog pins

`shared-lib`
- Contains common source/header file implementations of libraries uses across many folders in this project
    - e.g. both `onboard-software` and `wireless-controller` require the `communication` library to talk to each other

`simulation`
- Contains all the Python source files/logs of the PyBullet based simulation of the robot

`tools`
- Contains simple Jupyter Notebooks to analyze/view data from simulations

`wireless-controller`
- Contains all the files for the wireless transceiver used to communicate with the robot via the ESP-NOW protocol

## Important Files
`onboard-software/src/main.cpp`
- This is the main file that the physical robot is running on. If you look at the `BalanceTask()`, this is the main "loop" that the robot uses to balance itself.

`simulation/src/sim.py`
- This file is the "environment" I used to test all the control algorithms. Here, you can see how I injected noise into my sensor readings and actuator commands.

`simulation/src/demo.py`
- This file actually instantiates the controllers and runs them inside the simulation. Additionally, it provides renders of the robot so you can see what it looks like while it tries to balance.

## Build Instructions
#### 1. Make a virtual env
```
python3 -m venv venv
source venv/bin/activate
```

#### 2. Install dependencies
```
pip install -r requirements.txt
```


## Simulation
#### Run 3 Trials for Each Type of Controller
```
cd simulation
python3 src/demo.py -c pid
python3 src/demo.py -c lqr
python3 src/demo.py -c mpc
```


## Deploying to Hardware

### To Deploy to ESP32
```
pio run --target upload --upload-port /dev/cu.usbserial-0001 
```

### To Compile
```
pio run
```

## Acknowledgements
The `imu` and `I2Cdev` libraries used in this project are largely based off of existing software written by Jeff Rowberg and maintainers of the [I2C Device Library](https://github.com/jrowberg/i2cdevlib)
- see [original MPU6050 code here](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)



