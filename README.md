# Wobble, the Self Balancing Robot

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
#### Build C++ Shared Libraries and Run the Simulation
```
cd simulation
python3 build.py
```

#### Only Build C++ Shared Libraries
```
cd simulation
python3 build.py --build-only
```

#### Only Run Simulation
```
cd simulation
python3 build.py --no-build
```



## Acknowledgements
The `imu` and `I2Cdev` libraries used in this project are largely based off of existing software written by Jeff Rowberg and maintainers of the [I2C Device Library](https://github.com/jrowberg/i2cdevlib)
- see [original MPU6050 code here](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
