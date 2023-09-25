\page how-to-use How to use
[TOC]

The library lives in the KMR::dxl namespace. 

# I. Initializations
## Step 1: Write a motor configuration file

The first thing that needs to be done is to create a yaml configuration file of the motors used in the projet. \n
Let's assume the robot has 4 motors with IDs {1, 2, 3, 4}, all of model MX_64R. Motors 1 and 3 are in multiturn mode, while the 2 others are not. \n
The motor configuration file will look as follows:

```yaml
# motors_config.yaml
nbr_motors: 4    
motors: 
  - ID: 1
    model: MX_64R
    multiturn: 1
  - ID: 2
    model: MX_64R
    multiturn: 0
  - ID: 3
    model: MX_64R
    multiturn: 1
  - ID: 4
    model: MX_64R
    multiturn: 0
```

The multiturn mode corresponds to the "extended position control" in dynamixel's SDK. It's a position control allowing 255 turns in each direction before the input value saturates. In order to avoid saturation and thus, to allow motors turning indefinitely, this library resets a motor after it does more than a full turn relative to its starting 0 position. \n
After the reset, the motor thus detects its position as being between -180° and +180° again.  

In addition to setting the "multiturn" configuration in the yaml file correctly, the motors themselves must of course be configured for the correct operation mode.

*Note*: the "multiturn" configuration must always be present in the yaml file, even if it's not used in any motor. Just set it to 0 for each motor.

## Step 2: Initialize hal

The second step is to initialize a KMR::dxl::Hal object in the project's main file. \n
This allows to make sure the motors configuration files are correct, as well as to create the hidden control tables used to abstract the hardware layer:

```cpp
// In main.cpp
KMR::dxl::Hal hal;

char path_to_motor_config[] = "../config/motors_config.yaml";
char path_to_KMR_dxl[] = "../KMR_dxl";

std::vector<int> all_ids = hal.init(path_to_motor_config, path_to_KMR_dxl);
```

# II. Create your Robot class

## Step 3: Declare Robot

The project's Robot class needs to inherit KMR::dxl::BaseRobot, which results in this class declaration: 

```cpp
// robot.hpp
class Robot : public KMR::dxl::BaseRobot {
    ....
};
```
and this constructor:

```cpp
// robot.cpp
Robot::Robot(vector<int> all_ids, const char *port_name, int baudrate, KMR::dxl::Hal hal)
: KMR::dxl::BaseRobot(all_ids, port_name, baudrate, hal)
{
    ...
}
```
## Step 4: Writer handlers

To create a handler that sends data to the motors (example: goal positions, LED control), a KMR::dxl::Writer object is used. \n
It can be declared as a private member of Robot:

```cpp
// robot.hpp
class Robot : public KMR::dxl::BaseRobot {
    private:
        KMR::dxl::Writer *m_writer;
};
```

and initialized in Robot's constructor:
```cpp
m_writer = new KMR::dxl::Writer(writer_fields, ids, portHandler_, packetHandler_, m_hal, 0);
```
The "writer_fields" is the list of field(s) that will be handled by this specific Writer object. Since this library uses the protocol 2 of dynamixel's SDK, a single Writer can handle several control fields, resulting in an indirect address writing - but this is handled by the library automatically.

The fields themselves are enumerated in KMR::dxl::Fields and correspond to control fields found in Dynamixels' control tables, as in https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#control-table \n 
For example, if we wish to have a Writer handler that sends goal positions and LED status commands to motors, the Writer definition becomes:

```cpp
vector<KMR::dxl::Fields> writer_fields = {KMR::dxl::GOAL_POS, KMR::dxl::LED};
m_writer = new KMR::dxl::Writer(writer_fields, ids, portHandler_, packetHandler_, m_hal, 0);
```

Finally, the last element we need to use the Writer is the function that actually sends the data to motors. The creation of this function is very straightforward.

The method KMR::dxl::Writer::addDataToWrite allows to save data that need to be sent to motors into the Writer's private attribute table. Then, once all the data is updated, it is sent with the method KMR::dxl::Writer::syncWrite. \n 
Keeping the same Writer example, a public method can be defined inside Robot: 

```cpp
// robot.cpp
void Robot::writeData(vector<float> angles, vector<int> LED_vals, vector<int> ids)
{
    m_writer->addDataToWrite(angles, KMR::dxl::GOAL_POS, ids);
    m_writer->addDataToWrite(LED_vals, KMR::dxl::LED, ids);

    m_writer->syncWrite(ids);
}
```

This public method Robot::writeData can be for example called from the main when new control values are received from the controller.

## Step 5: Reader handlers

In order to fetch data from the motors' sensors (for example current position and temperature), a KMR::dxl::Reader object is required. It works extremely similarly to its Writer counterpart.

When writing the read function, one needs to be careful about the order in which the control fields were written when declaring the Reader. \n
The method KMR::dxl::Reader::syncRead stores the data received from motors into the Reader's attribute table "m_dataFromMotor", organized like this:

|          | field1 | field2 | .... | field_n |
|----------|--------|--------|------|---------|
| id[0]    |        |        |      |         |
| ...      |        |        |      |         |
| id[last] |        |        |      |         |


As such, if for example we wanted a Reader that reads present position and LED status, the declaration would be:
```cpp
vector<KMR::dxl::Fields> reader_fields = {KMR::dxl::PRESENT_POS, KMR::dxl::LED};
m_reader = new KMR::dxl::Reader(reader_fields, handlers_ids, portHandler_, packetHandler_, m_hal, 0);
```

which means the present position data will be saved in the first column of "m_dataFromMotor" and the LED status in the second. \n
As such, the reading function is:
```cpp
// robot.cpp
void Robot::readData(vector<int> ids, vector<float>& fbck_angles, vector<float>& fbck_leds)
{
    m_reader->syncRead(ids);

    for (int i=0; i<ids.size(); i++) {
        fbck_angles[i] = m_reader->m_dataFromMotor[i][0];
        fbck_leds[i] = m_reader->m_dataFromMotor[i][1];
    }

}
```

## Note: multiturn reset
The public method KMR::dxl::BaseRobot::resetMultiturnMotors resets the motors flagged as in need of a reset. It is inherited by the Robot class, and needs to be called only if the project contains multiturn motors. 

It is up to the user where they want to call it. A good idea is to call it at the start of each control loop, before reading the sensor values. \n
If wished, one can also add it for example at the end of the writing functions.

> **Warning** <br> 
> If the resetMultiturnMotors method is called at the end of the writing method, make sure the motors had enough time to execute the movement before calling the reset, such as by adding a short sleep time. If they are reset before they could execute the whole movement, it results in undefined behavior.


# III. Create a Robot object

Keeping the previous examples, a very basic project could look like this:
```cpp
// main.cpp

KMR::dxl::Hal hal;

char path_to_motor_config[] = "../config/motors_config.yaml";
char path_to_KMR_dxl[] = "../KMR_dxl";

std::vector<int> all_ids = hal.init(path_to_motor_config, path_to_KMR_dxl);

// Create robot instance
int baudrate = 1000000;
Robot robot(all_ids, "/dev/ttyUSB0", baudrate, hal);

// Feedback tables
vector<float> fbck_angles(4);
vector<float> fbck_leds(4);
vector<float> goal_angles(4);
vector<float> goal_leds(4);

// Start the loop
robot.enableMotors();

while(1) {
    // Only needed if there are multiturn motors
    robot.resetMultiturnMotors();

    robot.readData(all_ids, fbck_angles, fbck_leds);

    // Send the feedback values to the controller and get the new
    // goal values into goal_angles and goal_leds

    robot.writeData(goal_angles, goal_leds, all_ids);

    sleep(1);
}
``` 