# How to build the ROV Submarine

## Intro

The goal of this project is to make an open source ROV submarine. This is not a full tutorial on how to build it but more a detailed project. Tguis is still a work in progress, I managed to have a functionnal Submarine but with plenty of effort. I share all my work and feel free to make your own, modify it and explore the see.

My main inspiration comes from this youtube channel. 
A great project for DIY submarine is the blueRov project but it requires a pixwwhak board or equivalent. I wanted to start from nothing so i created my own controller board with all the sensors i needed.

### Bill of Materials
This is a list of the main components needed. I bought most of the parts on Aliexpress.
https://docs.google.com/spreadsheets/d/1C2mfdet0Q4sjmvpAgbT_p8TrK4USZ2G9Qunxdg3Yzaw/edit?usp=sharing

### List of tools
Tools available in a fablab should be enough, in particular I used:
- a laser cutter for cutting acrylic plates
- a CNC router for cutting aluminium and acrylic plates
- a 3D printer
- soldering tools
- a spot welder tool for the battery

## The Electronic

Inside the Sub yo have 3 boards:
- a raspberry pi 
- a sensor and controller board (SCB)
- a power delivery board (PDB)

with the following structure:

The raspberry pi run ros, receive commands from the computer via the ethernet cable, receive sensor data and send motor commands via the SCB. The PDB deliver the power to the boards and the motors.

The SCB and PDB are customed boards made with EasyEda, I added the gerber files. This is still a work in progress but the first version works. You can use a pixhawk if you prefere.

### The raspberry pi
I'm using a Raspberry Pi 3B+ running ros with ubuntu. A raspicam is connected for the live video feed. 

### Sensor and Controller Board (SCB)
The SCB replaces the function of an Pixhack board. An ESP32 read data from sensors, control the motors and the lights. Thes ESP32 is connected to the raspberry via the Serial with an USB cable.

The following sensors are used:
- an IMU BN080, gives the orientation of the ROV
- a current sensor (resistor + op AMP) to measure the current consumption of the Sub
- a voltage sensor for the battery voltage
- 3 temperatures sensors for the battery, the esc and the water 
- a pressure sensor for measuring depth

The board controls:
- 8 motors via 2 4in1 ESC using the D600 protocol
- 2 lights: front and bottom
- 2 fans: one for the esc and for the the deshumidifer 
- an oled screen to display sensors values

work in progress (TODO):
- a sonar for measuring distance
- a servo to add a gripper

### Power Delivery Board (PDB)
The PDB takes the input power from the battery and delivers it to the esc and the other boards. A voltage regulator is used to deliver 5V. 

#### The reed switch
To turn off the Sub, a reed (-magnetic) switch is used with a magnet. When a magnet is closed to this switch, it turns off the 3 mosfets of the PDB.  This is a simple solution to avoid using a mechanical switch that needs to be waterproof. 

### The battery
A 3S3P battery made of 18650 Liion battery is used to power the sub. This is plenty of power, around 7500 MAh. In idle mode, ie without the motors turning, the sub draws around 0.5A, when diving around 5 to 10 A. So with this battery the sub should last around an hour, but it needs to be tested. 

The battery is soldered with a $15 spot welder used with a 12V car battery. An BMS is used to controller the balance of the cell and add low voltage and over curretn safety. A temperature sensor is added on the SCB to monitor the battery temperature.

### Camera
A raspicam is added on the front of the tube for a live video feed. It is also possible to add a camera facing down on the bottom of the sub.

### The ethernet cable
This is the only way to transfert data on the long range, cat6 not shilded ethernet cable works fine.

## Mechanical assembly
The two main parts are the body and the tube.

### The Tube
The goal of the tube is to keep everything inside dry. An acrylic pipe of 80mm inside diameter and 300mm long with 2 aluminium flange are used to keep everything waterproof.

Remarqs: buying flange was my only opition to make the sub waterproof. Do not try to make your own 3D printed flange, i watched multiple videos on 3D printed waterproof parts, plot twist it doesnt work.

#### Back Flange
The back flange is a critical part of the rov. On the back plate, 4 connectors with 10 pins make the connection with the motors, the lights and the ethernet cable. A preasure sensor and a temperature sensor is also added. To make it fully waterproof, I filed it with epoxy resin.

#### Front Flange
The front flange is made of 8mm clear acrylic.

#### The Deshumidifier
To avoid condensed water that could block the camera view caused by the fresh water and the hot electronic inside the tube, a deshumidifier is added. Silica gel is added and a fan push the air to remove the humidity inside the tube


#### Testing the tube
Before testing the Sub in real condition it is important to check if the tube is waterproof. 
I created a preasure tester that consist of a thick PVC pipe closed with 2 plates. The tube (without the electronic) is put inside and the tester is put under preasure with a bycycle pump.I tested mine with a pressure of 6 bars (50m deapth) for 1 hour and it was a sucess.

### The Body of the sub

The body of the Sub consists of two 8mm acrylic plates laser cutted and mutliple 3D printed parts. All 3D printed parts are printed with PETG, it is more flexible and less britle than PLA. It is printed with 100% infill to avoid water to get inside. 

#### Thrusters
I used the cheapest brushless motors I could find. The lowest kv the better (960kv  works great) because it doesnt need to turn fast in the water.
Brushless motors do work under water but the bearing can wear and rust over time. I will see if i have any problem with the current motors but they are cheap to replace (4 euros).

I made my owm model for propeller but i will try to improve the design to make it more efficient.

#### The lights
8 lights are added, 4 in the front and 4 in the bottom for the cameras. They are made of 3W LED with 90° lens connected in series of 4 and powered by the 3S battery. Clear epoxy is used to make it waterproof.

### Checking buoyancy
The last step of the build is to adjust the buoyance of the sub so it slightly float. For this sub i added around 1L of air with Depron foam.

## The software
The sub is running ROS2 on the rapsberry. It is controlled by a computer rinning also ros2. I created my own programs in python to control the sub with a PS4 controller. More details in the ros folder.






