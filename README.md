# Building the Application

1. Download and unpack "A1 SDK for Linux ARMv7" XC111-version. (If you are using another connector board (e.g. XC112) use the SDK for your connector board and modify makefile_build_waste_level.inc to use the libraries for your connector board.)

2. Set up the development environment according to the instructions in the readme file in the root of the unpacked SDK.

3. Copy the directories user_rule and user_source to the root of the unpacked SDK.

4. Open a terminal window and go to the root of the unpacked SDK. Type "make" to build the reference application and other programs shipped with the SDK.

5. An executable called "ref-app-parking" is now created in the out directory

# Using the Application 
The program outputs a 0 or 1 for every measurement it does, depending on whether there is a car or the spot is empty.

1. Calibration – record reflections from an empty parking spot by typing "./out/ref-app-parking -c".
During the calibration the application record envelope data and stores it in the file "parking.cal". These values are later used to calculate a threshold vector.

2. Once the calibration data is recorded you can start measurements by typing "./out/ref-app-parking -f parking.cal". One measurement will be done and then the program will print its result ("Car detected"/"Nothing detected") and exit. 

## Options
- It is possible to make several measurements (independent of algorithm used) with a time delay in between by typing "./out/ref-app-parking -d <time_delay_in_seconds>". The application will make at least two measurements, but will continue until the two latest results are equal before it exits. This means that if someone is passing under the sensor when doing a measurement, and the following measurement is empty, this will trigger a third measurement and so on.

- The default start range is set to 12 cm, but you can easy change the start range by typing "./out/ref-app-parking -c -a <distance_in_metres>" during calibration. If the start range for the calibration and the start range set when running differs, the start range will be set to the calibration start range. 

- The default sensor is 1, but this can easily be changed by typing "./out/ref-app-parking -s <number_of_sensor>"

Example:
```
pi@rpi:~/evk/out/ref-app-parking -f parking.cal -a 0.2 -d 5
Start range: 0.200000
0
1
1

Car detected.
```
