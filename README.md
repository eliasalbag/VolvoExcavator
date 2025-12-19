# Volvo Excavator
A custom control system designed for Volvo excavator EC20E, enabling simplified and remote operation.

Open ./VolvoExcavator.prj to load project parameters

## master.slx
Matlab 2025a
Dependant on ./lookupTable/, ./Scripts_Data/Excavator_Parameters.m
Uses ./velocity_controller/ for control

CAN via kvaser needs to be setup on startup -- **Choose correct device in CAN configuration blocks**

### Encoder calibration with ./Scripts_Data/calculateAnglesFromLengths.m
Measure distance between hydraulic links, read encoder data. Use this to calculate the calibration offset for ./master.slx/CalibrationOffsets[1,2,3]
See ./notation.gif for link notation

## ./models/SimExcavator.slx
Matlab 2025a
Dependant on ./Scripts_Data/Excavator_Parameters.m
Hydraulic simulation system. For proof-of-concept controller testing. 


## Neural Network
See ./neural_network/readme.md
