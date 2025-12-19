# Lookup tables

Due to project time constraints - 3D LUT not tested in an ideal way. Could improve performance over 1D.

## LUT.m
Input: "training_data.csv"
Output: Boom_Cmd_LUT.mat, Boom_Vel_LUT.mat, Bucket_Cmd_LUT.mat, Bucket_Vel_LUT.mat
Create 1-D Lookup tables from collected data for use with Matlab 1-D lookup table block. 
Dependancy for master.slx.

## LUT_3D_V3.m
Input: "training_data.csv"
Output: 
    Boom_Vel_Vector, Boom_Ang_Vector, Stick_Ang_Vector_Boom, Boom_LUT_Data 
    Stick_Vel_Vector, Boom_Ang_Vector_Stick, Stick_Ang_Vector, Stick_LUT_Data
Create 3-D Lookup tables from collected data for use with Matlab 3-D lookup table block. 