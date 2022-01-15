% This script has to be run to set up the parameters for the problem 1
% in the Navigation part. The simulink corresponding file is testSensors.slx,
% Authors: Laura Train & Juan María Herrera

clear;clc;close all;

% Add paths

addpath ../../Navigation

% Magnetometer
hyst = (60e-6)*0.001; %T
noiseDens = ((50e-12)^2); %T^2/Hz

%IMU
noiseAcc =(0.07/(60*9.81))^2; %g^2/Hz
noiseAng =(0.15/60)^2; %(deg/s)^2/Hz
biasAcc = ((0.00004)^2)/(2*pi); %g^2
biasAng = ((0.3/3600)^2)/(2*pi);%(deg/s)^2

%StarTracker
noiseNEA =(0.55*pi/(3600*180))^2; %rad^2/Hz
