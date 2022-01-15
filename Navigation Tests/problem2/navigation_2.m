% This script has to be run to set up the parameters for the problem 2
% in the Navigation part. The simulink corresponding files are testIMU.slx,
% testMagnetometer.slx, testStarTracker.slx
% Authors: Laura Train & Juan María Herrera

clear;clc;close all;

% Add paths

addpath ../../Navigation

%IMU
noiseAcc =(0.07/(60*9.81))^2; %g^2/Hz
noiseAng =(0.15/60)^2; %(deg/s)^2/Hz
biasAcc = ((0.00004)^2)/(2*pi); %g^2
biasAng = ((0.3/3600)^2)/(2*pi);%(deg/s)^2

%StarTracker
noiseNEA =(0.55*pi/(3600*180))^2; %rad^2/Hz