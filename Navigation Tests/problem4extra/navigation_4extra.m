% This script has to be run to set up the parameters for the problem 4extra
% in the Navigation part. The simulink corresponding file is testGyrostellar_KF.slx,
% Authors: Laura Train & Juan María Herrera

clear;clc;close all;

% Add paths

addpath ../../Dynamics
addpath ../../Navigation

% Initial conditions
% orbital parameter
mu = 3.986e+5;

% orbit altitude and radius
h = 3000;
RE = 6371;
rmag = RE + h;

% orbital period
Torb = 2*pi*sqrt(rmag^3/mu);

% position and velocity -> assume Equatorial circular orbit
r0 = [rmag; 0; 0];
v0 = [0; sqrt(mu/rmag); 0];

% assume a perturbation a 1% perturbation wrt Z axis angular velocity
wn = 10*pi/Torb;
w0 = [wn*0.01; wn*0.01; wn];
q0 = angle2quat(0,0,0,'ZYX');

% Geometric and massic properties
% Iz>Iy, Iz>Ix stable configuration. h>w>d
% geometric dimensions of the S/C
m = 10;
w = 0.2;
h = 0.304;
d = 0.1;

% inertia tensor of the S/C
Isc = [m/12 * (h^2 + d^2), 0, 0; 0, m/12 * (w^2 + d^2), 0; 0, 0, m/12 * (h^2 + w^2)];
Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);

% IMU
noiseDensAcc = 0.33*40*((((0.07/60)/0.01)*10^(-3))^2);
noiseDensAng =  0.33*40*((0.15/60)^2);
biasAcc = 0.33*((0.00004)^2)/(2*pi);
biasAng = 0.33*((0.3/3600)^2)/(2*pi);

% StarTracker
noiseNEA = 0.33*10*((0.55*pi/(3600*180))^2);

gyro_std = [sqrt(deg2rad(noiseDensAng));sqrt(deg2rad(noiseDensAng));sqrt(deg2rad(noiseDensAng)) ];
gyro_bias_init = [sqrt(deg2rad(biasAng));sqrt(deg2rad(biasAng));sqrt(deg2rad(biasAng)) ];
init_align_error = deg2rad([1;2;2]);
str_std =[sqrt(deg2rad(noiseNEA));sqrt(deg2rad(noiseNEA));sqrt(deg2rad(noiseNEA)) ];

% gyro_std = [1;1;1];
% gyro_bias_init = [1;1;1];
% init_align_error = deg2rad([1;1;2]);
% str_std =[1;1;1];

