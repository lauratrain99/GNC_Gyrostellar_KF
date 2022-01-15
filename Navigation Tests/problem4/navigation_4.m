% This script has to be run to set up the parameters for the problem 4
% in the Navigation part. The simulink corresponding file is testNavigationSol.slx,
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
%CASE 1
wn = 0;
w0 = [0; 0; wn];
euler0 =[pi/2, 0, pi/4];
q0 = angle2quat(euler0(3),euler0(2),euler0(1),'ZYX');

%CASE 2
%wn = deg2rad(2);
%w0 = [0; 0; wn];
% euler0 =[0, 0, 0];
%q0 = angle2quat(0,0,0,'ZYX');

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


%IMU
noiseAcc = 0.33*((((0.07/60)/0.01)*10^(-3))^2); %g^2/Hz
noiseAng =  0.33*((0.15/60)^2); %(deg/s)^2/Hz
biasAcc = 0.33*((0.00004)^2)/(2*pi); %g^2
biasAng = 0.33*((0.3/3600)^2)/(2*pi);%(deg/s)^2

%StarTracker
noiseNEA = 0.33*((0.55*pi/(3600*180))^2); %rad^2/Hz

%% 
% Plot set up

set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

figure()
plot(out.error_euler.Time, out.error_euler.Data(:,1),'r', ...
     out.error_euler.Time, out.error_euler.Data(:,2),'b', ...
     out.error_euler.Time, out.error_euler.Data(:,3),'g')
title("Navigation solution: error in Euler angles")
legend("yaw","pitch","roll") 
xlabel("Time [s]")
ylabel("Error in Euler angles [deg]")
grid minor

figure()
plot(out.error_omega.Time, out.error_omega.Data(:,1),'r', ...
     out.error_omega.Time, out.error_omega.Data(:,2),'b', ...
     out.error_omega.Time, out.error_omega.Data(:,3),'g')
title("Navigation solution: error in angular velocity")
legend("$\omega_x$","$\omega_y$","$\omega_z$") 
xlabel("Time [s]")
ylabel("Error in angular velocity [rad/s]")
grid minor


