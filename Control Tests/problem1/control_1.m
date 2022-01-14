% This script has to be run to set up the parameters for the problem 1
% in the Control part. The simulink corresponding files are testOpenLoop.slx,
% testOpenLoopNoisy.slx
% The second section plots the results obtained in simulink
% Authors: Laura Train & Juan María Herrera

clear;clc;close all;

% Add paths

addpath ../../Dynamics
addpath ../../Control
addpath ../../

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
w0 = [0; 0; 0];
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

% Control requirements
% Thruster specifications
Fmax = 25e-3;
Isp = 60;
Larm = 5e-2;
% tmin = 2e-3;
t_thrust = 1;
Tmax = Fmax*Larm;

% Desired angles
yaw = 45;
pitch = 30;
roll = 15;

% desired angular velocities and times between burns
wz = Tmax*t_thrust/Iz;
tz = deg2rad(yaw)/wz;
wy = Tmax*t_thrust/Iy;
ty = deg2rad(pitch)/wy;
wx = Tmax*t_thrust/Ix;
tx = deg2rad(roll)/wx;

% initial and final times for each thruster
t0z = 10;
tfz = t0z + tz;
t0y = tfz + 10;
tfy = t0y + ty;
t0x = tfy + 10;
tfx = t0x + tx;

%% 

% Plot set up

set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

figure()
plot(out.Torque.Time, out.Torque.Data(:,1),'r', ...
     out.Torque.Time, out.Torque.Data(:,2),'b', ...
     out.Torque.Time, out.Torque.Data(:,3),'g')
title("Torque values for Open Loop Maneuver")
legend("$T_x$","$T_y$","$T_z$") 
xlabel("Time [s]")
ylabel("Torque [Nm]")
grid minor


figure()
plot(out.omega.Time, out.omega.Data(:,1),'r', ...
     out.omega.Time, out.omega.Data(:,2),'b', ...
     out.omega.Time, out.omega.Data(:,3),'g','LineWidth',2)
title("Angular Velocities for Open Loop Maneuver")
legend("$\omega_x$","$\omega_y$","$\omega_z$") 
xlabel("Time [s]")
ylabel("Angular velocity [rad/s]")
grid minor


figure()
plot(out.euler_angles.Time, out.euler_angles.Data(:,1),'g', ...
     out.euler_angles.Time, out.euler_angles.Data(:,2),'b', ...
     out.euler_angles.Time, out.euler_angles.Data(:,3),'r','LineWidth',2)
title("Euler Angles for Open Loop Maneuver")
legend("yaw","pitch","roll") 
xlabel("Time [s]")
ylabel("Euler angles [deg]")
% ylim([-5,50])
grid minor

