% This script has to be run to set up the parameters for the problem 3 extra
% in the Control part. The simulink corresponding files are test_ControlNavigation.slx,
% The second section plots the results obtained in simulink
% Authors: Laura Train & Juan María Herrera

clear;clc;close all;

% Add paths

addpath ../../Dynamics
addpath ../../Control
addpath ../../Navigation
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
wx0 = 0;
wy0 = 0;
wz0 = 0;
w0 = [wx0; wy0; wz0];
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

% desired angles
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

% System matrices
% state x = [roll, pitch, yaw, wx, wy, wz];
% control u = [Tx, Ty, Tz];

A = zeros(6,6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,5) = (Iy - Iz)/Ix * wz0;
A(4,6) = (Iy - Iz)/Ix * wy0;
A(5,4) = (Iz - Ix)/Iy * wz0;
A(5,6) = (Iz - Ix)/Iy * wx0;
A(6,4) = (Ix - Iy)/Iz * wy0;
A(6,5) = (Ix - Iy)/Iz * wx0;

B = zeros(6,3);
B(4,1) = 1/Ix;
B(5,2) = 1/Iy;
B(6,3) = 1/Iz;

% the system is controllable if rank(P) = 6
P = ctrb(A,B);
rank(P)

% yaw transfer function
numZ = [0, 0, 1];
denZ = [Iz, 0, 0]; 
sysZ = tf(numZ, denZ);
% yaw PID values
PID_paramsZ = pidtune(sysZ,'PID');
KpZ = PID_paramsZ.Kp;
KiZ = PID_paramsZ.Ki;
KdZ = PID_paramsZ.Kd;

% pitch transfer function
numY = [0, 0, 1];
denY = [Iy, 0, 0]; 
sysY = tf(numY, denY);
% pitch PID values
PID_paramsY = pidtune(sysY,'PID');
KpY = PID_paramsY.Kp;
KiY = PID_paramsY.Ki;
KdY = PID_paramsY.Kd;

% roll transfer function
numX = [0, 0, 1];
denX = [Ix, 0, 0]; 
sysX = tf(numX, denX);
% roll PID values
PID_paramsX = pidtune(sysX,'PID');
KpX = PID_paramsX.Kp;
KiX = PID_paramsX.Ki;
KdX = PID_paramsX.Kd;

% set up Kp,Ki,Kd overall matrices
Kp = [KpX,0,0;0,KpY,0;0,0,KpZ];
Ki = [KiX,0,0;0,KiY,0;0,0,KiZ];
Kd = [KdX,0,0;0,KdY,0;0,0,KdZ];

% IMU
noiseAcc = 0.33*40*((((0.07/60)/0.01)*10^(-3))^2);
noiseAng =  0.33*40*((0.15/60)^2);
biasAcc = 0.33*((0.00004)^2)/(2*pi);
biasAng = 0.33*((0.3/3600)^2)/(2*pi);

% StarTracker
noiseNEA = 0.33*10*((0.55*pi/(3600*180))^2);

%%
% Plot set up

set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

figure()

plot(out.euler_angles_nav.Time, out.euler_angles_nav.Data(:,1),'r', ...
     out.euler_angles_nav.Time, out.euler_angles_nav.Data(:,2),'b', ...
     out.euler_angles_nav.Time, out.euler_angles_nav.Data(:,3),'g', 'LineWidth',2)
title("Euler angles including navigation")
legend("yaw nav","pitch nav","roll nav") 
xlabel("Time [s]")
ylabel("Euler angles [deg]")
grid minor


figure()
plot(out.euler_angles.Time, out.errors.Data(:,1),'r', ...
     out.euler_angles.Time, out.errors.Data(:,2),'b', ...
     out.euler_angles.Time, out.errors.Data(:,3),'g')
title("Error in Euler angles with Navigation")
legend("yaw","pitch","roll") 
xlabel("Time [s]")
ylabel("Euler angles [deg]")
grid minor