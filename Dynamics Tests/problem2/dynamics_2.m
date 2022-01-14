% This script has to be run to set up the parameters for the problem 2 in
% the Dynamics part. The simulink corresponding file is testReactionWheel.slx.
% The second section plots the results obtained in simulink
% Authors: Laura Train & Juan María Herrera

clear;clc;close all

% Add paths

addpath ../../Dynamics
addpath ../../

% Initial conditions
% orbital parameter
mu = 3.986e+14;

% orbit altitude and radius
h = 3000000;
RE = 6371000;
rmag = RE + h;

% orbital period
Torb = 2*pi*sqrt(rmag^3/mu);

% position and velocity -> assume Equatorial circular orbit
r0 = [rmag; 0; 0];
v0 = [0; sqrt(mu/rmag); 0];

% value to counteract by the reaction wheels
wz0 = deg2rad(0.5);
w0 = [0; 0; wz0];
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

% reaction wheels inertial tensor
Irw = [5.02e-5, 0, 0; 0, 9.41e-5, 0; 0, 0, 5.02e-5];

% value of the angular velocity impulse
wzrw = (Isc(3,3) + Irw(3,3))/Irw(3,3)* wz0;
twait = Torb/2;
t0 = 2*pi/wz0 * 3;
tramp = 10;

%%
% Plot set up

set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

figure()
plot(out.Torque.Time, out.Torque.Data(:,1),'r', ...
     out.Torque.Time, out.Torque.Data(:,2),'b', ...
     out.Torque.Time, out.Torque.Data(:,3),'g','LineWidth',2)
title("Torque for reaction wheels dynamics")
legend("$T_x$","$T_Y$","$T_Z$") 
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Torque [Nm]")
grid minor

figure()
plot(out.omega_B.Time, out.omega_B.Data(:,1),'r', ...
     out.omega_B.Time, out.omega_B.Data(:,2),'b', ...
     out.omega_B.Time, out.omega_B.Data(:,3),'g','LineWidth',2)
title("Angular velocity of the spacecraft")
legend("$\omega_x$","$\omega_y$","$\omega_z$") 
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Angular velocity [rad/s]")
grid minor


figure()
plot(out.omega_rw.Time, out.omega_rw.Data(:,1),'r', ...
     out.omega_rw.Time, out.omega_rw.Data(:,2),'b', ...
     out.omega_rw.Time, out.omega_rw.Data(:,3),'g','LineWidth',2)
title("Angular velocity of the reaction wheels")
legend("$\omega_x$","$\omega_y$","$\omega_z$") 
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Angular velocity [rad/s]")
grid minor

figure()
plot(out.euler_angles.Time, rad2deg(unwrap(out.euler_angles.Data(:,1))),'r',...
     out.euler_angles.Time, rad2deg(unwrap(out.euler_angles.Data(:,2))),'b', ...
     out.euler_angles.Time, rad2deg(unwrap(out.euler_angles.Data(:,3))),'g','LineWidth',2)
legend("yaw","pitch","roll")
title("Euler angles for reaction wheels dynamics")
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Euler angles [deg]")
grid minor