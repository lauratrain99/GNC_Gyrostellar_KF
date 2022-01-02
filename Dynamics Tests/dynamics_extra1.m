% This script has to be run to set up the parameters for the extra problem 1 in
% the Dynamics part. The simulink corresponding file is testMagnetoField.slx.
% The second section plots the results obtained in simulink
% Authors: Laura Train & Juan María Herrera

clear;clc;close all

% Add paths

addpath ../Dynamics
addpath ../

% Initial conditions
% orbital parameter
mu = 3.986e+5;

% orbit altitude and radius
h = 3000;
RE = 6371;
rmag = RE + h;

% orbital period
Torb = 2*pi*sqrt(rmag^3/mu);

% orbital plane
a = RE + h;
e = 0.2;
Omega = 0;
inc = 30*pi/180;
omega = 0;
theta = 0;

% position and velocity initial conditions
[r0, v0] = coe2rv(mu, a, e, Omega, inc, omega, theta);

% value to counteract by the reaction wheels
wz0 = deg2rad(2);
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

% magnetic field intensity [microT]
B0 = 30.0367;

%% PLOT SETTING
% Default properties of plots
set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

% figure()
% plot(out.tout, rad2deg(out.Dynamics.omega_B.Data(1,:)),'r', ...
%      out.tout, rad2deg(out.Dynamics.omega_B.Data(2,:)),'b', ...
%      out.tout, rad2deg(out.Dynamics.omega_B.Data(3,:)),'g')
% title("Free torque motion Iz $>$ Iy $>$ Ix")
% legend("$\omega_x$","$\omega_y$","$\omega_z$") 
% xlabel("Time [s]")
% ylabel("Angular velocity in principal axes [deg/s]")
% grid minor
% 
% figure()
% plot(out.tout, out.Dynamics.quat.Data(1,:),'r', ...
%      out.tout, out.Dynamics.quat.Data(2,:),'b', ...
%      out.tout, out.Dynamics.quat.Data(3,:),'g', ...
%      out.tout, out.Dynamics.quat.Data(4,:),'k')
% legend("$q_0$","$q_1$","$q_2$","$q_3$")
% title("Free torque motion Iz $>$ Iy $>$ Ix")
% xlabel("Time [s]")
% ylabel("Quaternions")
% grid minor

