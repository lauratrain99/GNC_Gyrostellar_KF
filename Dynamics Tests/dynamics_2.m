% This script has to be run to set up the parameters for the problem 2 in
% the Dynamics part. The simulink corresponding file is testReactionWheel.slx.
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

% position and velocity -> assume Equatorial circular orbit
r0 = [rmag; 0; 0];
v0 = [0; sqrt(mu/rmag); 0];

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

% reaction wheels inertial tensor
Irw = [5.02e-5, 0, 0; 0, 9.41e-5, 0; 0, 0, 5.02e-5];

% value of the angular velocity impulse
wzrw = (Isc(3,3) + Irw(3,3))/Irw(3,3)* wz0;
twait = 10*pi/wz0;
t0 = 1000;
tramp = 10;

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