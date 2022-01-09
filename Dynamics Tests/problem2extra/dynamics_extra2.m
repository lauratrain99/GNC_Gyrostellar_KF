% This script has to be run to set up the parameters for the extra problem 2 in
% the Dynamics part. The simulink corresponding file is testBoom.slx.
% The second section plots the results obtained in simulink
% Authors: Laura Train & Juan María Herrera

clear;clc;close all

% Add paths

addpath ../../Dynamics
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
wn = 10*pi/Torb;
w0 = [wn*0.01; wn*0.01; wn];
q0 = angle2quat(0,0,0,'ZYX');

% Geometric and massic properties
m = 10;
R = 0.1;
L = 0.304*2;

% inertia tensor of the S/C
Isc = [1/4 * m * R^2 + 1/12 * m * L^2, 0, 0; 0, 1/4 * m * R^2 + 1/12 * m * L^2 , 0; 0, 0, 1/2 * m * R^2];
Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);

%%
% Plot set up

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