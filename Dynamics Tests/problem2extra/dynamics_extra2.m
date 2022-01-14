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

% assume a perturbation a 1% perturbation wrt Z axis angular velocity
wn = 20*pi/Torb;
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


figure()
plot(out.omega_B.Time, out.omega_B.Data(:,1),'r', ...
     out.omega_B.Time, out.omega_B.Data(:,2),'b', ...
     out.omega_B.Time, out.omega_B.Data(:,3),'g','LineWidth',2)
title("Angular Velocity for Free torque motion Boom")
legend("$\omega_x$","$\omega_y$","$\omega_z$") 
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Angular velocity [rad/s]")
grid minor

figure()
plot(out.quat.Time, out.quat.Data(:,1),'r', ...
     out.quat.Time, out.quat.Data(:,2),'b', ...
     out.quat.Time, out.quat.Data(:,3),'g', ...
     out.quat.Time, out.quat.Data(:,4),'k','LineWidth',2)
legend("$q_0$","$q_1$","$q_2$","$q_3$")
title("Quaternions for Free torque motion")
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Quaternions")
grid minor

figure()
plot(out.r_ECI.Time, out.r_ECI.Data(:,1),'r', ...
     out.r_ECI.Time, out.r_ECI.Data(:,2),'b', ...
     out.r_ECI.Time, out.r_ECI.Data(:,3),'g','LineWidth',2)
legend("x","y","z")
title("Inertial position for Free torque motion Boom")
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Position [m]")
grid minor

figure()
plot(out.v_ECI.Time, out.v_ECI.Data(:,1),'r', ...
     out.v_ECI.Time, out.v_ECI.Data(:,2),'b', ...
     out.v_ECI.Time, out.v_ECI.Data(:,3),'g','LineWidth',2)
legend("$v_x$","$v_y$","$v_z$")
title("Inertial velocity for Free torque motion Boom")
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Velocity [m/s]")
grid minor

figure()
plot(out.euler_angles.Time, rad2deg(unwrap(out.euler_angles.Data(:,1))),'r',...
     out.euler_angles.Time, rad2deg(unwrap(out.euler_angles.Data(:,2))),'b', ...
     out.euler_angles.Time, rad2deg(unwrap(out.euler_angles.Data(:,3))),'g','LineWidth',2)
legend("yaw","pitch","roll")
title("Euler angles for Free torque motion Boom")
xlabel("Time [s]")
xlim([0,Torb])
ylabel("Euler angles [deg]")
grid minor