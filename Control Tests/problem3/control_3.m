% This script has to be run to set up the parameters for the problem 3
% in the Control part. The simulink corresponding files are test_PID_SteadyRotation.slx,
% The second section plots the results obtained in simulink
% Authors: Laura Train & Juan María Herrera

clear;clc;close all;

% Add paths

addpath ../../Dynamics
addpath ../../Control
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

% Desired angular velocity
wz_desired = 5*pi/1000;

% System matrices
% state x = [wx, wy, wz];
% control u = [Tx, Ty, Tz];

A = zeros(3,3);
A(1,2) = (Iy - Iz)/Ix * wz0;
A(1,3) = (Iy - Iz)/Ix * wy0;
A(2,1) = (Iz - Ix)/Iy * wz0;
A(2,3) = (Iz - Ix)/Iy * wx0;
A(3,1) = (Ix - Iy)/Iz * wy0;
A(3,2) = (Ix - Iy)/Iz * wx0;

B = zeros(3,3);
B(1,1) = 1/Ix;
B(2,2) = 1/Iy;
B(3,3) = 1/Iz;

% the system is controllable if rank(P) = 3
P = ctrb(A,B);
rank(P)

% transfer function of the open loop system
num = [0, 0, 1];
den = [Iz, 0]; 
sys = tf(num, den);

% tune the PID parameters
PID_params = pidtune(sys,'PID');
Kp = PID_params.Kp;
Ki = PID_params.Ki;
Kd = PID_params.Kd;

% close loop system poles
num_cl = [Kd, Kp, Ki];
den_cl = [Iz + Kd, Kp, Ki];
sys_cl = tf(num_cl,den_cl);
[wn,zeta,p] = damp(sys_cl);

% root locus
rlocus(sys_cl)

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
plot(out.omegaZ_step.Time, wz_desired*ones(length(out.omegaZ_step.Time)),'b', ...
     out.omegaZ.Time, out.omegaZ.Data,'r','LineWidth',1)
ylim([0,])
title("Angular velocity in Z axis for Steady Rotation LQR Maneuver")
xlabel("Time [s]")
ylabel("Angular velocity [rad/s]")
grid minor


figure()
plot(out.yaw.Time, unwrap(out.yaw.Data),'b', ...
     out.yaw.Time, 900*ones(length(out.yaw.Data),1),'r--')
title("Yaw angle for Steady Rotation LQR Maneuver")
xlabel("Time [s]")
ylabel("Yaw angle [deg]")
grid minor