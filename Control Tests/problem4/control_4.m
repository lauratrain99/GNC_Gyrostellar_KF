% This script has to be run to set up the parameters for the problem 4
% in the Control part. The simulink corresponding files are test_LQR_SteadyRotation.slx,
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
wz_desired = 20*pi/Torb;

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

% LQR design
% estimate the matrices Q,R,N
Q = 1e-3 * eye(3); 
R = 1e-4 * eye(3); 
N = zeros(3); 

% compute the K matrix of the LQR
[K_LQR,~,~] = lqr(A,B,Q,R,N); 

% check the poles
damp(A - B*K_LQR)
