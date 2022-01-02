clear;clc

%% PLOT SETTING
% Default properties of plots
set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

%% link to data bus

addpath ../Control
addpath ../Dynamics
addpath ../

buses = Simulink.data.dictionary.open('databus.sldd');

%% parameters
% orbital parameter
mu = 3.986e+5;

% orbit altitude and radius
h = 3000;
RE = 6371;
rmag = RE + h;

% orbital period
Torb = 2*pi*sqrt(rmag^3/mu);

% assume an Equatorial circular orbit
r0 = [rmag; 0; 0];
v0 = [0; sqrt(mu/rmag); 0];

%% Test no gravity torque
% Iz>Iy>Ix stable configuration. w>h>d
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

% attitude initial conditions. 
wx0 = 0;
wy0 = 0;
wz0 = 0;
w0 = [wx0; wy0; wz0];
q0 = angle2quat(0,0,0,'ZXZ');

%% Control requirements
Fmax = 25e-3;
Isp = 60;
Larm = 5e-2;
% tmin = 2e-3;
t_thrust = 1;

wz_desired = 20*pi/Torb;

Tmax = Fmax*Larm;

% state x = [roll, pitch, yaw, wx, wy, wz];

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

% y = [roll, pitch, yaw];

num = [0, 0, 1];

den = [Iz, 0]; 

sys = tf(num, den);

PID_params = pidtune(sys,'PID');

Kp = PID_params.Kp;
Ki = PID_params.Ki;
Kd = PID_params.Kd;


num_cl = [Kd, Kp, Ki];
den_cl = [Iz + Kd, Kp, Ki];

sys_cl = tf(num_cl,den_cl);

rlocus(sys_cl)
[wn,zeta,p] = damp(sys_cl);

% 
% ask Sanjurjo if he wants us to compute these values, does it make sense
% for systems with poles > 1?
% td = ;
% tr = ;
% tp = ;
% Mp = ;
% ts = ;