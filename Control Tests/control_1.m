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
wn = 10*pi/Torb;
w0 = [0; 0; 0];
q0 = angle2quat(0,0,0,'ZXZ');

%% Control requirements
Fmax = 25e-3;
Isp = 60;
Larm = 5e-2;
% tmin = 2e-3;
t_thrust = 1;

yaw = 45;
pitch = 30;
roll = 15;

Tmax = Fmax*Larm;

wz = Tmax*t_thrust/Iz;
tz = deg2rad(yaw)/wz;
wy = Tmax*t_thrust/Iy;
ty = deg2rad(pitch)/wy;
wx = Tmax*t_thrust/Ix;
tx = deg2rad(roll)/wx;

t0z = 10;
tfz = t0z + tz;
t0y = tfz + 10;
tfy = t0y + ty;
t0x = tfy + 10;
tfx = t0x + tx;





