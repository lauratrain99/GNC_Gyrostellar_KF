clear;clc

%% PLOT SETTING
% Default properties of plots
set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

%% link to data bus

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

a = RE + h;
e = 0.2;
Omega = 0;
inc = 30*pi/180;
omega = 0;
theta = 0;

[r0, v0] = coe2rv(mu, a, e, Omega, inc, omega, theta);

%% Test no gravity torque
% Iz>Iy>Ix stable configuration. w>h>d
m = 100;
w = 5;
h = 3;
d = 2;

% inertia tensor of the S/C
Isc = [m/12 * (h^2 + d^2), 0, 0; 0, m/12 * (w^2 + d^2), 0; 0, 0, m/12 * (h^2 + w^2)];
Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);

% attitude initial conditions. 
wn = 10*pi/Torb;
w0 = [wn*0.01; wn*0.01; wn];
q0 = angle2quat(0,0,0,'ZXZ');

% magnetic field intensity [microT]
B0 = 30.0367;
