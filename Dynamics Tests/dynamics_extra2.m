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

% assume an Equatorial circular orbit
r0 = [rmag; 0; 0];
v0 = [0; sqrt(mu/rmag); 0];

%% Test no gravity torque
% Iz>Iy>Ix stable configuration. w>h>d
m = 10;
R = 0.1;
L = 0.304*2;

% inertia tensor of the S/C
Isc = [1/4 * m * R^2 + 1/12 * m * L^2, 0, 0; 0, 1/4 * m * R^2 + 1/12 * m * L^2 , 0; 0, 0, 1/2 * m * R^2];
Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);

% attitude initial conditions. 
wn = 10*pi/Torb;
w0 = [wn*0.01; wn*0.01; wn];
q0 = angle2quat(0,0,0,'ZXZ');