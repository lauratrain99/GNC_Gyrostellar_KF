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
q0 = angle2quat(0,0,0,'ZYX');

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

% check if matrix is controllable
P = ctrb(A,B);
rank(P)

% yaw control

numZ = [0, 0, 1];
denZ = [Iz, 0, 0]; 
sysZ = tf(numZ, denZ);
PID_paramsZ = pidtune(sysZ,'PID');
KpZ = PID_paramsZ.Kp;
KiZ = PID_paramsZ.Ki;
KdZ = PID_paramsZ.Kd;

% pitch control

numY = [0, 0, 1];
denY = [Iy, 0, 0]; 
sysY = tf(numY, denY);
PID_paramsY = pidtune(sysY,'PID');
KpY = PID_paramsY.Kp;
KiY = PID_paramsY.Ki;
KdY = PID_paramsY.Kd;

% roll control

numX = [0, 0, 1];
denX = [Ix, 0, 0]; 
sysX = tf(numX, denX);
PID_paramsX = pidtune(sysX,'PID');
KpX = PID_paramsX.Kp;
KiX = PID_paramsX.Ki;
KdX = PID_paramsX.Kd;

Kp = [KpX,0,0;0,KpY,0;0,0,KpZ];
Ki = [KiX,0,0;0,KiY,0;0,0,KiZ];
Kd = [KdX,0,0;0,KdY,0;0,0,KdZ];

% num_cl = [Kd, Kp, Ki];
% den_cl = [Iz, Kd, Kp, Ki];
% 
% sys_cl = tf(num_cl,den_cl);
% 
% rlocus(sys_cl)
% [wn,zeta,p] = damp(sys_cl);


