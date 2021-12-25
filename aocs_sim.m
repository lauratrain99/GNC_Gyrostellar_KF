clear;clc;

buses = Simulink.data.dictionary.open('databus.sldd');

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

% geometric dimensions of the S/C
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
wn = 5*pi/Torb;
w0 = [0.1*wn; 0.1*wn; wn];
q0 = angle2quat(0,0,pi/2,'ZXZ');

% reaction wheels inertial tensor
Irw = [5.02e-5, 0, 0; 0, 9.41e-5, 0; 0, 0, 5.02e-5];

% magnetic field intensity [nT]
B0 = 30036.7;

% Gravity torque flag
flag_gravtorque = 0;

% Magnetometer
hyst = (60e-6)*0.001;
noiseDens = (30e-12)/6;

