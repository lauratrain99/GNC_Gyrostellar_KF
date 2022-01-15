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
wn = 5*pi/Torb;
w0 = [0.1*wn; 0.1*wn; wn];
q0 = angle2quat(0,0,pi/2,'ZXZ');

% reaction wheels inertial tensor
Irw = [5.02e-5, 0, 0; 0, 9.41e-5, 0; 0, 0, 5.02e-5];

% magnetic field intensity [nT]
B0 = 30036.7;

% Magnetometer
hyst = (60e-6)*0.001; %T
noiseMag = ((50e-12)^2)*40; %T^2/Hz

%IMU
noiseAcc = 0.33*((((0.07/60)/0.01)*10^(-3))^2); %g^2/Hz
noiseAng =  0.33*((0.15/60)^2); %(deg/s)^2/Hz
biasAcc = 0.33*((0.00004)^2)/(2*pi); %g^2
biasAng = 0.33*((0.3/3600)^2)/(2*pi);%(deg/s)^2

%StarTracker
noiseNEA = 0.33*((0.55*pi/(3600*180))^2); %rad^2/Hz