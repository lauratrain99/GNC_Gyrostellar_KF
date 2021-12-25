clear;clc

%% PLOT SETTING
% Default properties of plots
set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

%% link to data bus
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
wn = 20*pi/Torb;
w0 = [wn*0.01; wn*0.01; wn];
q0 = angle2quat(0,0,0,'ZXZ');

%%
figure()
plot(out.tout, rad2deg(out.Dynamics.omega_B.Data(1,:)),'r', ...
     out.tout, rad2deg(out.Dynamics.omega_B.Data(2,:)),'b', ...
     out.tout, rad2deg(out.Dynamics.omega_B.Data(3,:)),'g')
title("Free torque motion Iz $>$ Iy $>$ Ix")
legend("$\omega_x$","$\omega_y$","$\omega_z$") 
xlabel("Time [s]")
ylabel("Angular velocity in principal axes [deg/s]")
grid minor

figure()
plot(out.tout, out.Dynamics.quat.Data(1,:),'r', ...
     out.tout, out.Dynamics.quat.Data(2,:),'b', ...
     out.tout, out.Dynamics.quat.Data(3,:),'g', ...
     out.tout, out.Dynamics.quat.Data(4,:),'k')
legend("$q_0$","$q_1$","$q_2$","$q_3$")
title("Free torque motion Iz $>$ Iy $>$ Ix")
xlabel("Time [s]")
ylabel("Quaternions")
grid minor

%%
% reaction wheels inertial tensor
Irw = [5.02e-5, 0, 0; 0, 9.41e-5, 0; 0, 0, 5.02e-5];

% magnetic field intensity [nT]
B0 = 30036.7;

