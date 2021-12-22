
h = 3000;
RE = 6371;
rmag = RE + h;
mu = 3.986e+5;
Torb = 2*pi*sqrt(rmag^3/mu)

% assume an Equatorial circular orbit
r0 = [rmag; 0; 0];
v0 = [0; sqrt(mu/rmag); 0];

m = 50;
w = 5;
h = 6;
d = 3;

Isc = [m/12 * (h^2 + d^2), 0, 0; 0, m/12 * (w^2 + d^2), 0; 0, 0, m/12 * (h^2 + w^2)];
Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);


