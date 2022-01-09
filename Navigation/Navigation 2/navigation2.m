% Magnetometer
hyst = (60e-6)*0.001;
noiseDens = ((50e-12)^2)*40;

%IMU
noiseDensAcc = 0.33*40*((((0.07/60)/0.01)*10^(-3))^2);
noiseDensAng =  0.33*40*((0.15/60)^2);
biasAcc = 0.33*((0.00004)^2)/(2*pi);
biasAng = 0.33*((0.3/3600)^2)/(2*pi);

%StarTracker
noiseNEA = 0.33*10*((0.55*pi/(3600*180))^2);