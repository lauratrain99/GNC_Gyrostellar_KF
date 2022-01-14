figure()
hold on
plot3(out.r_ECI.Data(1,1), out.r_ECI.Data(1,2), out.r_ECI.Data(1,3), 'r*', 'MarkerSize',12)
earth_sphere('m')
hold on
plot3(out.r_ECI.Data(:,1), out.r_ECI.Data(:,2), out.r_ECI.Data(:,3),'r','LineWidth',3)
grid minor

