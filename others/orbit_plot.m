figure()
hold on
plot3(out.Dynamics.r_ECI.Data(1,1), out.Dynamics.r_ECI.Data(2,1), out.Dynamics.r_ECI.Data(3,1), 'r*', 'MarkerSize',12)
earth_sphere('km')
hold on
plot3(out.Dynamics.r_ECI.Data(1,:), out.Dynamics.r_ECI.Data(2,:), out.Dynamics.r_ECI.Data(3,:),'r','LineWidth',3)
grid minor

