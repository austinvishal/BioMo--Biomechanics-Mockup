function [q,qdot,qddot]= makequinticjtraj(to,tf,q0,qf,qdoto,qdotf,qddoto,qddotf)
 
% Coefficient matrix for quintic trajectory and its derivative
% at initial and final values.
A = [1,  to,  to^2, to^3,   to^4,    to^5;   ...
     0,  1,   2*to, 3*to^2, 4*to^3,  5*to^4;   ...
     0,  0,   2,    6*to,   12*to^2, 20*to^3; ...
     1,  tf,  tf^2, tf^3,   tf^4,    tf^5;   ...
     0,  1,   2*tf, 3*tf^2, 4*tf^3,  5*tf^4;   ...
     0,  0,   2,    6*tf,   12*tf^2, 20*tf^3];
% Vector of intial and final joint positions and velocities
b = [qo; qdoto; qddoto; qf; qdotf; qddotf];
% Compute coefficients of trajectory polynomial using
% notion of a = inv(A)*b, but using Gaussian Elimination
a = A\b;
% Evaluate quintic polynomial at times for plotting
t = linspace(to, tf, 501);
q     = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;
qdot  = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
qddot = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
% Plot trajectories
% subplot(2,2,2);
plot(t,q,'b-',t,qdot,'g--',t,qddot,'r-.','LineWidth',2);
legend('pos','vel','acc');
xlabel('time (sec)'); ylabel('Trajectory');
title('Trajectory using Quintic Polynomial');
grid;
theta0,thetaf,thetad0,thetadf,thetadd0,thetaddf,tstart,tfinal
createTraj5(theta0,thetaf,thetad0,thetadf,thetadd0,thetaddf,tstart,tfinal)
end