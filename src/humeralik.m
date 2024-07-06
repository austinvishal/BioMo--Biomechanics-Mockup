function [th1,th2,th3,th4,th5,normal_hum,thn,Z03hat,X35hat,gam]= humeralik(X05hat,X00hat,r,d3)
% given pose, or pointing direction ie the components of 0X5_hat =[x5 ;y5; z5]
% X05hat given for IK find angles
normal_hum= (cross(X05hat,-X00hat))/norm(cross(X05hat,-X00hat))
gam= acos(dot(X05hat,-X00hat))
% r=2; % or 3
gam1=gam/(r+1)
gam2=gam1/r;
x5=X05hat(1);
y5=X05hat(2);
z5=X05hat(3);
% now we know, gam, gam1 and normal to the humeral plane
% we can determine 0Z3_hat (x3,y3,z3) shoulder girdle pointing direction
thn=atan2(y5,-z5)
Z03hat=[-cos(gam1); sin(gam1)*sin(thn); -sin(gam1)*cos(thn)];
x3=Z03hat(1);
y3=Z03hat(2);
z3=Z03hat(3);
%once the inner shoulder direction is defined, using the forward kinematic
%map of T03, we can solve for IK
% determination first joint angle using IK
% th2=acos(-Z03hat(3))
% th1=atan2((Z03hat(2)/sin(th2)),-(Z03hat(1)/sin(th2)))
th2=acos(-z3)
th1=atan2((y3/sin(th2)),-(x3/sin(th2)))

%th3 must be optimized considering the performance of parallel mechanism
th3=-pi/3;   % refer kinematic design of a humanoid robotic shoulder complex for the derivaiton

%once th3 is known, the direction cosine R03 is known FROM fk
T03=[  cos(th1)*cos(th2)*cos(th3) - sin(th1)*sin(th3), - cos(th3)*sin(th1) - cos(th1)*cos(th2)*sin(th3), -cos(th1)*sin(th2), -d3*cos(th1)*sin(th2)
- cos(th1)*sin(th3) - cos(th2)*cos(th3)*sin(th1),   cos(th2)*sin(th1)*sin(th3) - cos(th1)*cos(th3),  sin(th1)*sin(th2),  d3*sin(th1)*sin(th2)
                              -cos(th3)*sin(th2),                                sin(th2)*sin(th3),          -cos(th2),          -d3*cos(th2)
                                               0,                                                0,                  0,                     1];

R03=T03(1:3,1:3)
X35hat=transpose(R03)*X05hat; % pointing direction of humerus with respect to shoulder center

%this can be equated with T35 first column to determine the angles th4 th5
th5=asin(-x5*(cos(th1)*cos(th2)*sin(th3)+sin(th1)*cos(th3))+y5*(sin(th1)*cos(th2)*sin(th3)-cos(th1)*cos(th3))+z5*sin(th2)*sin(th3));

s4=(-x5*cos(th1)*sin(th2)+y5*sin(th1)*sin(th2)-z5*cos(th2))/(cos(th5));
c4=(x5*(cos(th1)*cos(th2)*cos(th3)-sin(th1)*sin(th3))-y5*(sin(th1)*cos(th2)*cos(th3)+cos(th1)*sin(th3))-z5*sin(th2)*cos(th3))/(cos(th5));
th4=atan2(s4,c4);
%th2 th1 th3 th5 th4 completes the ik
end