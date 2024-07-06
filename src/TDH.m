function TDH = TDH(dh)
a = dh(1);
alp = dh(2);
d = dh(3);
th = dh(4);
%UNTITLED4 Summary of this function goes here
% Detailed explanation goes here
TDH = [cos(th),-sin(th),0,a;
    sin(th)*cos(alp),cos(th)*cos(alp),-sin(alp),-d*sin(alp);...
sin(th)*sin(alp),cos(th)*sin(alp),cos(alp),d*cos(alp);
0,0,0,1];
end