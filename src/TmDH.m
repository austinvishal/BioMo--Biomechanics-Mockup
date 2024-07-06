function TmDH = TmDH(mDH)
d = mDH(1);
alp = mDH(2);
r = mDH(3);
th = mDH(4);
%UNTITLED4 Summary of this function goes here
% Detailed explanation goes here
TmDH = [cos(th),-sin(th),0,d;
    sin(th)*cos(alp),cos(th)*cos(alp),-sin(alp),-r*sin(alp);...
sin(th)*sin(alp),cos(th)*sin(alp),cos(alp),r*cos(alp);
0,0,0,1];
end

