function [xx,yy,zz] = sphere2(varargin)
%SPHERE Generate sphere.
%   [X,Y,Z] = SPHERE(N) generates three (N+1)-by-(N+1)
%   matrices so that SURF(X,Y,Z) produces a unit sphere.
%
%   [X,Y,Z] = SPHERE uses N = 20.
%
%   SPHERE(N) and just SPHERE graph the sphere as a SURFACE
%   and do not return anything.
%
%   SPHERE(AX,...) plots into AX instead of GCA.
%
%   See also ELLIPSOID, CYLINDER.

%   Clay M. Thompson 4-24-91, CBM 8-21-92.
%   Copyright 1984-2002 The MathWorks, Inc. 

% Parse possible Axes input
narginchk(0,2);
[cax,args,nargs] = axescheck(varargin{:});
% Extract input arguments
n = 20;
d = [0,0,1];

if nargs > 0, n = args{1}; end
if nargs > 1, d = args{2}; end
d = d(:); % Make sure  d is vector.
% -pi <= theta <= pi is a row vector.
% -pi/2 <= phi <= pi/2 is a column vector.
theta = (-n:2:n)/n*pi;
phi = (-n:2:n)'/n*pi/2;
cosphi = cos(phi); cosphi(1) = 0; cosphi(n+1) = 0;
sintheta = sin(theta); sintheta(1) = 0; sintheta(n+1) = 0;

% Generate basic sphere
x = cosphi*cos(theta);
y = cosphi*sintheta;
z = sin(phi)*ones(1,n+1);

% Define rotation axis and angle
d0      = [0,0,1];

rotaxis = cross(d0,d);
if norm(rotaxis)==0
    rotaxis = [1,0,0];
end
rotaxis = rotaxis/norm(rotaxis);
angle   = -atan2(norm(cross(d0,d)),dot(d0,d));

% Rotation quaternion
q(1:3,1) = rotaxis*sin(angle/2);
q(4,1)   = cos(angle/2);

% Rotation matrix
Q   = [0, -q(3), q(2);q(3), 0, -q(1);-q(2), q(1), 0];
C   = eye(3)*(q(4)^2-q(1:3)'*q(1:3))+2*q(1:3)*q(1:3)'-2*q(4)*Q;

% Generate cylinder
x1      = reshape(x,1,size(x,1)*size(x,2));
y1      = reshape(y,1,size(y,1)*size(y,2));
z1      = reshape(z,1,size(z,1)*size(z,2));
M       = (C*[x1;y1;z1])';
X1      = M(:,1);
Y1      = M(:,2);
Z1      = M(:,3);
X       = reshape(X1,size(x,1),size(x,2));
Y       = reshape(Y1,size(y,1),size(y,2));
Z       = reshape(Z1,size(z,1),size(z,2));

%output
if nargout == 0
    cax = newplot(cax);
    surf(x,y,z,'parent',cax);
else
    xx = x; yy = y; zz = z;
end