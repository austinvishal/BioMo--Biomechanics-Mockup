function rev_joint_axis(r,axis,N,scale,rot_matrix,trans_vec,facecolo,edgecolo,facecolor)
[x0,y0,z0] = cylinder2(r,axis,N); %k4c parallel to k2c
axis
 x0=x0*scale-axis(1)*scale/2;
 y0=y0*scale-axis(2)*scale/2;
 z0=z0*scale-axis(3)*scale/2;
% z0=z0*0.5;
%    z0=z0*2;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
        cylin = [rot_matrix ,trans_vec'; 0 0 0 1]*[x0(i,:); y0(i,:); z0(i,:); ones(1,21)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
ts2=surf(x1,y1,z1,'FaceColor',facecolo,'Facealpha',0.6, ...
'EdgeColor',edgecolo);
%  camlight
%   light('Position',[2 10 10],'Style','infinite')
   lighting gouraud
p2=patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:20],'FaceColor',facecolor, ...
'EdgeColor','none','AmbientStrength',0.3);
p3=patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:20],'FaceColor',facecolor, ...
'EdgeColor','none','AmbientStrength',0.3);
end