function sphere_joint_axis(r,axis,scale,rot_matrix,trans_vec,facecolo,edgecolo)
[x0,y0,z0] =sphere2(r,axis);
  x0=x0*scale;
 y0=y0*scale;
z0=z0*scale;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays

for i = 1:11
        sph = [rot_matrix trans_vec'; 0 0 0 1]*[x0(i,:); y0(i,:); z0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; sph(1,:)];
    y1 = [y1; sph(2,:)];
    z1 = [z1; sph(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolo,'Facealpha',0.6, ...
'EdgeColor',edgecolo,'AmbientStrength',0.4)
end