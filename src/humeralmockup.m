function [T35,T05,T06]=humeralmockup(th1,th2,th3,th4,th5,normal_hum,thn,Z03hat,X35hat,gam,d3)

%% color spec, design
lighting gouraud

%% grey
jointfc2=[0.8,0.8,0.8];
   str = '#F5F5F5';
%  set(gcf,'color','w')
% set(gcf, 'color', 'white','units','pixels','position',[0 0 1920 1080]);
%  set(gcf,'color', 'white','unit','norm','position',[0 0 1 1])
 set(gcf,'color', 'white','unit','norm','position',[0 0 0.8 0.8])
color = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;
% facecolor='none';
facecolor=color;
edgecolor='none';
%     edgecolor='k';
%   jointfc='none';
   jointfc=color;
   jointfc1=[0.8,0.8,0.8];
   jointec='k';
linkfc='r';
 linkfc2='r';
linkec='k';
colorp=[1,0,0]; 
jointfcp=colorp;
facecolorp=colorp;
% hold off

joints=[th1;th2;th3;th4;th5];
% Denavit-Hartenberg parameters 7 DoF
%DH: [a , alpha,    d , theta] 
dh= [ 0    pi     0    0;
      0    pi/2   0    0;
      0    -pi/2  d3   0;
      0    pi/2   0    0;
      0    -pi/2  0    0;]; %last row is the endeffector transformation

%Assign joint values to the theta column of the DH parameters
dh(:,4) = dh(:,4)+joints;
% l_SE=-312e-3 
T01=TDH(dh(1,:));
T12=TDH(dh(2,:));
T23=TDH(dh(3,:));
T34=TDH(dh(4,:));
T45=TDH(dh(5,:));
% T56=TDH([ 0    0  l_SE   0]);
T56=trvec2tform([312e-3  0 0])*TDH([ 0    0  0   0]);

T02=T01*T12;
T03=T01*T12*T23;
T04=T01*T12*T23*T34;

T35=T34*T45;
T05=T03*T35;

T06=T03*T35*T56; % base to ee
% plot the joints and links
% axis determination
k_1ik=T01(1:3,3);
k_2ik=T02(1:3,3);
k_3ik=T03(1:3,3);
k_4ik=T04(1:3,3);
k_5ik=T05(1:3,3);
i_5ik=T05(1:3,1);

% plot the joints
rev_joint_axis(0.3,k_1ik',20,0.009,eye(3),T01(1:3,4)',jointfc,jointec,facecolor);
hold on
rev_joint_axis(0.3,k_2ik',20,0.009,eye(3),T02(1:3,4)',jointfc,jointec,facecolor);
hold on
rev_joint_axis(0.3,k_3ik',20,0.009,eye(3),T03(1:3,4)',jointfc,jointec,facecolor);
hold on
rev_joint_axis(0.3,k_4ik',20,0.009,eye(3),T04(1:3,4)',jointfc,jointec,facecolor);
hold on
rev_joint_axis(0.3,k_5ik',20,0.009,eye(3),T05(1:3,4)',jointfc,jointec,facecolor);
hold on

%plot the links
P00=T02(1:3,4)'
P0up=T03(1:3,4)'
P01=T05(1:3,4)'
% P02=T05(1:3,4)'+0.055*i_5ik'
% P02=T05(1:3,4)'+0.055*k_3ik'
% P02=T05(1:3,4)'+0.055*k_3ik' % this will change if k_3ik will change based on transf matrix, so put a fixed distance based on ee transf.
P02= T06(1:3,4)'

plotboxbetweenpoints(0.001,[P00(1) P00(2) P00(3)],[P0up(1) P0up(2) P0up(3)],[0.8,0.8,0.8],linkec,jointfc2)
hold on
plotboxbetweenpoints(0.001,[P01(1) P01(2) P01(3)],[P02(1) P02(2) P02(3)],[0.8,0.8,0.8],linkec,jointfc2)
hold on

%% plot frames 
%plot z-axes

 P00up=T01(1:3,4)'+0.0346*k_1ik';
 P001=T01(1:3,4)';
 P0a1=T02(1:3,4)';
 P0a1up=T02(1:3,4)'+0.0346*k_2ik';
 P0b1=T03(1:3,4)';
 P0b1up=T03(1:3,4)'+0.0346*k_3ik';
 P0c1=T04(1:3,4)';
 P0c1up=T04(1:3,4)'+0.0346*k_4ik';
 P0d1=T05(1:3,4)';
 P0d1up=T05(1:3,4)'+0.0346*k_5ik';

 h1 = mArrow3([P001(1) P001(2) P001(3)],[P00up(1) P00up(2) P00up(3)],'color','blue','stemWidth',0.001,'tipWidth',0.002,'facealpha',1);
 h2 = mArrow3([P0a1(1) P0a1(2) P0a1(3)],[P0a1up(1) P0a1up(2) P0a1up(3)],'color','blue','stemWidth',0.001,'tipWidth',0.002,'facealpha',1);
 h3 = mArrow3([P0b1(1) P0b1(2) P0b1(3)],[P0b1up(1) P0b1up(2) P0b1up(3)],'color','blue','stemWidth',0.001,'tipWidth',0.002,'facealpha',1);
 h4 = mArrow3([P0c1(1) P0c1(2) P0c1(3)],[P0c1up(1) P0c1up(2) P0c1up(3)],'color','blue','stemWidth',0.001,'tipWidth',0.002,'facealpha',1);
 h5 = mArrow3([P0d1(1) P0d1(2) P0d1(3)],[P0d1up(1) P0d1up(2) P0d1up(3)],'color','blue','stemWidth',0.001,'tipWidth',0.002,'facealpha',1);


lighting gouraud     
grid off
% view(-17.805881531238548,40.523863504193613) % Abduction adduction in XZ plane
view(-0.737646237120895,90) % Intra extra rotation in XY plane
ylim([-0.07 0.15])
xlabel('$X$','interpreter','latex','FontSize',14)
ylabel('$Y$','interpreter','latex','FontSize',14)
zlabel('$Z$','interpreter','latex','FontSize',14)
end