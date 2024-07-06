
clc
%%
h= 17/100; %17cm average size of human shoulder 
d3=(4/5)*h; %the lateral size of shoulder girdle is represented by d3, distance between inner and outer shoulder centers
l_SE=-312e-3; % length of shoulder elbow, forearm
r=2;  % ratio for scapulo-humeral rhythym 
X00hat=[1;0;0]; % basis of the base frame at origin



%% test trajectory
% % Abductionin XZ plane

% waypoints=[-(d3+abs(l_SE))*sind(45),0,(d3+abs(l_SE))*cosd(45);
%     -(d3+abs(l_SE))*sind(50),0,(d3+abs(l_SE))*cosd(50);
%     -(d3+abs(l_SE))*sind(60),0,(d3+abs(l_SE))*cosd(60);
%     -(d3+abs(l_SE))*sind(70),0,(d3+abs(l_SE))*cosd(70);
%     -(d3+abs(l_SE))*sind(80),0,(d3+abs(l_SE))*cosd(80);
%     -(d3+abs(l_SE))*sind(90),0,(d3+abs(l_SE))*cosd(90);
%     -(d3+abs(l_SE))*sind(100),0,(d3+abs(l_SE))*cosd(100);
%     -(d3+abs(l_SE))*sind(110),0,(d3+abs(l_SE))*cosd(110);]'

waypoints=[-(d3+abs(l_SE))*sind(45),(d3+abs(l_SE))*cosd(45),0;
    -(d3+abs(l_SE))*sind(50),(d3+abs(l_SE))*cosd(50),0;
    -(d3+abs(l_SE))*sind(60),(d3+abs(l_SE))*cosd(60),0;
    -(d3+abs(l_SE))*sind(70),(d3+abs(l_SE))*cosd(70),0;
    -(d3+abs(l_SE))*sind(80),(d3+abs(l_SE))*cosd(80),0;
    -(d3+abs(l_SE))*sind(90),(d3+abs(l_SE))*cosd(90),0;
    -(d3+abs(l_SE))*sind(100),(d3+abs(l_SE))*cosd(100),0;
    -(d3+abs(l_SE))*sind(110),(d3+abs(l_SE))*cosd(110),0;]'

% waypoints =   [-(d3+abs(l_SE))*sind(45),0,(d3+abs(l_SE))*cosd(45);-(d3+abs(l_SE))*sind(55),0,(d3+abs(l_SE))*cosd(55); -(d3+abs(l_SE))*sind(65),0.0,(d3+abs(l_SE))*cosd(65); -(d3+abs(l_SE))*sind(95),0,(d3+abs(l_SE))*cosd(95);-(d3+abs(l_SE))*sind(105),0,(d3+abs(l_SE))*cosd(105)]';
numWaypoints = size(waypoints,2);
orientations=zeros(numWaypoints,3)';

waypointTimes = 0:4:28;

%% Trajectory sample time
ts = 0.2;
trajTimes = 0:ts:waypointTimes(end);

%% Boundary conditions (for polynomial trajectories)
waypointVels = zeros(numWaypoints,3)';

%% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

%% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;

%% Set up plot
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
hold on
if plotMode == 1
%     hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
% plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);
[q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccels);

%% plotting trajectory and extracting video
%  set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));
% plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)
% plotTrajectory(trajTimes,q)
% set(gcf, 'color', 'white','units','pixels','position',[0 0 1920 1080]);
% vidObj=VideoWriter('trajhumeralikthreexy.mp4');
% vidObj.FrameRate = 24;
% open(vidObj);

%% Intra/extra rotation in XY plane
% Trajectory following loop

th1=0;th2=0;th3=0;th4=0;th5=0;
[T03,T35,T05,T06]=humeralpointingfk(th1,th2,th3,th4,th5,d3)
P_GH=T06(1:3,4);
% axis manual

 for idx = 1:numel(trajTimes) 
   
    % Solve IK
    tgtPose = trvec2tform(q(:,idx)');
    X05hat=(tgtPose(1:3,4)-P_GH/norm(tgtPose(1:3,4)-P_GH))  %this is very important, pointing direction not pose, so check to give correct input
   [th1,th2,th3,th4,th5,normal_hum,thn,Z03hat,X35hat,gam]= humeralik(X05hat,X00hat,r,d3);
   [T35,T05,T06]=humeralmockup(th1,th2,th3,th4,th5,normal_hum,thn,Z03hat,X35hat,gam,d3);

   title(['Trajectory at t = ' num2str(trajTimes(idx))])
   drawnow
   %% for extracting video frames
%    cla
%    currFrame= getframe(gcf);
%    writeVideo(vidObj, currFrame);
   hold off
 end
%  close(gcf)
%  close(vidObj); % close video object