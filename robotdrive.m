%
% use keyboard to drive a wheeled mobile robot
%
% to run: just type in
%
% >> robotdrive
%
% use the arrow key in the figure window to drive the robot
% (up=forward, down=backware, left=turn left, right=turn right)
% press q to quit
%
% this program calls
%
% roomspec: set up the room and all collision objects in colobj structure
% roomshow: show all the collision objects
% robotspec: set up the robot collision body
% robotshow: show robot in the same plot
% robotfcn: function that gets called when a key is pressed in the figure
% output: generates all the sensor reading based on the current robot pose
% pose_est: estimating the pose of the robot
% colcheck: collision check between robot and all objects in room

clear all;close all;

addpath(genpath(pwd))

% define room
roomspec; % want to modify
% show room

fignum=1;
h_fig=roomshow(colobj,fignum);
axis('square');

% define robot
rL=.4;rW=.2;rz=.1;
robot=robotspec([rL;rW;2*rz]);

% show robot
q0=[1;3;0]; %starting point
q(:,1)=q0;
robotshow(robot,q0);
ez=[0;0;1];

% # of scan lines in lidar
N_scan=3;

% **** range sensors ****
% UWB (local GPS)
zW=colobj.obj{1}.Z; %what is this? unused?
lW=colobj.obj{1}.X;
zW=0;
%sensor locations?
pL(:,1)=[0;0;zW];
pL(:,2)=[lW;0;zW];
pL(:,3)=[0;lW;zW];
pL(:,4)=[lW;lW;zW];

% **** bearing sensors ****
pB(:,1)=colobj.obj{14}.Pose(1:3,4);
pB(:,2)=colobj.obj{10}.Pose(1:3,4);

% **** number
% 4 range sensors, 2 bearing sensors for robot, 2 IMU sensors
% 5 independent LIDAR sensor for room mapping
N_range=size(pL,2);
N_bearing=size(pB,2);
N_odo=2;
ns=N_range+N_bearing+N_odo+N_scan; % total # of sensors

% noise covariance
wcov=[0.01;0.01]*10;
vcov=.01*ones(ns,1)*10; 

% steering command and sampling period
v=0.5; w=pi/6; ts=1;
% initial sensor reading
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial state estimate
qhat(:,1)=pose_est(y(:,1),pL,pB,N_scan,wcov,vcov);

k=1;
% set up key press detection in figure
set(h_fig,'KeyPressFcn',@robotfcn);
keyvalue='a';
keypressed=0;

offset = 0; %for accounting for theta discontinuities

while keyvalue~='q' % keypress of q will quit
    while keypressed==0;pause(.05);end
    u(:,k)=[0;0];
    keypressed=0;
    switch keyvalue
        case 'uparrow'
            u(1,k)=v;
        case 'downarrow'
            u(1,k)=-v;
        case 'leftarrow'
            u(2,k)=w;
        case 'rightarrow'
            u(2,k)=-w;
    end
    
    % ==================================================
    % propagate robot state
    [q(:,k+1), utrue(:,k)]=wmr(q(:,k),u(:,k),ts,wcov);
    % generate sensor output
    y(:,k+1)=output(q(:,k+1),pL,pB,N_scan,utrue(:,k),vcov,robot,colobj);
    % estimate robot state
    qhat(:,k+1)=pose_est(y(:,k+1),pL,pB,N_scan,wcov,vcov);
    % ==================================================

    % check collision
    [isInt,dist,wp]=colcheck(robot,q(:,k+1),colobj);
    if max(isnan(dist))>0
        q(:,k+1)=q(:,k);
        disp('collision!');
    end
    
    %correction of theta_hat discontinuities
    qhat(3,end) = qhat(3,end) + offset; %dynamic offset
    jump = qhat(3,k+1)-qhat(3,k);
    if abs(jump) > 3*(w+sqrt(wcov(1))) %dynamic threshold adjustment
        qhat(3,k+1) = qhat(3,k+1) - sign(jump)*pi; %static offset
        offset = offset - sign(jump)*pi; %adjusting dynamic offset
    end
       
    robotshow(robot,q(:,k+1));
    k=k+1;
end

% post-run analysis
t=[1:k];
figure(2);
plot(t,q,t,qhat, ':','linewidth',2);
grid on
legend('x','y','\theta','x_{est}','y_{est}','\theta_{est}');


