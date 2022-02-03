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
%

clear all;
close all;

addpath(genpath(pwd))

% define room
roomspec;
% show room

fignum=1;
h_fig=roomshow(colobj,fignum);
axis('square');

% define robot
rL=.4;rW=.2;rz=.1;
robot=robotspec([rL;rW;2*rz]);

% show robot
q0=[0.5;0.5+rand()*3;0];
q(:,1)=q0;
robotshow(robot,q0);
ez=[0;0;1];

% # of scan lines in lidar
N_scan=3;

% **** range sensors ****
% UWB (local GPS)
%zW=colobj.obj{1}.Z; is this used?
lW=colobj.obj{1}.X;
zW=0;
pL(:,1)=[0;0;zW];
pL(:,2)=[lW;0;zW];
pL(:,3)=[0;lW;zW];
pL(:,4)=[lW;lW;zW];
% **** bearing sensors ****
pB(:,1)=colobj.obj{14}.Pose(1:3,4);
pB(:,2)=colobj.obj{10}.Pose(1:3,4);
%
N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;
%
ns=N_range+N_bearing+N_odo+N_scan; % total # of sensors
wcov=[0.1;0.1];
vcov=.1*ones(ns,1); % noise covariance
% steering command and sampling period
%v=.1;w=.1;ts=1;
v=0.3; w=1; ts=1;

% initial sensor reading
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial state estimate
qhat(:,1)=pose_est(y(:,1),pL,pB,N_scan,wcov,vcov);

N_step=100;
offset = 0;
for k=1:N_step
    collision = true;
    disp(['Step ' num2str(k)])
    while collision == true
        a1 = 50;
        a2 = 5;
        a3 = 10;
        a4 = 10;
        move_num = rand()*(a1+a2+a3+a4);
        
        u(:,k)=[0;0];
        if move_num < a1
            u(1,k) = v;
            move_word = 'Forward';
        elseif (move_num >= a1)&(move_num < a1+a2)
            u(1,k) = -v;
            move_word = 'Backward';
        elseif (move_num >= a1+a2)&(move_num < a1+a2+a3)
            u(2,k) = w;
            move_word = 'Turn L';
        else
            u(2,k) = -w;
            move_word = 'Turn R';
        end
        
        % propagate robot state
        [q(:,k+1),utrue(:,k)]=wmr(q(:,k),u(:,k),ts,zeros(size(wcov)));
        % generate sensor output
        y(:,k+1)=output(q(:,k+1),pL,pB,N_scan,utrue(:,k),vcov,robot,colobj);
        % estimate robot state
        qhat(:,k+1)=pose_est(y(:,k+1),pL,pB,N_scan,wcov,vcov);
        
        % what is this?
        %[qhat_iter,delta_y]=pose_est_NLS(qhat(:,k+1),y(:,k+1),pL,pB);
        %dy{k}=delta_y;
        
        %qhat1(:,k+1)=qhat_iter(:,size(qhat_iter,2));
        qhat(:,k+1)=pose_est(y(:,k+1),pL,pB,N_scan,wcov,vcov);
        
        % check collision
        [isInt,dist,wp]=colcheck(robot,q(:,k+1),colobj);
        if max(isnan(dist))>0
            %q(:,k+1)=q(:,k);
            collision = true;
            disp('collision!');
        else
            disp(move_word)
            collision = false;
        end
    end
    
    %correction of theta_hat discontinuities
    qhat(3,end) = qhat(3,end) + offset;
    jump = qhat(3,k+1)-qhat(3,k);
    if abs(jump) > 3*(w+sqrt(wcov(1))) %dynamic threshold adjustment
        qhat(3,k+1) = qhat(3,k+1) - sign(jump)*pi; %static offset
        offset = offset - sign(jump)*pi; %adjusting dynamic offset
    end
    robotshow(robot,q(:,k+1));
    k=k+1;
    
    pause(0.05)
end

% post-run analysis

% t=[0:N_step];
% figure(2);plot(t,q(:,1:N_step+1),t,qhat(:,1:N_step+1),'.',...
%     t,qhat1(:,1:N_step+1),':','linewidth',2);
% legend('x','y','\theta','x_{est}','y_{est}','\theta_{est}','x1','y1','\theta_1');

t=[1:N_step+1];
figure(2);
plot(t,q,t,qhat, ':','linewidth',2);
grid on
legend('x','y','\theta','x_{est}','y_{est}','\theta_{est}');
