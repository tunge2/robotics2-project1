

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
rL=.5;rW=.2;rz=.1;
robot=robotspec([rL;rW;2*rz]);

% show robot
q0=[0.5;0.5+rand()*3;0];
q(:,1)=q0;
robotshow(robot,q0);
ez=[0;0;1];

% # of scan lines in lidar
N_scan=5;

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
% wcov=[0.05;0.05];
% vcov=.1*ones(ns,1);
wcov=[0.1;0.1];
vcov=.1*ones(ns,1);

% steering command and sampling period
%v=.1;w=.1;ts=1;
v=0.3; w=1; ts=1;
% w is angle

% initial sensor reading
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial state estimate
qhat(:,1)=pose_est(y(:,1),pL,pB,N_scan,wcov,vcov);


N_step=100;
offset = 0;
collision = false;
for k=1:N_step
    try
        if collision == true
            decide_turn = round(rand());
            if decide_turn == 0
                u(2,k) = w;
                move_word = 'Turn L';
            else
                u(2,k) = -w;
                move_word = 'Turn R';
            end
        else
            u(:,k)=[0;0];
            u(1,k) = v;
            move_word = 'Forward';
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
        
        if max(isnan(dist))>0 % if next move will collide
            q(:,k+1)=q(:,k); % no move at all
            collision = true; % change collision state to 
            % actuate a turn on the next move
        else
            collision = false; % keep going
        end
        
        %correction of theta_hat discontinuities
        qhat(3,end) = qhat(3,end) + offset;
        jump = qhat(3,k+1)-qhat(3,k);
        if abs(jump) > 3*(w+sqrt(wcov(1))) %dynamic threshold adjustment
            qhat(3,k+1) = qhat(3,k+1) - sign(jump)*pi; %static offset
            offset = offset - sign(jump)*pi; %adjusting dynamic offset
        end
        
        
        robotshow(robot,q(:,k+1));
        disp(move_word);
        k=k+1;
        pause(0.5)
        
    catch
        
        %correction of theta_hat discontinuities
        qhat(3,end) = qhat(3,end) + offset; %dynamic offset
        jump = qhat(3,k+1)-qhat(3,k);
        if abs(jump) > 3*(w+sqrt(wcov(1))) %dynamic threshold adjustment
            qhat(3,k+1) = qhat(3,k+1) - sign(jump)*pi; %static offset
            offset = offset - sign(jump)*pi; %adjusting dynamic offset
        end
        
        robotshow(robot,q(:,k+1));
        disp(move_word);
        k=k+1;
        pause(0.01)
    end
end

%t=[1:k];
figure(2);
plot(1:size(q,2),q,1:size(qhat,2),qhat, ':','linewidth',2);
grid on
legend('x','y','\theta','x_{est}','y_{est}','\theta_{est}');
