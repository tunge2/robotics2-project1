
% choose a robot configuration
close all;

tmax=size(q,2);
addpath(genpath(pwd))
roomspec; % want to modify

%cloud_x = zeros(tmax*N_scan,1);
%cloud_y = cloud_x;

fignum=1;

hfig = roomshow(colobj,fignum);
axis('square');

for i=1:tmax
    qq=q(:,i);
    % show robot
    robotshow(robot,qq);
    % show the scan lines
    [x, y, l] = scanpattern(qq,robot,colobj,N_scan);
    
    cloud_x(N_scan*i:N_scan*i+N_scan-1) = x;
    cloud_y(N_scan*i:N_scan*i+N_scan-1) = y;
    disp(['Step ' num2str(i)])
    pause(0.01)
end
%end

figure(2)
hold on
plot(cloud_x,cloud_y, 'b.', 'linewidth',4)
grid on
axis square
plot(q(1,:), q(2,:), 'r-','linewidth',3)
legend('Room Map','Path Traveled','location','north')
view(-90,90)

