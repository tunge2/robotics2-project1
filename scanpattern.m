function [cloud_x, cloud_y, l]=scanpattern(q,robot,colobj,M,nofig)
phi=2*pi*[0:M-1]/M; r=.01; rz=robot.Z;
for i=1:M
    l(i)=rayscan(robot,q,colobj,phi(i));
    ray=defineray(l(i),r,q,rz,phi(i));
    if ~exist('nofig')
        show(ray);
    end
    cloud_x(i) = (l(i)/2)*cos(q(3)+phi(i))+ray.Pose(1,4);
    cloud_y(i) = (l(i)/2)*sin(q(3)+phi(i))+ray.Pose(2,4);
end
end


