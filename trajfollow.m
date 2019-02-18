% Trajectory following by Ackermann steering car
% (c) Universidad de Guanajuato 2019
% Author: Juan-Pablo Ramirez-Paredes
%         <jpi.ramirez@ugto.mx>
% Robotica Movil

% The trajectory must be stored in a file called 'traj.mat', containing 
% variables t for the time vector and y for the states (Nx3).

load traj.mat;
tt = t;
yy = y;

d = 0.001;
xd = @(t) interp1(tt, yy(:,1), t);
yd = @(t) interp1(tt, yy(:,2), t);
Kv = 5.5;
Kh = 1.5;
Ki = 0.5;
L = 0.5;

err = @(t, x) sqrt( (xd(t)-x(1))^2 + (yd(t)-x(2))^2) - d;
angerr = @(t, x) atan2(yd(t)-x(2), xd(t)-x(1));
ccar = @(t, x) [(Kv*err(t,x)+Ki*x(4))*cos(x(3));
               (Kv*err(t,x)+Ki*x(4))*sin(x(3));
               ((Kv*err(t,x)+Ki*x(4))/L)*tan(Kh*(angerr(t,x)-x(3)));
               err(t,x)];

[t, y] = ode45(ccar, [tt(1) tt(end)], [yy(1,:)'; 0]);
           
np = size(t, 1);
T = zeros(3, 3, np);
for i=1:np
    T(:,:,i) = SE2(y(i,1), y(i,2), y(i,3));
end

figure(1)
hold
plot(yy(:,1), yy(:,2), 'g');
axis equal;
tranimate2(T)
figure(2)
trplot2(T(:,:,1), 'frame', 'inicio')
hold
trplot2(T(:,:,end), 'color', 'r', 'frame', 'final')
plot(y(:,1), y(:,2))
axis([-13 13 -13 13])

figure(3)
plot(diff(y(:,4))/mean(diff(t)))