% DDR simulation following a figure 8 pattern
% (c) Universidad de Guanajuato 2019
% Author: Juan-Pablo Ramirez-Paredes
%         <jpi.ramirez@ugto.mx>
% Robotica Movil
ts = 0;
tf = 5;
tstep = 5;
animate = 1;

t = [];
y = [];
yy = [0 0 0];


v = 10;
omega = pi/2;
tf = (2*pi)/omega;
[tt, yy] = ode45(@(t, x) ucycle(x, v, omega), [ts tf], yy(end,:)');
t = [t; tt];
y = [y; yy];
ts = tf + (mean(diff(t)));
tf = tf + (2*pi)/omega;
omega = -omega;
[tt, yy] = ode45(@(t, x) ucycle(x, v, omega), [ts tf], yy(end,:)');
t = [t; tt];
y = [y; yy];
ts = tf;
tf = tf + tstep;



np = size(t, 1);
T = zeros(3, 3, np);
for i=1:np
    T(:,:,i) = SE2(y(i,1), y(i,2), y(i,3));
end

if animate == 1
    tranimate2(T)
end
figure(1)
trplot2(T(:,:,1), 'frame', 'inicio')
hold
trplot2(T(:,:,end), 'color', 'r', 'frame', 'final')
plot(y(:,1), y(:,2))
axis([-13 13 -13 13])
figure(2)
plot(t, y(:,1), t, y(:,2), t, y(:,3))
legend('X', 'Y', '\theta')

save('traj.mat', 't', 'y');