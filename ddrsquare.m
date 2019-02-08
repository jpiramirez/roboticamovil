% DDR simulation following a square pattern
% (c) Universidad de Guanajuato 2019
% Author: Juan-Pablo Ramirez-Paredes
%         <jpi.ramirez@ugto.mx>
% Robotica Movil
ts = 0;
tf = 5;
tstep = 5;

t = [];
y = [];
yy = [0 0 0];

for k=1:4
    v = 10/5;
    [tt, yy] = ode45(@(t, x) ucycle(x, v, 0), [ts tf], yy(end,:)');
    t = [t; tt];
    y = [y; yy];
    ts = tf;
    tf = tf + tstep;
    omega = (pi/2)/5;
    [tt, yy] = ode45(@(t, x) ucycle(x, 0, omega), [ts tf], yy(end,:)');
    t = [t; tt];
    y = [y; yy];
    ts = tf;
    tf = tf + tstep;
end


np = size(t, 1);
T = zeros(3, 3, np);
for i=1:np
    T(:,:,i) = SE2(y(i,1), y(i,2), y(i,3));
end

tranimate2(T)
figure(1)
trplot2(T(:,:,1), 'frame', 'inicio')
hold
trplot2(T(:,:,end), 'color', 'r', 'frame', 'final')
plot(y(:,1), y(:,2))
axis([-3 13 -3 13])

figure(2)
plot(t, y(:,1), t, y(:,2), t, y(:,3))
legend('X', 'Y', '\theta')