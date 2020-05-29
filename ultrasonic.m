%% Small demonstration of ultrasonic sensor model
% J.P. Ramirez-Paredes <jpi.ramirez@ugto.mx>

x = 5;
y = 5;
theta = pi/5;
phi = pi/40;
S = SE2(x, y, theta);
figure(1);
hold;
trplot2(S);
Nobs = 50;
obstacles = 10*rand(2, Nobs);
v = 0.5;
omega = pi;
dt = 0.05;

for i=1:20
    % Displacing the sensor along a circular path
    x = x + dt*v*cos(theta);
    y = y + dt*v*sin(theta);
    theta = theta + dt*omega;
    S = SE2(x, y, theta);
    trplot2(S);
    [d, c] = usensor(x, y, theta, phi, obstacles(1,:), obstacles(2,:));
    % From the sensor's perspective, all detected points lie in the
    % positive X axis at coordinate (d, 0).
    pobs = [d; zeros(1, Nobs)];
    % We have to select only those points that the sensor can actually
    % detect. This is encoded in the c array returned by usensor.
    robs = S.T*[pobs(:,c); ones(1, sum(c))];
    plot(robs(1,:), robs(2,:), 'bs');
end

plot(obstacles(1,:), obstacles(2,:), 'r*');

axis equal;
