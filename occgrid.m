%% Occupancy grid construction using a moving ultrasonic sensor
% J.P. Ramirez-Paredes <jpi.ramirez@ugto.mx>

x = 5;
y = 5;
theta = pi/5;
dmax = 2; % Maximum distance for sensing
phi = pi/20;
S = SE2(x, y, theta);
figure(1);
hold;
trplot2(S);
Nobs = 50;
%obstacles = 10*rand(2, Nobs);
xobs = linspace(5, 7, Nobs);
obstacles = [xobs; 12-xobs];
v = 0.5;
omega = pi;
dt = 0.1;

% Matrix to hold the occupancy grid
ssize = 0.5;
M = 0.5*ones(ceil(10/ssize)); % 10 cm per cell side, covering 10x10 sq meters

w2g = @(x, y) [floor(y./ssize); floor(x./ssize)]+1;

for i=1:200
    % Displacing the sensor along a circular path
    x = x + dt*v*cos(theta);
    y = y + dt*v*sin(theta);
    theta = theta + dt*omega;
    angdiff(0, theta);
    S = SE2(x, y, theta);
    trplot2(S);
    d = usensormod(x, y, theta, phi, dmax, obstacles(1,:), obstacles(2,:))
    % From the sensor's perspective, all detected points lie in the
    % positive X axis at coordinate (d, 0).
    if d > 0
        pobs = [d; 0];
        robs = S.T*[pobs; 1];
        plot(robs(1,:), robs(2,:), 'bs');
        pt = w2g(robs(1,:), robs(2,:));
        p = bresenham(5/ssize, 5/ssize, pt(1), pt(2));
        for j=1:length(p)
            M(p(j,1), p(j,2)) = 0;
        end
        M(pt(1), pt(2)) = 1;
    else
        pnobs = [dmax; 0];
        rnobs = S.T*[pnobs; 1];
        plot(rnobs(1,:), rnobs(2,:), 'rs');
        pt = w2g(rnobs(1,:), rnobs(2,:));
        p = bresenham(5/ssize, 5/ssize, pt(1), pt(2));
        for j=1:length(p)
            M(p(j,1), p(j,2)) = 0;
        end
    end
end

plot(obstacles(1,:), obstacles(2,:), 'r*');

axis equal;

figure(2);
imagesc(M);
set(gca, 'YDir', 'normal');
axis equal;