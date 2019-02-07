%% Trajectory generation using quintic polynomials

%% One-dimensional case
% Defining the start and ending times
ts = 0; 
tf = 10;

S = @(ts, tf) [ts^5    ts^4    ts^3   ts^2 ts 1;
     tf^5    tf^4    tf^3   tf^2 tf 1;
     5*ts^4  4*ts^3  3*ts^2 2*ts 1  0;
     5*tf^4  4*tf^3  3*tf^2 2*tf 1  0;
     20*ts^3 12*ts^2 6*ts   2    0  0;
     20*tf^3 12*tf^2 6*tf   2    0  0];

pos = zeros(3, 3, 200);
% Desired boundary conditions for position, velocity and acceleration
ss = 0;
sf = 1;
sdots = 0;
sdotf = 0;
sddots = 0;
sddotf = 0;

bcond = [ss sf sdots sdotf sddots sddotf]';
A = S(ts,tf);
scoef = A\bcond;
sdotcoef = scoef(1:5)'.*[5 4 3 2 1];
sddotcoef = scoef(1:4)'.*[20 12 6 2];
t = linspace(ts, tf, 100);
s = polyval(scoef, t);
sdot = polyval(sdotcoef, t);
sddot = polyval(sddotcoef, t);

% Generating a multi-point trajectory
theta = 2*pi/4;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
P = [1; 0];
p0 = P;
p1 = [0; -1];
p2 = [-1; 0];
p3 = [0; 1];

% p0 to p1
% First, x dimension
ss = p0(1);
sf = p1(1);
sdots = 0;
sdotf = 0;
sddots = 0;
sddotf = 0;
bcond = [ss sf sdots sdotf sddots sddotf]';
A = S(ts,tf);
scoef = A\bcond;
sdotcoef = scoef(1:5)'.*[5 4 3 2 1];
sddotcoef = scoef(1:4)'.*[20 12 6 2];
t = linspace(ts, tf, 100);
tg = t;
x = polyval(scoef, t);
xdot = polyval(sdotcoef, t);
xddot = polyval(sddotcoef, t);
% Now for y
ss = p0(2);
sf = p1(2);
sdots = 0;
sdotf = 0;
sddots = 0;
sddotf = 0;
bcond = [ss sf sdots sdotf sddots sddotf]';
A = S(ts,tf);
scoef = A\bcond;
sdotcoef = scoef(1:5)'.*[5 4 3 2 1];
sddotcoef = scoef(1:4)'.*[20 12 6 2];
y = polyval(scoef, t);
ydot = polyval(sdotcoef, t);
yddot = polyval(sddotcoef, t);
% p1 to p2
% X
ss = p1(1);
sf = p2(1);
sdots = 0;
sdotf = 0;
sddots = 0;
sddotf = 0;
bcond = [ss sf sdots sdotf sddots sddotf]';
A = S(tf,2*tf);
scoef = A\bcond;
sdotcoef = scoef(1:5)'.*[5 4 3 2 1];
sddotcoef = scoef(1:4)'.*[20 12 6 2];
t = linspace(tf, 2*tf, 100);
x = [x polyval(scoef, t)];
xdot = [xdot polyval(sdotcoef, t)];
xddot = [xddot polyval(sddotcoef, t)];
% Y
ss = p1(2);
sf = p2(2);
sdots = 0;
sdotf = 0;
sddots = 0;
sddotf = 0;
bcond = [ss sf sdots sdotf sddots sddotf]';
A = S(tf,2*tf);
scoef = A\bcond;
sdotcoef = scoef(1:5)'.*[5 4 3 2 1];
sddotcoef = scoef(1:4)'.*[20 12 6 2];
t = linspace(tf, 2*tf, 100);
y = [y polyval(scoef, t)];
ydot = [ydot polyval(sdotcoef, t)];
yddot = [yddot polyval(sddotcoef, t)];
tg = [tg t];

subplot(3,1,1)
plot(tg, x, tg, y)
subplot(3,1,2)
plot(tg, xdot, tg, ydot)
subplot(3,1,3)
plot(tg, xddot, tg, yddot)

for i=1:200
    pos(:,:,i) = SE2(x(i), y(i), 0);
end