% Simulating a camera

% 3D coordinates of a cube
cube = [0 1 0 1 0 1 0 1; 1 1 0 0 1 1 0 0; 1 1 1 1 0 0 0 0];
%figure;
%plot3(cube(1,:), cube(2,:), cube(3,:), '*');

R = rotx(10);
t = [0 0 -5]';
K = eye(3);

x = K*[R t]*[cube; ones(1,8)];
p = x./repmat(x(3,:), 3, 1);
cam = SE3(R, t);

figure;
plot(p(1,:), p(2,:), '*');
axis([-1 1 -1 1]);