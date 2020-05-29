%Small demonstration of the usensor function
phi = pi/10;
pts = rand(2,100);
[d,c]=usensor(0, 0, pi/4, phi, pts(1,:), pts(2,:));
figure;
hold;
plot(pts(1,:), pts(2,:), 'r.', pts(1,c), pts(2,c), 'bs');
plot([0 1], [0 1], 'b');
plot([0 sqrt(2)*cos(pi/4+phi)], [0 sqrt(2)*sin(pi/4+phi)], 'b--');
plot([0 sqrt(2)*cos(pi/4-phi)], [0 sqrt(2)*sin(pi/4-phi)], 'b--');
axis equal;