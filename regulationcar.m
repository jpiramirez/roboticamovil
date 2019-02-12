Kh = 0.5;
Kv = 0.5;
L = 0.5;

[t, y] = ode45(@(t, x) ctrlcar(x, 0, 0, Kv, Kh, L), [0 10], [10 10 pi/2]');

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
axis([-5 15 -5 15])

figure(2)
plot(t, y(:,1), t, y(:,2), t, y(:,3))
legend('X', 'Y', '\theta')