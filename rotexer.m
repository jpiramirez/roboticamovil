N = 10;
T = zeros(4,4,N);
% Null initial rotation
T(:,:,1) = eye(4);
T(4,4,:) = 1;
dt = 0.1; % Sampling time
w = [0.1 0.3 0.2]; % Rotational velocity
for i=2:10
    T(1:3,1:3,i) = (dt*skew(w)+eye(3))*T(1:3,1:3,i-1);
    det(T(1:3,1:3,i))
end

cmap = colormap(jet(N));
figure(1);
ax = gca;
colorbar('TickLabels', dt*(0:10));
hold;
for i=1:10
    trplot(T(:,:,i), 'color', cmap(i,:));
end

