s = lspb(0, pi/2, 100);
p = lspb(0, pi/2, 100);
x = lspb(0, 1, 100);
y = x;
z = y;
n = length(s);
T = zeros(4, 4, n);
for i=1:n
    T(:,:,i) = [rotx(s(i)*180/pi)*roty(p(i)*180/pi)*rotz(s(i)*180/pi) [x(i) y(i) z(i)]'; 0 0 0 1];
end