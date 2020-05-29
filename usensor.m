% Brief ultrasonic sensor simulation
% J.P. Ramirez-Paredes <jpi.ramirez@ugto.mx>

function [d, c] = usensor(x, y, theta, phi, xp, yp)

phii = atan2(yp-x, xp-x)-theta;

d = sqrt((xp-x).^2+(yp-y).^2);
c = abs(phii) < phi;
%d = phii;
d(~c) = -1;