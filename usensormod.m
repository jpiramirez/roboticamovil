% Brief ultrasonic sensor simulation
% J.P. Ramirez-Paredes <jpi.ramirez@ugto.mx>

function d = usensormod(x, y, theta, phi, dmax, xp, yp)

phii = angdiff(theta*ones(1, length(xp)), atan2(yp-y, xp-x)); %determine the angle between the sensing direction and the angle to every point
vp = abs(phii) < phi; %choose only the points that can actually be covered by the sensor
if sum(vp) == 0 % if no points can be covered by the sensor, just return a -1 (no detection)
    d = -1;
    return
end

d = min(sqrt((xp(vp)-x).^2+(yp(vp)-y).^2)); % from the covered points, select the one closest to the sensor

if d > dmax % even if the point is on the FOV of the sensor, it can be too far to be detected
    d = -1;
end