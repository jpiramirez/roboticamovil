function xdot = ctrlcar(x, xd, yd, Kv, Kh, L)


v = Kv*sqrt((xd-x(1))^2 + (yd-x(2))^2);
thetad = atan2(yd-x(2), xd-x(1));
gamma = Kh*(thetad-x(3));

xdot = [v*cos(x(3)); 
        v*sin(x(3));
        (v/L)*tan(gamma)];