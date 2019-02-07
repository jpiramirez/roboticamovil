function xdot = car(x, r, L, ur, ul)

xdot = [0.5*r*(ul+ur)*cos(x(3)); 
        0.5*r*(ul+ur)*sin(x(3));
        (r/L)*(ur-ul)];