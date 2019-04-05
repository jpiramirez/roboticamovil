% Differential steering car simulation
% (c) Universidad de Guanajuato 2019
% Author: Juan-Pablo Ramirez-Paredes
%         <jpi.ramirez@ugto.mx>
% Robotica Movil

function xdot = car(x, r, L, ur, ul)

xdot = [0.5*r*(ul+ur)*cos(x(3)); 
        0.5*r*(ul+ur)*sin(x(3));
        (r/L)*(ur-ul)];