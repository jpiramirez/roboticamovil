% Unicycle model simulation
% (c) Universidad de Guanajuato 2019
% Author: Juan-Pablo Ramirez-Paredes
%         <jpi.ramirez@ugto.mx>
% Robotica Movil
function xdot = ucycle(x, v, omega)

xdot = [v*cos(x(3)); 
        v*sin(x(3));
        omega];