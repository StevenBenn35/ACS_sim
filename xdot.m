%=========================================================================%
% 'xdot.m' returns an array of first-order differential equations of      % 
% motion to be integrated in 'RK4.m'.                                     %
%                                                                         %
% INPUT:                                                                  % 
% current state_vector (x_old), Moment vector (M), inertia tensor (I)     %
%                                                                         %
% OUTPUT:                                                                 %      
% state propagation results (x_dot)                                       %
%=========================================================================%

function [x_dot] = xdot(x_old,M,I)     
    Ixx = I(1,1);    Ixy = I(1,2);    Ixz = I(1,3);                        
    Iyy = I(1,4);    Iyz = I(1,5);    Izz = I(1,6);

    omega = x_old(1:3,1);
    phi   = x_old(4,1);
    theta = x_old(5,1);
             
    Sphi   = sin(phi);
    Cphi   = cos(phi);
    Ttheta = tan(theta);
    Ctheta = cos(theta);
   
    tensor = [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];
    
    transf = [1, Sphi*Ttheta, Cphi*Ttheta; ...
              0, Cphi, -Sphi;              ...
              0, Sphi/Ctheta, Cphi/Ctheta];
    
   	x_dot(1:3,1) = tensor\(M'-cross(omega,tensor*omega));
    x_dot(4:6,1) = transf*omega; 
end