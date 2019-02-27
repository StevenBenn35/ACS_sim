%=========================================================================%
% 'RK4.m' integrates a specified set of differential equations using the  %
% Runga-Kutta 4th-order integration scheme. The state variables are       % 
% inputted and 'SpinEqns.m' is called which contains the differential     %
% equations of motion.                                                    % 
%                                                                         %
% INPUTS:                                                                 %
% current state vector (x_old), moment vector (M), inertia tensor (I),    %   
% and sample rate (dt)                                                    % 
%                                                                         %
% OUTPUTS:                                                                %
% updated state vector (x_new)                                            %
%=========================================================================%

function [x_new] = RK4(x_old, M, I, dt) 
    dt2 = dt/2;
    dt6 = dt/6;

    k1 = xdot(x_old, M, I);
    xp = x_old+dt2*k1;
    
    k2 = xdot(xp, M, I);
    xp = x_old+dt2*k2;
    
    k3 = xdot(xp, M, I);
    xp = x_old+dt*k3;
    
    k4 = xdot(xp, M, I);
    x_new = x_old+dt6.*(k1+2.*k2+2.*k3+k4);
    
    % maintain roll angle within 0-360 deg (0-2*pi rad)
    if x_new(4,1) >= 2*pi
        x_new(4,1)= x_new(4,1)-2*pi;
    end    
end