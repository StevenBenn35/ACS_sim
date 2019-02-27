%=========================================================================% 
% slosh_model.m is a function that contains the method for implementing   %
% the equivalent mechanical slosh model. The actual logic is handled      %
% inside Slosh_Dynamics.m, which is called within this function. A        %
% variable G-level that is based on main motor activation is calculated   %
% and input into the slosh model.                                         %
%=========================================================================%

function [P_Slosh, Y_Slosh, Msl, Moff, Fsl] = slosh_model(P_Slosh,  ...
          Y_Slosh, state, mass, g, g0, dt, CGs, mm_thrust, i)
    
    % masses
    total_mass = mass(1);
    tank_mass = mass(2);
    
    % organize angle array for slosh model
    if i == 1
        angles = state(7:9,i);
    elseif i==2
        angles = state(7:9,i-1:i);
    else
        angles = state(7:9,i-2:i);
    end
    
    % variable G-level based on main motor activation
    if mm_thrust.fired(i,1) == true
        G = norm(mm_thrust.F)/total_mass; 
        g = g0*G;
    end
    
    % slosh model about pitch axis
    [P_Slosh,M_slP,M_offP,F_slP] = P_Slosh.Slosh_MAIN(angles,state(1:3,i), ...
        tank_mass,g,dt,CGs,i);
    
    % slosh model about yaw axis
    [Y_Slosh,M_slY,M_offY,F_slY] = Y_Slosh.Slosh_MAIN(angles,state(1:3,i), ...
        tank_mass,g,dt,CGs,i);
    
    % sum slosh forces and moments 
    Msl  = M_slP+M_slY;
    Moff = M_offP+M_offY;
    Fsl  = F_slP+F_slY;
end