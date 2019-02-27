%=========================================================================%
% 'UpdateInertias.m' updates the instantaneous inertia tensor by          %
% estimating the new inertia tensor and mapping the new tensor about the  %
% updated center of gravity. The new inertia tensor is approximated       %
% by "scaling" the tensor by the ratio of the current mass to the initial %
% mass.                                                                   %
%                                                                         %
% INPUT:                                                                  %
% previous inertia tensor (I), initial inertia tensor (I0), current mass  %
% (mass), initial stage mass (mass0), incremental CG location (dCG), and  %
% flag indicating if a burn occurred during this timestep (fire)          %
%                                                                         %
% OUTPUT:                                                                 %
% updated intertia tensor (I_new)                                         %
%=========================================================================%

function Inew = UpdateInertias(I, I0, mass, mass0, dCG, fire)  
    if any(fire) == true
        Ixx = I0(1,1)*(mass/mass0);
        Ixy = I0(1,2)*(mass/mass0);
        Ixz = I0(1,3)*(mass/mass0);
        Iyy = I0(1,4)*(mass/mass0);
        Iyz = I0(1,5)*(mass/mass0);
        Izz = I0(1,6)*(mass/mass0);
    
        x = dCG(1); 
        y = dCG(2); 
        z = dCG(3);

        Ixx = Ixx+mass*(y^2+z^2);                                 
        Ixy = Ixy+mass*(x*y); 
        Ixz = Ixz+mass*(x*z); 
        Iyy = Iyy+mass*(x^2+z^2); 
        Iyz = Iyz+mass*(y*z); 
        Izz = Izz+mass*(x^2+y^2); 

        Inew = [Ixx,Ixy,Ixz,Iyy,Iyz,Izz];
    else
        Inew = I;
    end
 
end 

