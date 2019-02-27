%=========================================================================%
% 'UpdateCG.m' updates the location of the center of gravity (CG)         %
% assuming a linear transition from the initial CG location to the final  % 
% CG location. Location is given in terms of {x, y, z}.                   %
%                                                                         %              
% INPUT:                                                                  %
% previous CG location (CG), incremental CG location (dCG), and a flag    %
% indicating if the ADCS was activated (fire)                             %
%                                                                         %
% OUTPUT:                                                                 %  
% updated CG location (CGnew)                                             %  
%=========================================================================%

function [CGnew] = UpdateCG(CG, dCG, fire)     
    % CG transition if a fire occured
    if any(fire) == true
        CGnew(1) = CG(1)+dCG(1);
        CGnew(2) = CG(2)+dCG(2);
        CGnew(3) = CG(3)+dCG(3); 
    else
        CGnew = CG;
    end 
end 