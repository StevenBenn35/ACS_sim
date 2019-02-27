%=========================================================================%
% 'UpdateMass.m' updates the instantaneous mass of the stage by           % 
% subtracting the instantaneous total propellant mass burned by the       %
% thrusters from the initial stage mass. The oxidizer mass and fuel mass  %
% are also calculated based on the Oxidizer-to-fuel-ratio.                %
%                                                                         % 
% INPUT:                                                                  %
% instantaneous stage mass (mass), mass attenuation of current timestep   %
% (attenuation), oxidizer-to-fuel ratio (OF)                              %
%                                                                         % 
% OUTPUT:                                                                 %
% new total mass, oxidizer mass, and fuel mass (new_mass)                 %
%=========================================================================%
function [new_mass] = UpdateMass(mass, attenuation, OF)  
    
    new_mass_ox = mass(2)-((OF-1)/OF)*attenuation;
    new_mass_fuel  = mass(3)-(1/OF)*attenuation;
    new_mass_total  = mass(1)-attenuation;
    
    new_mass = [new_mass_total, new_mass_ox, new_mass_fuel];        
end 