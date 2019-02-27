%=========================================================================%
% 'UpdatePosition.m' updates the position vectors of each thruster with   %
% respect to the stage's center of gravity.                               %
%                                                                         %                  
% INPUT:                                                                  %     
% thruster class (thruster), incremental CG location (dCG), flag          %
% indicating if thruster activation occurred this timestep (fire), roll   %
% controller class (CtrlRoll), ADCS controller classes (PitchCtrl,        %
% YawCtrl), main motor thruster class (mm_thrust)                         %
%                                                                         %
% OUTPUT:                                                                 %
% roll controller class (CtrlRoll), ADCS controller classes (PitchCtrl,   %
% YawCtrl), and main motor thruster class (mm_thrust)                     %
%=========================================================================%

function [ CtrlRoll, PitchCtrl, YawCtrl, mm_thrust] = ...
    UpdatePosition(thruster, dCG, fire, CtrlRoll, PitchCtrl, YawCtrl, mm_thrust)     
    
    % position vector only changes when firings occur
    if any(fire) == true
        % indexers
        a = 1;
        b = 1;
        c = 1;
        
        for k = 1:length(thruster)
            if strcmp('roll',thruster(1,k).type) == true                
                CtrlRoll.R_thrusters(1,a).R = CtrlRoll.R_thrusters(1,a).R-dCG;
                a = a+1;
            elseif strcmp('pitch',thruster(1,k).type) == true
                PitchCtrl.P_thrusters(1,b).R = PitchCtrl.P_thrusters(1,b).R-dCG;
                b = b+1;                
            else
                YawCtrl.Y_thrusters(1,c).R = YawCtrl.Y_thrusters(1,c).R-dCG;
                c = c+1;                
            end
        end
			
        if fire(end) == true
            mm_thrust.R=mm_thrust.R-dCG;
        end
    end
end 

