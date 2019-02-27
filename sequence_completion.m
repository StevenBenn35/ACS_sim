%=========================================================================% 
% ADD DOCUMENTATION                                                       % 
%=========================================================================% 
classdef sequence_completion < handle
    
    properties
        sequence       % array of maneuver sequence 
        spin_up        % spin up condition
        spin_down_tol  % spin down tolerance around zero, rad/sec
        coast_lim      % coast maneuver time limit, sec
        coast_timer    % coast maneuver timer
        nutdamp_lim    % nutation damping maneuver time limit, sec
        nutdamp_timer  % nutation damping maneuver timer
        reorient_lim   % reorienation maneuver time limit, sec
        reorient_timer % reorienation maneuver timer
        complete       % boolean for "maneuver complete"
        event          % current maneuver (element within sequence)

    end
    
    methods
        function obj = sequence_completion(seq,up,cst,nut_damp,reorient)
            % initialize properties
            obj.sequence       = seq;
            obj.spin_up        = up;
            obj.coast_lim      = cst;
            obj.coast_timer    = 0;
            obj.nutdamp_lim    = nut_damp;
            obj.nutdamp_timer  = 0;
            obj.reorient_lim   = reorient;
            obj.reorient_timer = 0;
            obj.complete       = zeros(1,length(obj.sequence));
            obj.event          = 1;           
            
        end
        
        function bool = eval_complete(obj,sequence,value)
            % evaluate the completion of current maneuver
            bool = false;            
            if sequence == 1
                if obj.coast_timer >= obj.coast_lim
                    bool = true;
                end
            elseif sequence == 3
                if value >= obj.spin_up
                    bool = true;
                end
            elseif sequence == 4
                if value <= obj.spin_down_tol && value >= -obj.spin_down_tol
                    bool = true;
                end
            elseif sequence == 5
                if obj.nutdamp_timer >= obj.nutdamp_lim
                    bool = true;
                end
            elseif sequence == 6
                if obj.reorient_timer >= obj.reorient_lim
                    bool = true;                    
                end
            else
                error('Invalid maneuver input');
            end
        end
    end
end

