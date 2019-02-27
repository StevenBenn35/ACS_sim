%=========================================================================% 
% R_control organizes the roll thrusters and their properties into a      %
% class. The spin-up or spin-down command is initiated here. Moment       %
% calculations and fuel consumption for the roll thrusters is handled     %
% here.                                                                   %
%=========================================================================%

classdef R_Control < handle 
    
    properties        
        R_thrusters    % roll thrusters       
        t_total_fire   % total time fired for each thruster, sec
        t_single_fire  % time of individual firing segments for each thruster, sec
        tmin           % minimum thruster activation time, sec
        dt             % timestep, sec
        fired          % thruster activation history
        Drate_min      % minimum induced change in rate, rad/sec
        mburn          % array of mass burned at each timestep, lbm
        g0             % reference gravity, 32.174 ft/sec^2         
    end
    
    methods
        function obj = R_Control(thruster, complete, I, gref, t, dim)
            % create array of roll thrusters
            LT = length(thruster);
            rT = [];        
            for k=1:LT
                if strcmp('roll',thruster(k).type) == true
                    rT = [rT,thruster(k)];
                end            
            end            
            obj.R_thrusters = rT;
            
            % initialize class properties 
            LRT               = length(rT);                      
            obj.t_total_fire  = zeros(1,LRT);
            obj.t_single_fire = zeros(1,LRT);
            obj.dt            = t;            
            obj.fired         = false(dim+1,LRT);
            obj.g0            = gref;
            obj.tmin          = thruster(1,1).t_min;           
            obj.mburn         = zeros(dim+1,1);

            % calculate the minimum rate change based on tmin
            obj = calc_min_rate_change(obj,rT,I);
            
            % set spin-down tolerance to +/- min rate change
            complete.spin_down_tol = obj.Drate_min;
        end
        
        function [obj,M] = controller_MAIN(obj,spin_up,i)
            % apply a burn to spin up or spin down
            [obj,M] = burn_command(obj,spin_up,i);
            
            % update firing timers of activated thrusters
            obj = update_timers(obj,i);
            
            % calculate fuel burned
            obj = calc_mburn(obj,i);
        end
        
        function [obj,M] = burn_command(obj,spin_up,i)
            wT       = [];                      
            thruster = obj.R_thrusters;
            LT       = length(thruster);
            
            % specify spin up or spin down
            if spin_up == true
                condition = 1;
            else
                condition = -1;
            end
            
            % create array of thrusters fired at current timestep 
            for k=1:LT
                if thruster(1,k).sign_M == condition
                    wT = [wT, thruster(k)];
                    obj.fired(i,k) = true;
                end
            end
            
            % calculate sum of moments of activated thrusters
            M = calc_momentt(obj,wT);

        end
        
        function obj = update_timers(obj,i)
            % update activated thruster timers and zero deactivated thruster timers
            for k=1:length(obj.fired(i,:))
                if obj.fired(i,k)== true
                    obj.t_single_fire(1,k)=obj.t_single_fire(1,k)+obj.dt;
                    obj.t_total_fire(1,k) = obj.t_total_fire(1,k)+obj.dt;
                else
                    obj.t_single_fire(1,k) = 0.0;
                end
            end            
        end
        
        function obj = calc_mburn(obj,i)
            sum_mburn=0.0;
            thruster = obj.R_thrusters;
            
            % calculate and sum mass burned from activated thrusters
            for k=1:length(obj.fired(i,:))
                if obj.fired(i,k) == true
                    mburn_i = thruster(1,k).mag*obj.dt/(obj.g0*thruster(1,k).isp);
                else
                    mburn_i = 0;
                end
                
                sum_mburn = sum_mburn+mburn_i;
            end
            obj.mburn(i)=sum_mburn*obj.g0; % conversion to lbm            
        end
        
        function obj = calc_min_rate_change(obj, thrusters, I)
            % calculate moment of arbitrary thruster couple
            aT = thrusters(1);
            M  = calc_momentt(obj,aT);
            
            % minimum rate change based on tmin
            obj.Drate_min = abs(M(1)*obj.dt/I);           
        end
        
        function M = calc_momentt(obj,thrust_array)
            % sum moments of activated thrusters
            if isempty(thrust_array) == false
                sum_M = [0,0,0];
                for k = 1:length(thrust_array)
                    r = thrust_array(k).R;
                    f = thrust_array(k).F;

                    rx = r(1); ry = r(2); rz = r(3);
                    fx = f(1); fy = f(2); fz = f(3);

                    Mx = ry*fz-rz*fy;
                    My = -(rx*fz-rz*fx);
                    Mz = rx*fy-ry*fx;

                    sum_M = sum_M+[Mx,My,Mz];
                end

                M = sum_M.*(12*obj.g0); % conversion to lbm-in^2/s^2
            else
                M = [0,0,0];
            end
        end       
        
    end % end of methods
    
end % end of class

