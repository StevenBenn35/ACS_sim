%=========================================================================% 
% PY_control organizes the pitch and yaw thrusters and their properties   %
% into a class. The ADCS control logic is implemented here. Moment        %
% calculations and fuel consumption for the pitch and yaw thrusters is    %
% handled here.                                                           %
%=========================================================================%

classdef PY_Control < handle
    
    properties 
        
        % thrusters
        P_thrusters
        Y_thrusters
        
        % orientation 
        current_angle       
        current_rate
        
        % timing
        t_total_fire
        t_single_fire
        tmin
        dt       
        
        % location and firing feedback
        inside_deadband  
        outside_deadband  
        fired           
        coast

        % limits
        position_MAX
        rate_MAX
        ss_Limit_x   
        ss_Limit_y
        reorient
        
        % switching line properties
        slope        
        y_intercept_line1
        y_intercept_line2
        
        sl1
        line1_rate       
        line1_position 
        
        sl2
        line2_rate
        line2_position   
        
        % other
        mburn
        g
                            
    end
    
    methods
        function obj = PY_Control( type, maxPos, tau, I, g0, t, thruster, dim, reorient_angle)
            
            LT = length(thruster);
            pT = [];
            yT = [];
            
            % separate pitch and yaw thrusters into different arrays
            for k=1:LT
                if type ==2
                    if strcmp('pitch',thruster(k).type) == true
                        pT = [pT,thruster(k)];
                    end
                elseif type ==3
                    if strcmp('yaw',thruster(k).type) == true
                        yT = [yT,thruster(k)];
                    end
                end
            end
            
            obj.P_thrusters = pT;
            obj.Y_thrusters = yT;
            
            % array lengths for pitch and yaw thrusters
            LPT = length(pT);
            LYT = length(yT);
            
            % initialize class properties
            obj.tmin          = thruster(1,1).t_min;
            obj.dt            = t;
            obj.g             = g0;
            
            obj.inside_deadband  = false(dim,1);
            obj.outside_deadband = false(dim,1);
            
            % create arrays of pitch/yaw firing properties
            if type == 2
                obj.t_total_fire  = zeros(1,LPT);
                obj.t_single_fire = zeros(1,LPT);
                obj.fired         = false(dim+1,LPT);
                obj.coast(1,1:LPT)= true;
            elseif type == 3
                obj.t_total_fire  = zeros(1,LYT);
                obj.t_single_fire = zeros(1,LYT);
                obj.fired         = false(dim+1,LYT);
                obj.coast(1,1:LYT)= true;                
            else
                error('Invalid type input');
            end

            obj.sl1               = zeros(dim,2);            
            obj.line1_rate        = zeros(dim+1,1);    
            obj.line1_position    = zeros(dim+1,1);
            
            obj.sl2               = zeros(dim,2);            
            obj.line2_rate        = zeros(dim+1,1);
            obj.line2_position    = zeros(dim+1,1);
                                         
            % calculate the maximum steady state rate. 
            if type == 2
                obj = calc_rate_MAX(obj,pT,I,type);
            elseif type ==3
                obj = calc_rate_MAX(obj,yT,I,type);
            else
                error('Invalid type input');
            end
            
            obj.reorient = reorient_angle*pi/180;
            obj.position_MAX = maxPos;
            obj.slope        = -1/tau;
            
            obj.y_intercept_line1  = obj.rate_MAX-obj.slope*(obj.position_MAX+obj.reorient);
            obj.y_intercept_line2  = -obj.rate_MAX-obj.slope*(-obj.position_MAX+obj.reorient);
            
            obj.ss_Limit_x = [ obj.position_MAX, obj.position_MAX, ...
                              -obj.position_MAX, -obj.position_MAX,...
                               obj.position_MAX]+obj.reorient;
            
            obj.ss_Limit_y = [ obj.rate_MAX, -obj.rate_MAX, ...
                              -obj.rate_MAX, obj.rate_MAX,  ...
                               obj.rate_MAX];
                           
            obj.mburn = zeros(dim+1,1);
            
        end
        
        function [obj,M] = controller_MAIN(obj,currPos,currRate,thruster,M_prev,type,i)
            % calculate the switching line crossings
            obj = sl_crossings(obj,currPos,currRate,i);
            
            % check which thrusters fired last timestep
            history = get_fire_history(obj,i);
            
            % initiate burn command
            [obj,M] = burn_commands(obj,history,M_prev,type,i);
            
            % update timers of all applicable thrusters
            obj = update_timers(obj,i);
            
            % calculate propellant mass burned from all applicable thrusters
            obj = calc_mburn(obj,thruster,i);
            
        end
        
        function obj = sl_crossings(obj,currPos,currRate,i)
            % current angle and angular velocity
            obj.current_angle = currPos;
            obj.current_rate  = currRate;
           
            % switching line properties
            x = obj.current_angle;
            y = obj.current_rate;
            m = obj.slope;
            b1 = obj.y_intercept_line1;
            b2 = obj.y_intercept_line2;
            
            % calculate switching line #1 crossings
            obj.line1_rate(i)     =  m*x+b1;
            obj.line1_position(i) = (y-b1)/m;            
            obj.sl1(i,1)          =  x;
            obj.sl1(i,2)          =  obj.line1_rate(i);            
            obj.sl1(i,3)          =  obj.line1_position(i);
            obj.sl1(i,4)          =  y;
            
            % calculate switching line #2 crossings
            obj.line2_rate(i)     =  m*x+b2;
            obj.line2_position(i) = (y+b2)/m;
            obj.sl2(i,1)          =  x;
            obj.sl2(i,2)          =  obj.line2_rate(i);            
            obj.sl2(i,3)          =  obj.line2_position(i);
            obj.sl2(i,4)          =  y;
            
        end
        
        function history = get_fire_history(obj,i)
            % assume all thrusters are off at first timestep (i=1)
            % for i > 1, grab previous timestep. 
            if i == 1
                history = false(1,length(obj.fired(i)));
            else
                history = obj.fired(i-1,:);
            end
        end
        
        function [obj,M] = burn_commands(obj,history,M_prev,type,i)
            % use the applicable thrusters for pitch/yaw controller
            if type == 2
                thruster = obj.P_thrusters;
            elseif type == 3
                thruster = obj.Y_thrusters;
            end
            
            % check if a fire occured last timestep. if not, fire based on phase plane location.  
            if any(history) == true
                index = find(history);
                
                % repeat the previous fire if tmin hasn't been reached.
                % fire based on phase plane location if tmin was reached.
                if obj.t_single_fire(1,index(1))<= obj.tmin
                    [obj,M] = repeat_fire(obj,index,M_prev,type,i);
                else
                    [obj,M] = fire_by_location(obj,history,thruster,i);
                end
                
            else
                [obj,M] = fire_by_location(obj,history,thruster,i);
            end
        end
        
        function obj = update_timers(obj,i)
            % update timers of activated thrusters of the current timestep.
            for k=1:length(obj.fired(i,:))
                if obj.fired(i,k)== true
                    obj.t_single_fire(1,k) = obj.t_single_fire(1,k)+obj.dt;
                    obj.t_total_fire(1,k)  = obj.t_total_fire(1,k)+obj.dt;
                else
                    % reset timers of deactivated thrusters.
                    obj.t_single_fire(1,k) = 0.0;
                end
            end
        end
        
        function [obj,M]=repeat_fire(obj,index,M_prev,type,i)
            % return the previous moment of the axis of interest.
            if type == 2
                M = [0,M_prev(2),0];
            elseif type == 3
                M = [0,0,M_prev(3)];
            end
            
            % record current thruster activation 
            for k=1:length(index)
                obj.fired(i,index(k))=true;
            end
            
        end
        
        function [obj,M] = fire_by_location(obj,history,thruster,i) 
            
            % current ang vel and switching line #1 ang vel
            line1rate   = obj.line1_rate(i);
            currRate   = obj.current_rate;
            
            % LH == length of firing history array
            % wT == array of 'which thrusters' are activated
            LH = length(history);
            wT = [];
            
            % evaluate phase plane location
            obj = eval_location(obj,i);
            
            % determine burn command based on phase plane location
            if obj.inside_deadband(i) == true
                obj.fired(i,:)=false;
                obj.coast(i,:)=true;
                M = [0,0,0];
            else
                if currRate > line1rate
                    % fire thrusters to decrease ang vel (above sl #1)
                    for k=1:LH
                        if thruster(k).sign_M == -1
                            wT = [wT, thruster(k)];
                            obj.fired(i,k)=true;
                        end                                              
                    end                                     
                else
                    % fire thrusters to increase ang vel (below sl#2)
                    for k=1:LH
                        if thruster(k).sign_M == 1
                            wT = [wT, thruster(k)];
                            obj.fired(i,k)=true;
                        end                                               
                    end                    
                end
                
                % sum moments of thrusters in wT
                M = calc_momentt(obj,wT);

            end
        end
        
        function obj = eval_location(obj,i)
            % current ang vel and ang vel at switching line crossings
            currRate  = obj.current_rate;
            line1rate = obj.line1_rate(i);
            line2rate = obj.line2_rate(i);
            
            % if current ang vel is between both switching lines or not...
            if currRate < line1rate && currRate > line2rate
                obj.inside_deadband(i)  = true;
                obj.outside_deadband(i) = false;
            else
                obj.inside_deadband(i)  = false;
                obj.outside_deadband(i) = true;
            end
            
        end
        
        function obj = calc_rate_MAX(obj,thruster,I,type)
            aT = thruster(1);
            M  = calc_momentt(obj,aT);
            if type == 2
                obj.rate_MAX = abs(M(2))*obj.tmin/I;
            elseif type == 3
                obj.rate_MAX = abs(M(3))*obj.tmin/I;
            else
                error('Invalid type input.');
            end   
        end
        
        function M = calc_momentt(obj,thrust_array)
            % create summing variable
            sum_M = [0,0,0];
            
            % loop through thrusters and calculate moment
            for k=1:length(thrust_array)
                r = thrust_array(k).R;
                f = thrust_array(k).F;
                
                rx = r(1); ry = r(2); rz = r(3);
                fx = f(1); fy = f(2); fz = f(3);
               
                Mx =  ry*fz-rz*fy;
                My = -(rx*fz-rz*fx);
                Mz =  rx*fy-ry*fx;
                
                sum_M = sum_M+[Mx,My,Mz];
            end

            % convert from lbf-in to lbm-in^2/s^2
            M =  sum_M.*(12*obj.g);        
        end
        
        function obj = calc_mburn(obj,thruster,i)
            % create summing variable
            sum_mburn=0.0;
            
            % calculate mass prop burned from activated thrusters
            for k=1:length(obj.fired(i,:))
                if obj.fired(i,k) == true
                    mburn_i = thruster(1,k).mag*obj.dt/(obj.g*thruster(1,k).isp);
                else
                    mburn_i = 0;
                end
                
                sum_mburn = sum_mburn+mburn_i;
            end
            
            % convert from slug to lbm
            obj.mburn(i)=sum_mburn*obj.g; 
        end
        
    end % end of methods 
    
end % end of class

