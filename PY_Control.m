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
        t_trail_off
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
        Fsav
        decayy
                            
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
            
            for k=1:length(pT)
                mag_F=nonzeros(obj.P_thrusters(1,k).F);
                sign_F=mag_F/abs(mag_F);
                obj.P_thrusters(1,k).unit_F=obj.P_thrusters(1,k).F;
                obj.P_thrusters(1,k).unit_F(obj.P_thrusters(1,k).unit_F~=0)=1;
                obj.P_thrusters(1,k).unit_F=obj.P_thrusters(1,k).unit_F.*sign_F;
            end
            
            for k=1:length(yT)
                mag_F=nonzeros(obj.Y_thrusters(1,k).F);
                sign_F=mag_F/abs(mag_F);
                obj.Y_thrusters(1,k).unit_F=obj.Y_thrusters(1,k).F;
                obj.Y_thrusters(1,k).unit_F(obj.Y_thrusters(1,k).unit_F~=0)=1;
                obj.Y_thrusters(1,k).unit_F=obj.Y_thrusters(1,k).unit_F.*sign_F;
            end            
            
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
                obj.t_trail_off   = zeros(1,LPT);
                obj.fired         = false(dim+1,LPT);
                obj.coast(1,1:LPT)= true;
                obj.Fsav          = zeros(dim+1,LPT);
                obj.decayy         = false(1,LPT);
            elseif type == 3
                obj.t_total_fire  = zeros(1,LYT);
                obj.t_single_fire = zeros(1,LYT);
                obj.t_trail_off    = zeros(1,LYT);
                obj.fired         = false(dim+1,LYT);
                obj.coast(1,1:LYT)= true; 
                obj.Fsav          = zeros(dim+1,LYT);
                obj.decayy         = false(1,LYT);
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

        function [obj,M,thruster_props] = controller_MAIN(obj,currPos,currRate,thruster,M_prev,type,thruster_props,i)
            % calculate the switching line crossings
            obj = sl_crossings(obj,currPos,currRate,i);
            
            % check which thrusters fired last timestep
            history = get_fire_history(obj,i);
            
            % initiate burn command
            [obj,M,thruster_props] = burn_commands(obj,history,M_prev,type,thruster_props,i);
            
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
        
        function [obj,M,thruster_props] = burn_commands(obj,history,M_prev,type,thruster_props,i)
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
                if obj.t_single_fire(1,index(1))< obj.tmin && obj.decayy(1,index)==false
                    [obj,M] = repeat_fire(obj,index,M_prev,type,i);
                elseif obj.t_single_fire(1,index(1))> (obj.tmin-obj.dt) && obj.decayy(1,index)==false
                    obj.decayy(1,index) = true;
                    [obj,M] = trail_off(obj,thruster,thruster_props,index,type,i);
                elseif obj.decayy(1,index) == true
                    [obj,M] = trail_off(obj,thruster,thruster_props,index,type,i);
                else
                    [obj,M,thruster_props] = fire_by_location(obj,history,thruster,thruster_props,type,i);
                end
                
            else
                [obj,M,thruster_props] = fire_by_location(obj,history,thruster,thruster_props,type,i);
            end
        end
        
        function [obj,M] = trail_off(obj,thruster,thruster_props,index,type,i)
            
            P_inf = thruster_props.P_inf;
            P0_ss = thruster_props.Pc;
            z     = thruster_props.z;
            wn    = thruster_props.wn;
            T0_ss = thruster_props.T0;
            ga    = thruster_props.gam;
            M_sup = thruster_props.M_sup;
            lam   = thruster_props.lam;
            ER    = thruster_props.ER;
            At    = thruster_props.Athroat;
            Rg    = thruster_props.Rg;
            M_sub = thruster_props.M_sub;
            Ae    = thruster_props.Aexit;
            
            t     = obj.t_trail_off(1,index);
            
            P0=P_inf+(P0_ss-P_inf)*(exp(-z*wn*t)*(z*sinh(sqrt(z^2-1)*wn*t)/sqrt(z^2-1)+cosh(sqrt(z^2-1)*wn*t)));
            T0=T0_ss*(P0/P0_ss)^(ga/(ga-1));
    
            if (P0/P_inf) >= ((0.5*(ga+1))^(ga/(ga-1)))  % choked
        
                Pe=P0/(1+0.5*(ga-1)*M_sup^2)^(ga/(ga-1));
                CF=lam*ga*sqrt(2/(ga-1)*(2/(ga+1))^((ga+1)/(ga-1))*(1-(Pe/P0)^((ga-1)/ga)))+ER*((Pe-P_inf)/P0);
                force=CF*P0*At;

            else % unchoked
        
                mdot=At*P0*sqrt((2*ga/((ga-1)*Rg*T0))*((P_inf/P0)^(2/ga)-(P_inf/P0)^((ga+1)/ga)));
                Pe=P0/(1+0.5*(ga-1)*M_sub^2)^(ga/(ga-1));
                Te=T0/(1+0.5*(ga-1)*M_sub^2);
                Ve=M_sub*sqrt(ga*Rg*Te);
        
                force=lam*mdot*Ve+Ae*(Pe-P_inf);
            end
            
            obj.fired(i,index)=true;
            
            if force < 0 
                obj.decayy(1,index) = false;
                force = 0;
                obj.fired(i,index)=false;
            end
            
            
            thruster(index).F=(force/4.448)*(thruster(index).F/norm(thruster(index).F));
            thruster(index).mag=norm(thruster(index).F);
            obj.Fsav(i,index)=thruster(index).mag;
            
            if type == 2
                obj.P_thrusters(1,index).F = thruster(index).F;
                obj.P_thrusters(1,index).mag= thruster(index).mag;
            elseif type == 3
                obj.Y_thrusters(1,index).F = thruster(index).F;
                obj.Y_thrusters(1,index).mag= thruster(index).mag;
            end
            
            wT=thruster(index);
            M = calc_momentt(obj,wT);          
             
        end
        
        function obj = update_timers(obj,i)
            % update timers of activated thrusters of the current timestep.
            for k=1:length(obj.fired(i,:))
                if obj.fired(i,k)== true
                    obj.t_single_fire(1,k) = obj.t_single_fire(1,k)+obj.dt;
                    obj.t_total_fire(1,k)  = obj.t_total_fire(1,k)+obj.dt;
                    
                    if obj.decayy(1,k)==true
                        obj.t_trail_off(1,k)=obj.t_trail_off(1,k)+obj.dt;
                    end
                else
                    % reset timers of deactivated thrusters.
                    obj.t_single_fire(1,k) = 0.0;
                    obj.t_trail_off(1,k) = 0.0;
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
            
            obj.Fsav(i,:)=obj.Fsav(i-1,:);
            
            % record current thruster activation 
            for k=1:length(index)
                obj.fired(i,index(k))=true;
            end
            
        end
        
        function [obj,M,thruster_props] = fire_by_location(obj,history,thruster,thruster_props,type,i) 
            
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
                            obj.fired(i,k)=true;
                            if obj.t_single_fire(1,k)==0
                                thruster_props=add_variability(thruster_props);
                                force=P_to_F(thruster_props);
                                thruster(k).F=(force/4.448)*thruster(k).unit_F;
                                thruster(k).mag=norm(thruster(k).F);
                                thruster(k).isp=thruster_props.isp;
                                
                                if type == 2
                                    obj.P_thrusters(k).F=thruster(k).F;
                                    obj.P_thrusters(k).mag=thruster(k).mag;
                                elseif type == 3
                                    obj.Y_thrusters(1,k).F = thruster(k).F;
                                    obj.Y_thrusters(1,k).mag= thruster(k).mag;
                                end
                            
                            end
                            obj.Fsav(i,k)=thruster(k).mag;
                            wT = [wT, thruster(k)];
                            
                        end                                              
                    end
                    
                else
                    % fire thrusters to increase ang vel (below sl#2)
                    for k=1:LH
                        if thruster(k).sign_M == 1
                            obj.fired(i,k)=true;
                            if obj.t_single_fire(1,k)==0
                                thruster_props=add_variability(thruster_props);
                                force=P_to_F(thruster_props);
                                thruster(k).F=(force/4.448)*thruster(k).unit_F;
                                thruster(k).mag=norm(thruster(k).F);
                                thruster(k).isp=thruster_props.isp;
                            end
                            obj.Fsav(i,k)=thruster(k).mag;
                            wT = [wT, thruster(k)];
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

