%=========================================================================% 
% main_motor.m is a class that contains the main motor properties. The    %
% moment induced on the stage during activation and the consumed mass is  %
% calculated within this class.                                           %
%=========================================================================%

classdef main_motor < handle
    properties
        R       % position vector main motor wrt stage CG, in
        F       % thrust vector, lbf
		isp     % specific impulse, sec
		fired   % boolean array of firing history
		mburn   % array of mass burned at each timestep, lbm
        t_fire  % thruster activation timer, sec
        dt      % timestep, sec
    end
    
    methods
        function obj = main_motor(r, f, isp_ax, delta_T,dim)
            % intialization
            obj.R      = r;
            obj.F      = f;
			obj.isp    = isp_ax;
            obj.t_fire = 0;
            obj.dt     = delta_T;
			obj.mburn  = zeros(dim+1,1);
			obj.fired  = false(dim+1,1);
        end
        
        function M = activate(obj,i)
            % calculate moment induced my main motor
            r = obj.R;
            f = obj.F;

            rx = r(1); ry = r(2); rz = r(3);
            fx = f(1); fy = f(2); fz = f(3);

            Mx =  ry*fz-rz*fy;
            My = -(rx*fz-rz*fx);
            Mz =  rx*fy-ry*fx;      

            M = [Mx,My,Mz].*(12*32.174); % lbf-in to lbm-in^2/s^2
            
            % update firing history and timer
            obj.t_fire = obj.t_fire+obj.dt;
			obj.fired(i) = true;
            
        end %end of activate
		
		function obj = calc_mburn(obj,g0,i)
            % calculate mass burned at current timestep
            f         = obj.F;
            sum_mburn = 0;
            
            for k=1:length(f)
                sum_mburn = sum_mburn+(f(k)*obj.dt/(g0*obj.isp))*g0; % slug to lbm
            end
            
            % populate mass burned array
            obj.mburn(i,1) = sum_mburn;
            
        end %end calc_mburn		
    end %end of methods    
end %end of class
