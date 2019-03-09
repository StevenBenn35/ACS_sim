%=========================================================================% 
% Thrusters.m organizes the properties of each ADCS thruster into a class %                                                                       
%=========================================================================%

classdef Thrusters < handle
    
    properties
        R         % position vector, in
        F         % thrust vector, lbf
        unit_F    % thrust unit vector
        mag       % magnitude of thrust vector, lbf
        isp       % specific impulse, sec
        t_min     % minimum thruster activation time, sec
        type      % thruster type (roll, pitch, yaw)
        sign_M    % sign of intended induced moment
    end
    
    methods
        function obj = Thrusters(r, f, mag_f, Isp, t, Type, Sign)
            obj.R = r;
            obj.F = f;
            obj.mag = mag_f;
            obj.isp = Isp;
            obj.t_min = t;  
            obj.type = Type;
            obj.sign_M = Sign;            
        end
    end    
end