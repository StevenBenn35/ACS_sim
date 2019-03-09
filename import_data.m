%=========================================================================% 
% import_data calls the simulaton parameter struct and organizes each     %
% parameter into various classes.                                         % 
%=========================================================================% 

function [ total_time, dt, g0, g, euler, omega, thruster, mass, I,      ...
           OF, CG0, dCG, tau, reorient_angle, sim_end, pitch, yaw,      ...
           theta_max, psi_max, dim, init_loc, Data_File, events,        ...
           mm_thrust, Nmodes, Rtank, bot_tank, htank, rho, damp, zeta,  ...
           thruster_props]= import_data(sim)
    %---------------------------------------------------------------------%
    % PARSE DATA           
    % "data_file"
    file1     = sim.data_file.output_file;
    delimiter = ' ';
    header    = 1;
    Data_File = data_file(file1,delimiter,header); 
    
    % spacecraft maneuver conditions
    sequence  = (sim.simulation.maneuvers.sequence)';
    spin_up   = sim.simulation.maneuvers.spin_up_rate*2*pi;
    coast     = sim.simulation.maneuvers.coast_time;
    nut_damp  = sim.simulation.maneuvers.nutation_damping_time;    
    reorient  = sim.simulation.maneuvers.reorient_time;
    sim_end   = sim.simulation.end_condition;
    reorient_angle  = sim.simulation.maneuvers.reorientation_angle;
    events    = sequence_completion(sequence,spin_up,coast,nut_damp, ...
                   reorient);   % maneuver completion class initialization   

    % "simulation"
    total_time = sim.simulation.time;        % sec
    g0         = sim.simulation.ref_gravity; % reference, gravity ft/sec^2
    Ag         = sim.simulation.G;           % G-load
    g          = Ag*g0;                      % gravity, ft/sec^2
       
    % "intial"
    euler = [sim.initial.phi,   ...
             sim.initial.theta, ...
             sim.initial.psi].*pi/180; % attitudes, deg to rad
    omega = [sim.initial.p,     ...
             sim.initial.q,     ...
             sim.initial.r].*2.*pi;     % rates, Hz to rad/sec
    
    % "stage"
    mass       = [sim.stage.stage_mass,    ...
                  sim.stage.oxidizer_mass, ...
                  sim.stage.fuel_mass];    % mass, lbm
    OF         = sim.stage.fuel.OF;        % Oxidizer-to-Fuel Ratio    
    Ixx        = sim.stage.inertias.Ixx;   % inertias, lbm*in^2
    Iyy        = sim.stage.inertias.Iyy;
    Izz        = sim.stage.inertias.Izz;
    Ixy        = sim.stage.inertias.Ixy;
    Ixz        = sim.stage.inertias.Ixz;
    Iyz        = sim.stage.inertias.Iyz;
    I          = [Ixx, Ixy, Ixz, Iyy, Iyz, Izz];
        
    Rtank    = sim.stage.fuel.tank_radius/12; % tank radius, ft
    htank    = sim.stage.fuel.tank_height/12; % tank height, ft
    tank_ref = [sim.stage.fuel.tank_CG.x, ...
                sim.stage.fuel.tank_CG.y, ...
                sim.stage.fuel.tank_CG.z];    % tank xyz, in (wrt nozzle origin)
    rho      = sim.stage.fuel.rho_fluid;      % fluid density, slug/ft^3
    
    htank_dummy = 0.5*12*htank;
    bot_tank = tank_ref-[htank_dummy,0,0];    % xyz bottom of tank, in 
    
    CG0   = [sim.stage.CGi.x, ...
             sim.stage.CGi.y, ...
             sim.stage.CGi.z];     % initial stage CG location, in
    CGf   = [sim.stage.CG_dry.x, ...
             sim.stage.CG_dry.y, ...
             sim.stage.CG_dry.z];  % dry stage CG location, in  

    % "thruster"   
    %f_thrust_roll  = sim.thruster.roll_thrust_mag;  % roll thrusters, lbf
    %f_thrust_pitch = sim.thruster.pitch_thrust_mag; % pitch thrusters, lbf
    %f_thrust_yaw   = sim.thruster.yaw_thrust_mag;   % yaw thrusters, lbf
    isp            = sim.thruster.isp;              % specific impulse of thrusters, sec
    min_fire       = sim.thruster.min_fire;         % minimum activation time, sec
    %R_mf_dt        = sim.thruster.R_min_fire_to_dt; % Ratio of min activation time to timestep
    %dt             = min_fire/R_mf_dt;              % simulation sample rate, sec
    dt             = sim.simulation.dt;
    dim            = floor(total_time/dt);          % preallocation dimension length
    thruster_props.Pc_avg = sim.thruster.Pc*6894.76; %psia to pa               
    thruster_props.z_avg  = sim.thruster.zeta;            
    thruster_props.wn_avg = sim.thruster.wn;             
    thruster_props.isp_avg = sim.thruster.isp;
    thruster_props.Pc = sim.thruster.Pc*6894.76; %psia to pa               
    thruster_props.z  = sim.thruster.zeta;            
    thruster_props.wn = sim.thruster.wn;             
    thruster_props.isp = sim.thruster.isp;
    thruster_props.U_isp  = sim.thruster.U_isp;             
    thruster_props.U_Pc   = sim.thruster.U_Pc;        
    thruster_props.U_z    = sim.thruster.U_zeta;           
    thruster_props.U_wn   = sim.thruster.U_wn;             
    thruster_props.T0     = sim.thruster.T0;           
    thruster_props.gam    = sim.thruster.gam;            
    thruster_props.MW     = sim.thruster.MW;         
    thruster_props.P_inf  = sim.thruster.P_inf;

    thruster_props.Dthroat= sim.thruster.Dt;
    thruster_props.ER = sim.thruster.ER;
    thruster_props.th = sim.thruster.th;

    thruster_props.Ru = sim.thruster.Ru;
    
    thruster_props.M_sup=ER_to_M(thruster_props.ER,1.05,thruster_props.gam);
    thruster_props.M_sub=ER_to_M(thruster_props.ER,0.1,thruster_props.gam);

    thruster_props.Te=thruster_props.T0/(1+0.5*(thruster_props.gam-1)*thruster_props.M_sup^2);

    thruster_props.Athroat=(pi/4)*thruster_props.Dthroat^2;
    thruster_props.Aexit=thruster_props.Athroat*thruster_props.ER;
    thruster_props.lam=0.5*(1+cos(thruster_props.th));

    thruster_props.Rg=thruster_props.Ru/thruster_props.MW;
    
    f_thrust_roll=P_to_F(thruster_props);
    f_thrust_roll=f_thrust_roll/4.448;
    f_thrust_pitch=f_thrust_roll;
    f_thrust_yaw = f_thrust_roll;
    
    dCG = [(CGf(1)-CG0(1))/(total_time/dt), ...    
           (CGf(2)-CG0(2))/(total_time/dt), ... 
           (CGf(3)-CG0(3))/(total_time/dt)];   % incremental changes in CG, in
       
    num_thrusters = numel(sim.thruster.location);
    init_loc      = zeros(num_thrusters, 3);
        
    for j=1:num_thrusters
        % thruster locations relative to THROAT_CYSY_DEF coordinate frame
        R = [sim.thruster.location(j).x, ...
             sim.thruster.location(j).y, ...
             sim.thruster.location(j).z];

        % thruster locations relative to CG
        R = [R(1)-CG0(1), R(2)-CG0(2), R(3)-CG0(3)];        
        init_loc(j,:) = R;
        
        if strcmp('roll',sim.thruster.vector(j).type) == true
            F   = [sim.thruster.vector(j).x, ...
                   sim.thruster.vector(j).y, ...
                   sim.thruster.vector(j).z].*f_thrust_roll; 
            type = sim.thruster.vector(j).type;
            sign = sim.thruster.vector(j).sign_M;                  
            thruster(j) = Thrusters(R,F,f_thrust_roll,isp,min_fire,type,sign); % roll thrusters
            
        elseif strcmp('pitch',sim.thruster.vector(j).type) == true
            F   = [sim.thruster.vector(j).x, ...
                   sim.thruster.vector(j).y, ...
                   sim.thruster.vector(j).z].*f_thrust_pitch; 
            type = sim.thruster.vector(j).type;
            sign = sim.thruster.vector(j).sign_M;                  
            thruster(j) = Thrusters(R,F,f_thrust_pitch,isp,min_fire,type,sign); % pitch thrusters
            
        elseif strcmp('yaw',sim.thruster.vector(j).type) == true
            F   = [sim.thruster.vector(j).x, ...
                   sim.thruster.vector(j).y, ...
                   sim.thruster.vector(j).z].*f_thrust_yaw;
            type = sim.thruster.vector(j).type;
            sign = sim.thruster.vector(j).sign_M;                  
            thruster(j) = Thrusters(R,F,f_thrust_yaw,isp,min_fire,type,sign); % yaw thrusters
        end
    end
    
    % "main_motor"
    T = sim.main_motor.thrust;                     % main motor thrust magnitude, lbf
    azi = sim.main_motor.offset_azimuth*pi/180;    % azimuth angle, thrust offset, deg
    ele = sim.main_motor.offset_elevation*pi/180;  % elevation angle, thrust offset, deg
    isp_motor = sim.main_motor.isp;                % main motor specific impulse, sec
    F_motor = T*[cos(ele)*cos(azi) cos(ele)*sin(azi) sin(ele)];
    R_mm   = -CG0;                                 % position vector to main motor wrt CGi, in
    mm_thrust = main_motor(R_mm,F_motor,isp_motor,dt,dim); % main motor class initialization 
    
    % "controller" (SHOULD NOT BE MODIFIED BY THE USER)
    tau        = 2.5;                                     % time constant, 1/sec
    pitch      = 2.0;                                     % pitch axis specifier
    yaw        = 3.0;                                     % yaw axis specifier
    theta_max  = sim.controller.pitch_disturbance*pi/180; % pitch disturbance limit, rad
    psi_max    = sim.controller.yaw_disturbance*pi/180;   % yaw disturbance limit, rad
        
    % "slosh"
    Nmodes = sim.slosh.modes;   % number of modes to use in slosh model
    damp   = sim.slosh.damping; % damping coefficient
    
    if Nmodes > 32
        error('Number of specified modes must not exceed 32.')
    else
        dJ1_roots=load('Roots_DJ1.txt');
        zeta=dJ1_roots(1:Nmodes,1);  % dJ1 roots
    end
    
end

