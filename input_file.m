% SIMULATION INPUT FILE %

function sim = input_file()
%-------------------------------------------------------------------------%
% DATA FILE SPECIFICATION
    sim.data_file.output_file = 'states.txt';
    
%-------------------------------------------------------------------------%
% GENERAL SIMULATION PARAMETERS AND MANEUVERS
    sim.simulation.time                            = 60;
    sim.simulation.dt                              = 0.001;
    sim.simulation.end_condition                   = 2;
    sim.simulation.ref_gravity                     = 32.174; 
    sim.simulation.G                               = 0;
    sim.simulation.maneuvers.sequence              = [5];
    sim.simulation.maneuvers.spin_up_rate          = 5.0; 
    sim.simulation.maneuvers.coast_time            = 10; 
    sim.simulation.maneuvers.nutation_damping_time = 50; 
    sim.simulation.maneuvers.reorient_time         = 20; 
    sim.simulation.maneuvers.reorientation_angle   = 15; 

%-------------------------------------------------------------------------%
% INITIAL ORIENTATION AND RATES
    sim.initial.phi   = 0; 
    sim.initial.theta = 0; 
    sim.initial.psi   = 0; 
    sim.initial.p     = 0.0; 
    sim.initial.q     = 0.015; 
    sim.initial.r     = 0.015; 
    
%-------------------------------------------------------------------------%
% STAGE PROPERTIES
    sim.stage.stage_mass       = 164.49;
    sim.stage.oxidizer_mass    = 96.4;
    sim.stage.fuel_mass        = 13.8;
    sim.stage.fuel.OF          = 5;
    sim.stage.fuel.tank_radius = 6;
    sim.stage.fuel.tank_height = 25.6;
    sim.stage.fuel.tank_CG.x   = 28.6;
    sim.stage.fuel.tank_CG.y   = 0;
    sim.stage.fuel.tank_CG.z   = 0;
    sim.stage.fuel.rho_fluid   = 2.697;
    sim.stage.inertias.Ixx     = 2.6838038e+03;
    sim.stage.inertias.Iyy     = 5.0528608e+04;
    sim.stage.inertias.Izz     = 5.0552156e+04;
    sim.stage.inertias.Ixy     = 1.6346237e+02;
    sim.stage.inertias.Ixz     = 4.3001915e+01;
    sim.stage.inertias.Iyz     = -6.5213446e+00;
    sim.stage.CGi.x            = 2.6515205e+01;
    sim.stage.CGi.y            = 3.1291079e-02;
    sim.stage.CGi.z            = 1.7648531e-02;
    sim.stage.CG_dry.x         = 3.0273238e+01;
    sim.stage.CG_dry.y         = 1.0657068e-01;
    sim.stage.CG_dry.z         = 6.0107096e-02;

%-------------------------------------------------------------------------%
% ADCS THRUSTER PROPERTIES
    sim.thruster.roll_thrust_mag   = 5;
    sim.thruster.pitch_thrust_mag  = 5;
    sim.thruster.yaw_thrust_mag    = 5;
    sim.thruster.min_fire          = 0.2;
%     sim.thruster.R_min_fire_to_dt  = 5;
    sim.thruster.isp               = 274.78;
%     sim.thruster.Pc                = 188.706; %psia
    sim.thruster.Pc                = 69.618117;
%     sim.thruster.zeta              = 1.7025;
    sim.thruster.zeta              = 1.25;
%     sim.thruster.wn                = 31.12;
    sim.thruster.wn                = 500;
    sim.thruster.U_isp             = 1.48111;
%     sim.thruster.U_Pc              = 4.29153; %psia
    sim.thruster.U_Pc              = 1.5567; %psia
%     sim.thruster.U_zeta            = 0.1382;
    sim.thruster.U_zeta            = 0.10147;
%     sim.thruster.U_wn              = 0.858;
    sim.thruster.U_wn              = 13.785;
    sim.thruster.T0                = 3224.1;
    sim.thruster.gam               = 1.12851;
    sim.thruster.MW                = 24.9382;
    sim.thruster.P_inf             = 15.54e+03; %Pa

    sim.thruster.Dt                = 0.4216e-02;
    sim.thruster.ER                = 9.438;
    sim.thruster.th                = 8.5*pi/180;

    sim.thruster.Ru                = 8314.4126;
    
    sim.thruster.location(1).x  = 28.6;
    sim.thruster.location(1).y  = 6;
    sim.thruster.location(1).z  = 0;
    sim.thruster.location(2).x  = 28.6;
    sim.thruster.location(2).y  = 6;
    sim.thruster.location(2).z  = 0;
    sim.thruster.location(3).x  = 15.8;
    sim.thruster.location(3).y  = 0;
    sim.thruster.location(3).z  = 6;
    sim.thruster.location(4).x  = 15.8;
    sim.thruster.location(4).y  = 0;
    sim.thruster.location(4).z  = -6;
    sim.thruster.location(5).x  = 15.8;
    sim.thruster.location(5).y  = -6;
    sim.thruster.location(5).z  = 0;
    sim.thruster.location(6).x  = 15.8;
    sim.thruster.location(6).y  = 6;
    sim.thruster.location(6).z  = 0;
    
    sim.thruster.vector(1).x       = 0;
    sim.thruster.vector(1).y       = 0;
    sim.thruster.vector(1).z       = -1;
    sim.thruster.vector(1).type    = 'roll';
    sim.thruster.vector(1).sign_M  = -1;
    sim.thruster.vector(2).x       = 0;
    sim.thruster.vector(2).y       = 0;
    sim.thruster.vector(2).z       = 1;
    sim.thruster.vector(2).type    = 'roll';
    sim.thruster.vector(2).sign_M  = 1;
    sim.thruster.vector(3).x       = 0;
    sim.thruster.vector(3).y       = 0;
    sim.thruster.vector(3).z       = -1;
    sim.thruster.vector(3).type    = 'pitch';
    sim.thruster.vector(3).sign_M  = -1;
    sim.thruster.vector(4).x       = 0;
    sim.thruster.vector(4).y       = 0;
    sim.thruster.vector(4).z       = 1;
    sim.thruster.vector(4).type    = 'pitch';
    sim.thruster.vector(4).sign_M  = 1;
    sim.thruster.vector(5).x      = 0;
    sim.thruster.vector(5).y      = 1;
    sim.thruster.vector(5).z      = 0;
    sim.thruster.vector(5).type   = 'yaw';
    sim.thruster.vector(5).sign_M = -1;
    sim.thruster.vector(6).x      = 0;
    sim.thruster.vector(6).y      = -1;
    sim.thruster.vector(6).z      = 0;
    sim.thruster.vector(6).type   = 'yaw';
    sim.thruster.vector(6).sign_M = 1;
    
%-------------------------------------------------------------------------%
% MAIN MOTOR PROPERTIES
    sim.main_motor.thrust           = 0;
    sim.main_motor.offset_azimuth   = 0.0;
    sim.main_motor.offset_elevation = 0.0;
    sim.main_motor.isp              = 300;

%-------------------------------------------------------------------------%
% NUTATION DAMPING CONTROLLER PROPERTIES
    sim.controller.pitch_disturbance = 3.0;
    sim.controller.yaw_disturbance   = 3.0;

%-------------------------------------------------------------------------%
% SLOSH MODEL PARAMETERS
    sim.slosh.modes   = 10;
    sim.slosh.damping = 0.7;

end

