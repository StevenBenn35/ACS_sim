%=========================================================================% 
% Simulation of Hybrid Upper Stage for NanoLaunch Vehicles                %
% NASA Phase II SBIR NNX16CD27C                                           %
%                                                                         %
% For Parabilis Space Technologies                                            %
%                                                                         %
% Written by: Steven Bennett                                              %
%             Graduate Research Assistant, Utah State University          %
%                                                                         % 
% Data is in English Units                                                % 
%                                                                         %
% Aircraft body-fixed coordinate frame used:                              %      
%   x: Through the nose of stage from CG location                         %
%   y: Out the right side of craft from CG location                       %
%   z: Normal direction, downwards from CG location                       %
%                                                                         %
%'Main.m' establishes the structure of the simulation.                    %                                                                       
%=========================================================================% 

clc; clear variables; close all; 
addpath Utilities
addpath Output_Files

%-------------------------------------------------------------------------% 
% INPUT SIMULATION PARAMETERS                                             
sim = input_file();

[total_time, dt, g0, g, angles, rates, thruster, mass0, I0, OF, CG0,  ...
 dCG, tau, reorient, sim_end, pitch, yaw, theta_max, psi_max, dim,    ...
 init_loc, Data_File, sim_complete, mm_thrust, Nroots, Rtank,         ...
 bot_tank, htank, rho, damp, zeta] = import_data(sim);

%-------------------------------------------------------------------------%
% PREALLOCATION OF DATA ARRAYS
state            = zeros(9,dim);     % state vector (rates,attitudes, and body-fixed angles), [rad/sec,rad,rad]
M                = zeros(dim+1,3);   % moment due to control actuation [lbf-in]
M_sl             = zeros(dim+1,3);   % moments due to slosh [lbf-in] 
M_off            = zeros(dim+1,3);   % moments due to tank CG offset [lbf-in]
M_ax             = zeros(dim+1,3);   % moments due to thrust offset [lbf-in] 
M_total          = zeros(dim+1,3);   % total moment vector [lbf-in]
F_sl             = zeros(dim+1,3);   % forces due to slosh [lbf]
time             = zeros(dim,1);     % time [sec]                         
mass             = zeros(dim+1,3);   % [stage mass, oxidizer mass, fuel mass] [lbm]
ADCS_mass_burned = zeros(dim+1,1);   % total ADCS fuel consumed at each timestep [lbm]

%-------------------------------------------------------------------------%
% INITIAL PARAMETERS
i  = 1;                            
I  = I0;
CG = CG0;
mass(i,:)    = mass0;
fuel_burned  = zeros(1,4);
state(1:9,i) = [rates'; angles';zeros(3,1)];

%-------------------------------------------------------------------------%       
% ADCS CLASS INITIALIZATION   
roll_ctrl  = R_Control(thruster, sim_complete, I(1), g0, dt, dim);
elevation_ctrl ... 
= PY_Control(pitch, theta_max, tau, I(4), g0, dt, thruster, dim, 0);
heading_ctrl ...
= PY_Control(yaw, psi_max, tau, I(6), g0, dt, thruster, dim,0);
ecr ... 
= PY_Control(pitch, theta_max, tau, I(4), g0, dt, thruster, dim, reorient);
hcr ...
= PY_Control(yaw, psi_max, tau,I(6), g0, dt, thruster, dim,reorient);

%-------------------------------------------------------------------------%       
% SLOSH CLASS INITIALIZATION
P_Slosh = Slosh_Dynamics(Rtank,htank,bot_tank,rho,damp,zeta,Nroots,2,dim);
Y_Slosh = Slosh_Dynamics(Rtank,htank,bot_tank,rho,damp,zeta,Nroots,3,dim);

%-------------------------------------------------------------------------%
% INPUT FILE INITIALIZATION
ARfile = fopen(Data_File.state_vars,'w');
fprintf(ARfile, '%0s %11s %14s %14s %18s %16s %12s %15s %16s %12s\n', ...
    'time' , 'p', 'q', 'r', 'phi_I', 'theta_I', 'psi_I', 'phi_bf',    ...
    'theta_bf','psi_bf' );
fprintf(ARfile, ...
    '%12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f\n', ...
    time(i), state(:,i)');

%-------------------------------------------------------------------------% 
% SIMULATION LOOP                                                          
loop = true;
N    = total_time/dt;
mass_calculator = zeros(length(sim_complete.sequence),1);

while loop == true

    % maneuver sequence    
    [ roll_ctrl, elevation_ctrl, heading_ctrl, ecr, hcr, mm_thrust,     ...
      M(i,:), M_ax(i,:), mass_calculator, state, fuel_burned]           ...
      = event_sequence(roll_ctrl, elevation_ctrl, heading_ctrl, ecr,    ...
      hcr, sim_complete, state, thruster, M(:,:), mm_thrust, dt, pitch, ...
      yaw, g0, i, mass_calculator, fuel_burned);
        
    % activation history of thrusters and total mass attenuation 
    firing_state = [roll_ctrl.fired(i,:),elevation_ctrl.fired(i,:), ...
		heading_ctrl.fired(i,:),mm_thrust.fired(i,1)]; 
    mass_attenuation = roll_ctrl.mburn(i)+elevation_ctrl.mburn(i) ...
		+heading_ctrl.mburn(i)+mm_thrust.mburn(i,1);
    ADCS_mass_burned(i,1) = roll_ctrl.mburn(i)+elevation_ctrl.mburn(i) ...
		+heading_ctrl.mburn(i)+ecr.mburn(i)+hcr.mburn(i);
    
    % Equivalent Mechanical Slosh Model
    x_dot = xdot(state(:,i),M(i,:),I);
    [P_Slosh, Y_Slosh, M_sl(i,:), M_off(i,:), F_sl(i,:)]                ...
        = slosh_model(P_Slosh, Y_Slosh, state, mass(i,:), g, g0, dt,    ...
          CG,  mm_thrust, i);
    
    % Total moment due to ADCS thrusters, main motor, and slosh disturbances
    M_total(i,:) = M(i,:)+M_sl(i,:)+M_off(i,:);    

    % Propagate state vector
    state(1:6,i+1) = RK4(state(1:6,i), M_total(i,:), I, dt);
    state(7:9,i+1) = inertial_2_bodyfixed_angles(state(4:6,i+1),state(4:6,1));
    
    % Check if any state variable has blown up
    eval_state = state(:,i+1) > 10^7;
    i = i+1; 
    
    % Update CG location 
    CG = UpdateCG(CG, dCG, firing_state); 
         
    % Update thruster position vectors 
    [roll_ctrl, elevation_ctrl, heading_ctrl, mm_thrust] =      ...
        UpdatePosition(thruster, dCG, firing_state, roll_ctrl,  ...
        elevation_ctrl, heading_ctrl, mm_thrust); 
     
    % Update mass of spacecraft
    mass(i,:) = UpdateMass(mass(i-1,:), mass_attenuation,OF);   
     
    % Update inertia tensor 
    I = UpdateInertias(I, I0, mass(i,1), mass(1,1), dCG, firing_state); 
               
    % Update time step 
    time(i) = time(i-1)+dt; 
    
    % write data to file
    fprintf(ARfile, ...
        '%12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f\n', ...
        time(i), state(:,i)');
    
    % Evaluate simulation success criteria
    if sim_end == 1
        if sim_complete.complete == true
            loop = false;
            success = true;
            disp('Simulation successfully ran.');
        elseif any(eval_state) == true || i==dim-5
            fuel_burned = [0,0,0,0];
            success = 0;
            disp('Simulation blew up.');
            loop = false;
        end
    else
        if time(i) >= total_time
            loop = false;
        end
    end
end 
 
%-------------------------------------------------------------------------% 
% TIME-HISTORY RESULTS
subplot_Results(state(:,1:i), time(1:i));

%-------------------------------------------------------------------------% 
% SIMULATION RESULTS
Total_fuel_consumption = sum(fuel_burned);
disp(' ');
disp('--------------------------------------- ');
disp(['Total Fuel Consumption: ', num2str(Total_fuel_consumption),' lbm']);
disp('--------------------------------------- ');

%-------------------------------------------------------------------------% 
% CLOSE DATA FILES
fclose(ARfile);

%-------------------------------------------------------------------------%
% END OF SIMULATION
    
    
    