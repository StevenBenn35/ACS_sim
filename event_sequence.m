function [ roll_ctrl, elevation_ctrl, heading_ctrl, ecr, hcr, ax_thrust, M, M_ax,calc_mass,state, output]               ...
            = event_sequence(roll_ctrl, elevation_ctrl, heading_ctrl, ecr, hcr, ...
              sim_complete,state,thruster,M,ax_thrust, ...
              dt,pitch,yaw,g0,i,calc_mass,output)
      
    sequence = sim_complete.sequence;
    event    = sim_complete.event;
    
    % previous moment calculation
    if i == 1
        M_prev = [0,0,0];
        state_prev = zeros(length(state),1);
    else
        M_prev = M(i-1,:);
        state_prev = state(:,i-1);
    end
    
    angle_diff = (state(7:9,i)-state_prev(7:9,1))./dt;
    
    % Maneuver #1: Coast
    if sequence(event) == 1
        M = [0,0,0];
        sim_complete.coast_timer = sim_complete.coast_timer+dt;
        completed = sim_complete.eval_complete(1,1);
        
        M_ax = M;
        
    % Maneuver #2: Axial Thrust only
    elseif sequence(event) == 2 
        M = ax_thrust.activate(i);
        ax_thrust = ax_thrust.calc_mburn(g0,i);
        completed = false;
                        
    % Maneuver #3: Spin-up w/Nutation Damping + Axial Thrust
    elseif sequence(event) == 3 
        spin_up = true;
        [roll_ctrl,M_r] = roll_ctrl.controller_MAIN(spin_up,i);
        M_ax = ax_thrust.activate(i);
        [elevation_ctrl, Mp] = ... 
            elevation_ctrl.controller_MAIN(state(8,i),state(2,i),thruster,M_prev,pitch,i);
        [heading_ctrl,My] =  ...
            heading_ctrl.controller_MAIN(state(9,i),state(3,i),thruster,M_prev,yaw,i);
        M = M_r+M_ax+Mp+My;
        ax_thrust = ax_thrust.calc_mburn(g0,i);
        completed = sim_complete.eval_complete(3,state(1,i));
        
        calc_mass(event,1) = calc_mass(event,1)+roll_ctrl.mburn(i)+elevation_ctrl.mburn(i)+heading_ctrl.mburn(i);
        
    % Maneuver #4: Spin-down w/Nutation Damping + Axial Thrust
    elseif sequence(event)== 4
        M_ax = ax_thrust.activate(i);
        ax_thrust = ax_thrust.calc_mburn(g0,i);
        
        spin_up = false;
        [roll_ctrl,M_r] = roll_ctrl.controller_MAIN(spin_up,i);
        completed = sim_complete.eval_complete(4,state(1,i));
        
%         if ~isequal(sign(state(1,i-1)),sign(state(1,i)))
%             completed = true;
%         end
        
        [elevation_ctrl, Mp] = ... 
            elevation_ctrl.controller_MAIN(state(8,i),state(2,i),thruster,M_prev,pitch,i);
        [heading_ctrl,My] =  ...
            heading_ctrl.controller_MAIN(state(9,i),state(3,i),thruster,M_prev,yaw,i);
        
        if completed == true
            state(1,i) = 0;
        end
        
        M=M_r+M_ax+Mp+My;
        
        calc_mass(event,1) = calc_mass(event,1)+roll_ctrl.mburn(i)+elevation_ctrl.mburn(i)+heading_ctrl.mburn(i);
        
    % Maneuver #5: Nutation Damping Controller + Axial Thrust 
    elseif sequence(event) == 5
        M_ax = ax_thrust.activate(i);
        ax_thrust = ax_thrust.calc_mburn(g0,i);
        
        [elevation_ctrl, Mp] = ... 
            elevation_ctrl.controller_MAIN(state(8,i),state(2,i),thruster,M_prev,pitch,i);
        [heading_ctrl,My] =  ...
            heading_ctrl.controller_MAIN(state(9,i),state(3,i),thruster,M_prev,yaw,i);
        
        M = M_ax+Mp+My;
        
        if angle_diff(2,1) <= elevation_ctrl.rate_MAX && angle_diff(3,1) <= heading_ctrl.rate_MAX
            sim_complete.nutdamp_timer = sim_complete.nutdamp_timer+dt;
        else
            sim_complete.nutdamp_timer = 0;
        end
                
        completed = sim_complete.eval_complete(5);
        
        calc_mass(event,1) = calc_mass(event,1)+elevation_ctrl.mburn(i)+heading_ctrl.mburn(i);

    % Maneuver #6: Reorientation + Axial Thrust
    elseif sequence(event) == 6 
        M_ax = ax_thrust.activate(i);
        ax_thrust = ax_thrust.calc_mburn(g0,i);
        
        [ecr, Mp] = ... 
            ecr.controller_MAIN(state(8,i),state(2,i),thruster,M_prev,pitch,i);
        [hcr,My] =  ...
            hcr.controller_MAIN(state(9,i),state(3,i),thruster,M_prev,yaw,i);
        
        M = Mp+My+M_ax;
        
%         if angle_diff(2,1) <= ecr.rate_MAX && angle_diff(3,1) <= hcr.rate_MAX
%             sim_complete.reorient_timer = sim_complete.reorient_timer+dt;
%         else
%             sim_complete.reorient_timer = 0;
%         end
        sim_complete.reorient_timer = sim_complete.reorient_timer+dt;
        completed = sim_complete.eval_complete(6,i);
%         disp(angle_diff(2:3,1));
        
        calc_mass(event,1) = calc_mass(event,1)+ecr.mburn(i)+hcr.mburn(i);
                        
    % flag error for any other numerical input
    else
        error('Invalid maneuver inputted');
    end
    
    % change maneuvers once one has completed
    if completed == true
        if sequence(event) == 1
            string = 'Coast ';
        elseif sequence(event) == 2
            string = 'Axial thrust ';
        elseif sequence(event) == 3
            string = 'Spin up ';
        elseif sequence(event) == 4
            string = 'Spin down ';
        elseif sequence(event) == 5
            string = 'Nutation damping ';
        elseif sequence(event) == 6
            string = 'Reorientation ';
        end
        
        output(event) = calc_mass(event,1);
%         disp([string,'maneuver propellant used: ', num2str(calc_mass(event,1)),' lbm']);
        sim_complete.complete(event) = true;
        sim_complete.event = sim_complete.event+1;
        
        % zero out all timers
        sim_complete.coast_timer = 0;
        sim_complete.nutdamp_timer = 0;
        sim_complete.reorient_timer = 0;
    end
    
end

