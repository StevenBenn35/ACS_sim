%=========================================================================%
% 'subplot_Results.m' creates a 2 - 1 x 3  subplot images of the stage's  %
% position and rate versus time.                                          %
%                                                                         %
% INPUT:                                                                  % 
% state vector history (state), time history vector (time)                %
%                                                                         %
% OUTPUT:                                                                 %      
% Subplots                                                                %   
%=========================================================================%

function subplot_Results(state, time)
   % set size of figure window
    fig_handle = figure; 
    set(fig_handle,'Position',[900, 450, 900, 450]);
    
    omega = state(1:3,:);
    euler = state(4:6,:);
    bf_angles = state(7:9,:);    
    
    % 2x3 subplot of phase plane, disturbance vs time, and rate vs time
    subplot(321);    
    plot(time,euler(1,:)*180/pi,'k');
%     plot(time,bf_angles(1,:)*180/pi,'k');
    title('\bf{Bank Angle vs. Time}','interpreter','latex');
    xlabel('\bf{time, sec}', 'interpreter','latex');
    ylabel('\bf{$$ \phi$$, deg}', 'interpreter','latex');

    subplot(323);
    plot(time, euler(2,:)*180/pi,'k');%,'LineWidth',2);
%     plot(time, bf_angles(2,:)*180/pi,'k');%,'LineWidth',2);
    title('\bf{Elevation Angle vs. Time}','interpreter','latex');
    xlabel('\bf{time}, sec', 'interpreter','latex');
    ylabel('\bf{$$ \theta$$, deg}', 'interpreter','latex');

    subplot(325);
    plot(time, euler(3,:)*180/pi,'k');%'LineWidth',2);
%     plot(time, bf_angles(3,:)*180/pi,'k');%'LineWidth',2);
    title('\bf{Azimuth Angle vs. Time}','interpreter','latex');
    xlabel('\bf{time}, sec', 'interpreter','latex');
    ylabel('\bf{$$ \psi$$}, deg', 'interpreter','latex');    

    subplot(322);
    plot(time,omega(1,:)*180/pi,'k');
    title('\bf{Roll Rate vs. Time}','interpreter','latex');
    xlabel('\bf{time, sec}', 'interpreter','latex');
    ylabel('\bf{$$ p $$ , deg/sec}', 'interpreter','latex');
    
    subplot(324);
    plot(time, omega(2,:)*180/pi,'k');%,'LineWidth',2);
    title('\bf{Pitch Rate vs. Time}','interpreter','latex');
    xlabel('\bf{time}, sec', 'interpreter','latex');
    ylabel('\bf{$$ q $$, deg/sec}', 'interpreter','latex');

    subplot(326);
    plot(time, omega(3,:)*180/pi,'k');%,'LineWidth',2);
    title('\bf{Yaw Rate vs. Time}','interpreter','latex');
    xlabel('\bf{time}, sec', 'interpreter','latex');
    ylabel('\bf{$$ r $$, deg/sec}', 'interpreter','latex');
end