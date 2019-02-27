%=========================================================================% 
% inertial_2_body_fixed_angles.m converts the inertial Euler angles to an %
% equivalent body-fixed angle.                                            %
%=========================================================================%

function bf = inertial_2_bodyfixed_angles(curr,init)
    ph  = curr(1); th  = curr(2); ps  = curr(3);
    ph0 = init(1); th0 = init(2); ps0 = init(3);
    
    bf(1,1) = ph-ph0;
    bf(2,1) = cos(ph)*(th-th0)+cos(th0)*sin(ph)*(ps-ps0);
    bf(3,1) = -sin(ph)*(th-th0)+cos(th0)*cos(ph)*(ps-ps0);
end

