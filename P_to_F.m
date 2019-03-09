function F = P_to_F( thruster_props )

    P0=thruster_props.Pc;
    P_inf=thruster_props.P_inf;
    g=thruster_props.gam;
    Ae=thruster_props.Aexit;
    Rg=thruster_props.Rg;
    T0=thruster_props.T0;
    M_sup=thruster_props.M_sup;
    M_sub=thruster_props.M_sub;
    lam=thruster_props.lam;
    ER=thruster_props.ER;
    At=thruster_props.Athroat;

    if (P0/P_inf) >= ((0.5*(g+1))^(g/(g-1)))  % choked

        Pe=P0/(1+0.5*(g-1)*M_sup^2)^(g/(g-1));
        CF=lam*g*sqrt(2/(g-1)*(2/(g+1))^((g+1)/(g-1))*(1-(Pe/P0)^((g-1)/g)))+ER*((Pe-P_inf)/P0);
        F=CF*P0*At;

    else % unchoked

        mdot = At*P0*sqrt((2*g/((g-1)*Rg*T0))*((P_inf/P0)^(2/g)-(P_inf/P0)^((g+1)/g)));
        Pe=P0/(1+0.5*(g-1)*M_sub^2)^(g/(g-1));
        Te=T0/(1+0.5*(g-1)*M_sub^2);
        Ve=M_sub*sqrt(g*Rg*Te);

        F=lam*mdot*Ve+Ae*(Pe-P_inf);
    end

end

