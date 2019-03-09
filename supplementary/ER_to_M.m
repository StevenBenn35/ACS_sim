%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ER_to_Mach implements Netwon's Method to determine the exit Mach number %
% for a given expansion ratio.                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function M = ER_to_M(ER,Mg,ga)
    % Helper variables
    A=(1-3*ga)/(2-2*ga);
    B=2/(ga+1);
    C=0.5*(ga-1);
    D=(ga+1)/(2*(ga-1));
    E=ga-1;
    G=ga+1;
    
    % Set loop parameters
    e=1.0;  %error
    loop=true;

    while loop==true
        ERg = (1/Mg)*(B*(1+C*Mg*Mg))^D;
        e=abs((ER-ERg)/ER);

        if e < 1e-04
            loop = false;
            M=Mg;
        else
            F=ERg-ER;
            dF=(2^A)*((Mg*Mg-1)/(Mg*Mg*(2+Mg*Mg*E)))*((1+Mg*Mg*C)/G)^D;
            Mg=Mg-F/dF;
        end
    end    
end

