%=========================================================================% 
% Slosh_Dynamics.m is a class that organizes all data necessary for the   %
% mechanical equivalent slosh model. The slosh forces and moments are     %
% calculated here.                                                        %
%=========================================================================%

classdef Slosh_Dynamics < handle
    
    properties
        R         % radius of tank, ft
        H         % height of tank, ft        
        bot       % tank bottom wrt nozzle origin, in
        full      % fraction of tank fill        
        X         % fluid translation vector of n-modes
        Z
        M_modes
        rho       % fluid density, slug/ft^3
        d         % damping factor        
        zeta      % roots of DJ1
        Nroots    % numbe of roots        
        m0        % mass of rigid fluid, lbm 
        h0        % height of rigid fluid below tank CG, ft
        I0        % rigid moment of inertia, lbm*ft^2
        mn        % mass of nth mode, lbm
        hn        % height of nth mode from tank CG, ft
        wn        % natural frequency of nth mode, rad/s             
        axis      % axis specifier 2 == pitch, 3 == yaw
    end
    
    methods
        function obj = Slosh_Dynamics(rtank,htank,bot_xyz,density,...
                damp,zta,roots,axis_type,dim)
            
            % initialize tank and fluid parameters 
            obj.R      = rtank;
            obj.H      = htank;
            obj.bot    = bot_xyz;                        
            obj.d      = damp;
            obj.rho    = density;
            obj.zeta   = zta; 
            obj.Nroots = roots;           
            
            % initialize modal parameters
            obj.X  = zeros(roots,3);
            obj.Z  = zeros(2*roots,1);
            obj.mn = zeros(roots,1);
            obj.hn = zeros(roots,1);
            obj.wn = zeros(roots,1);
            obj.M_modes = zeros(roots+1,dim);
            
            % initialize pitch or yaw axis specifier
            obj.axis = axis_type;
        end
        
        function [obj,Msl,Moff,Fsl] = Slosh_MAIN(obj,angles,rate, ...
                m,g,dt,CGstage,i)
            
            % obtain angles for specified axis
            a = obj.axis;
            if i==1
                angle = [0,0,angles(a,1)];
            elseif i==2
                angle = [0,angles(a,1),angles(a,2)];
            else
                angle = [angles(a,1),angles(a,2),angles(a,3)];
            end           
            
            % calculate fluid height in tank
            [obj,h,CGtank] = tank_parameters(obj,m,CGstage);

            % obtain modal parameters (mn, hn, wn, mo, ho)
            obj = modal_parameters(obj,m,h,g);

            % calculate inertias (Isolid, Ifluid, Irigid)
            obj = inertia_parameters(obj,m,h);

            % calculate slosh forces and moment
            [obj,Msl,Fsl]=slosh_forces_and_moments(obj,angle,rate,g,dt,i);
            
            % calculate moment due to tank CG offset from stage CG
            Moff = Slosh_Dynamics.calc_moment(CGtank,Fsl);            

        end
        
        function [obj,h,CGtank] = tank_parameters(obj,m,CGstage)
            htank = obj.H;
            
            % current fluid height
            m    = m/32.174;      % lbm to slug
            V    = m/obj.rho;     % volume, ft^3
            h    = V/(pi*obj.R^2); % undisturbed fluid height, ft
            hliq = [h,0,0];
            
            % fraction of tank fill
            obj.full = h/htank; % full tank ratio
            
            % tank CG wrt stage CG
            CGtank = obj.bot+(0.5*12).*hliq-CGstage; 
        end
        
        function obj = modal_parameters(obj,m,h,g)
            r = obj.R;
            z = obj.zeta;
            N = obj.Nroots;
            Mn = obj.mn;
            Hn = obj.hn;
            Wn = obj.wn;
            r_mn_mF = zeros(1,N);
            
            % modal masses, heights, and natural frequencies
            sum_mh = 0; h01 = 0; h02 = 0;
            for i = 1:N
                zhr = z(i)*h/r;
                zh2r = 0.5*zhr;
                r_mn_mF(i) = 2*(tanh(zhr)/zhr)/(z(i)^2-1);
                h01 = h01+tanh(zhr)/zh2r/(z(i)^2-1);
                h02 = h02+(z(i)*tanh(zhr)+4*(r/h/cosh(zhr)))/(z(i)^2*(z(i)^2-1)*h/r);

                % modal natural frequencies
                Wn(i)=sqrt((g*z(i)/r)*tanh(z(i)*h/r));

                % modal masses
                Mn(i) = m*r_mn_mF(i);

                % modal mass heights
                Hn(i) = 0.5*h*(1-2*tanh(zh2r)/zh2r);

                sum_mh = sum_mh+Mn(i)*Hn(i);        
            end
            
            obj.wn = Wn;
            obj.mn = Mn;
            obj.hn = Hn;

            % static mass and height
            obj.m0 = m-sum(obj.mn);
            obj.h0 = -sum_mh/obj.m0;
            
            % check EMM conditions
            h00 = obj.h0;
            h0_check = h*(0.5*(h/r)^(-2)-h02)/(1-h01);
          
        end
        
        function obj = inertia_parameters(obj,m,h)
            r = obj.R;
            z = obj.zeta;
            N = obj.Nroots;
            Mn = obj.mn;
            Hn = obj.hn;
            
            % summing parameters for Is, IF, and I0 calculations
            sum1 = 0; sum2 = 0;
            for i = 1:N
                znhr = z(i)*h/2/r;
                sum1 = sum1+4*r^2*m*(1-tanh(znhr)/znhr)/(z(i)*(z(i)^2-1));
                sum2 = sum2+(Mn(i)*Hn(i)^2);
                In(i,1)=Mn(i)*Hn(i)^2;
                CM(i,1)=Mn(i)*Hn(i);
                mm(i,1)=Mn(i);
            end

            % Equivalent "Solid" moment of inertia
            I_S = m*(h^2/12+r^2/4);

            % Fluid moment of inertia
            I_F = I_S-sum1;

            % Rigid moment of inertia
            obj.I0 = I_F-obj.m0*obj.h0^2-sum2; 
            
            % Check EMM conditions
            IF_check = obj.I0+obj.m0*obj.h0^2+sum(In);
            CM_check = obj.m0*obj.h0-sum(CM);
            m_check = obj.m0+sum(mm);
            
        end
        
        function [obj,Msl,Fsl]=slosh_forces_and_moments(obj,angle,rate,g,dt,k)
            N = obj.Nroots;
            dd = obj.d;
            Wn = obj.wn;
            Hn = obj.hn;
            Mn = obj.mn;
            
            A = zeros(N,N);
            B = A;
            C = zeros(N,1);
            D = C;
            
            a = obj.axis;
            rate = rate(a);
            
            if k>=2
                Xk  = obj.X(:,2);
                Xkm = obj.X(:,1);
            end

            % calculate matrices A,B,C,D
            for i=1:N
                den = 1+2*dd*(0.5*dt*Wn(i))+(0.5*dt*Wn(i))^2;

                A(i,i)=2*(1-(0.5*dt*Wn(i))^2)/den;
                B(i,i)=(1-2*dd*(0.5*dt*Wn(i))+(0.5*dt*Wn(i))^2)/den;
                C(i,1)=((0.5*dt)^2*g-Hn(i))/den;
                D(i,1)=2*((0.5*dt)^2*g+Hn(i))/den;
            end

            % calculate modal vectors
            if k==1
                Tk=angle(3);

                Tkm1=Tk-rate*dt;
                Tkp1=Tk+rate*dt;

                Xk=C*(Tkp1+Tkm1)+D*Tk;
                Xkp=A*Xk+C*(Tkp1+Tkm1)+D*Tk;
            else
                Tk=angle(3);
                Tkm1=angle(2);
                Tkm2=angle(1);
                Tkp1=Tk+rate*dt;
                Xkp=A*Xk-B*Xkm+C*(Tkp1+Tkm1)+D*Tk;
            end
            
            % disturbance moment due to slosh (for k>=2)
            if k >= 3
                mh2=0; mhx=0; mx=0;mxddot=0; mhddot=0;
                ddot = (Tk-2*Tkm1+Tkm2)/dt^2;

                for i=1:N
                    Xddot = (Xkp(i,1)-2*Xk(i,1)+Xkm(i,1))/dt^2;
                    mh2=mh2+Mn(i)*Hn(i)^2;
                    mhx=mhx+Mn(i)*Hn(i)*Xddot;
                    mx = mx+Mn(i)*Xk(i,1);
                    mxddot=mxddot+Mn(i)*Xddot;
                    mhddot=mhddot+Mn(i)*Hn(i)*ddot;
                end
                
                for i=1:N
                    Xddot = (Xkp(i,1)-2*Xk(i,1)+Xkm(i,1))/dt^2;
                    dM(i,1) = Mn(i)*Hn(i)^2*ddot+Mn(i)*Hn(i)*Xddot-g*Mn(i)*Xk(i,1);
                    dM(i,1) = dM(i,1)*12^2;
                end
                
                M0 = (obj.I0+obj.m0*obj.h0^2)*ddot;
                M0 = M0*12^2;
                obj.M_modes(:,k)= [M0;dM(:,1)];
                
                Msl=(obj.I0+obj.m0*obj.h0^2+mh2)*ddot+mhx-g*mx; %lbm*ft^2/s^2
                Msl=Msl*12^2; %lbm*in^2/s^2
                
                Fsl=obj.m0*obj.h0*ddot-mxddot+mhddot; %lbm*ft/s^2
                Fsl=Fsl/32.174; %lbf
            else
                Fsl=0;
                Msl=0;
            end

            if obj.axis ==2 %pitch
                Fsl=[0,0,Fsl];
                Msl=[0,Msl,0];
            elseif obj.axis == 3 %yaw
                Fsl=[0,Fsl,0];
                Msl=[0,0,Msl];
            end

            % populate modal vector history
            obj.X(:,1)=Xk;
            obj.X(:,2)=Xkp;            
        end
    end % end of methods
    
    methods(Static)
        function [M] = calc_moment(r,f)
            rx = r(1); ry = r(2); rz = r(3);
            fx = f(1); fy = f(2); fz = f(3);

            if length(r) == length(f)

                Mx = ry*fz-rz*fy; %lbf-in
                My = -(rx*fz-rz*fx);
                Mz = rx*fy-ry*fx;

                M = [Mx,My,Mz]'.*(32.174*12); %lbf-in to lbm*in^2/s^2

            else 
                error('Position and Force vectors must be the same length');
            end 
        end        
    end % end of static methods    
end % end of class

