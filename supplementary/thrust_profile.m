clc; clear variables; close all;

%% INPUT PARAMETERS

% P0_ss=1301082.07;
P0_ss=480000;
% U_P0_ss=29092.63524;
U_P0_ss=10732.963;
T0_ss=3224.1;
g=1.12851;
MW=24.9382;
P_inf=15.54e+03;

Dt=0.4216e-02;
ER=9.438;
th=8.5*pi/180;

Ru=8314.4126;

%% PULSE PARAMETERS
% wn=31.12;
wn = 500;
% U_wn=0.858;
U_wn=13.785;
% z=1.7025;
z=1.25;
% U_z=0.1382;
U_z=0.10147;

dt=.001;
t=0;

i=1;
CF=100;

spec_time=1.3;

%% CALCULATED PARAMETERS
M_sup=ER_to_M(ER,1.05,g);
M_sub=ER_to_M(ER,0.1,g);

Te_ss=T0_ss/(1+0.5*(g-1)*M_sup^2);

At=(pi/4)*Dt^2;
Ae=At*ER;
lam=0.5*(1+cos(th));

Rg=Ru/MW;

%% PRESSURE/THRUST TRAIL-OFF (OVER-DAMPED CASE)

% Apply variability to average parameters
new_vars=add_variability([P0_ss,wn,z],[U_P0_ss,U_wn,U_z]);
P0_ss=new_vars(1);
wn=new_vars(2);
z=new_vars(3);

while CF > 0
    
    P0(i)=P_inf+(P0_ss-P_inf)*(exp(-z*wn*t(i))*(z*sinh(sqrt(z^2-1)*wn*t(i))/sqrt(z^2-1)+cosh(sqrt(z^2-1)*wn*t(i))));
    T0(i)=T0_ss*(P0(i)/P0_ss)^(g/(g-1));
    
    if (P0(i)/P_inf) >= ((0.5*(g+1))^(g/(g-1)))  % choked
        
        mdot(i)=Ae*P0(i)*sqrt((g/Rg)*(2/(g+1))^((g+1)/(g-1)))/sqrt(T0(i));
        Pe(i)=P0(i)/(1+0.5*(g-1)*M_sup^2)^(g/(g-1));
        CF(i)=lam*g*sqrt(2/(g-1)*(2/(g+1))^((g+1)/(g-1))*(1-(Pe(i)/P0(i))^((g-1)/g)))+ER*((Pe(i)-P_inf)/P0(i));
        F(i)=CF(i)*P0(i)*At;
        
    else % unchoked
        
        mdot(i)=At*P0(i)*sqrt((2*g/((g-1)*Rg*T0(i)))*((P_inf/P0(i))^(2/g)-(P_inf/P0(i))^((g+1)/g)));
        Pe(i)=P0(i)/(1+0.5*(g-1)*M_sub^2)^(g/(g-1));
        Te(i)=T0(i)/(1+0.5*(g-1)*M_sub^2);
        Ve(i)=M_sub*sqrt(g*Rg*Te(i));
        
        F(i)=lam*mdot(i)*Ve(i)+Ae*(Pe(i)-P_inf);
    end
    
    t(i+1)=t(i)+dt;
    i=i+1;

end

%% POST-PROCESS DATA

% square wave portion
t_square=[0 0+dt];
F_square=[0 F(1)];

% data format
F_data=[F_square F(1:end)];
t_data=[t_square,t(1:end-1)+1];
data=[t_data; F_data];
data=data(:,1:end-1);

%% OBTAIN THRUST AT ANY TIME ALONG THE CURVE

time=data(1,1):.005:data(1,end);
if spec_time > time(end)
    F_current = 0;
else
    
    for i=1:length(time)
        F_int(i)=interpolation(data(1,:),data(2,:),time(i));
    end

    j=1;
    while spec_time>time(j)
        j=j+1;
    end
    F_current=F_int(j);
    
end

F_current

%% PLOT BURN PROFILE
figure;
plot(data(1,:),data(2,:),'k');
hold on;
plot(spec_time,F_current,'.r','MarkerSize',20);
xlabel('time, s')
ylabel('Thrust, N');
xlim([-0.5 1.5]);
hold on;