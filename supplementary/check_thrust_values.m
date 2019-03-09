clc; clear variables; close all;

%% IMPUT PARAMETERS
pulse=1.0; %sec
Favg=linspace(30.123-0.93,30.123+0.93,100); %N
wn=31.12;
z=1.7025;
t=0;
T0_ss=3224.1;
De=0.4216e-02;
Ru=8314.4126;
MW=24.9382;
g=1.12851;
ER=9.438;
mdot=0.01111;
P_inf=15.54e+03;
th=8.5*pi/180;


%% CALCULATED PARAMETERS
M_sup=ER_to_M(ER,1.05,g);
M_sub=ER_to_M(ER,0.1,g);
Ae=(pi/4)*De^2;
Te_ss=T0_ss/(1+0.5*(g-1)*M_sup^2);
Rg=Ru/MW;
lam=0.5*(1+cos(th));
As=Ae/ER;

%% FIND SS-CHAMBER PRESSURE
for i=1:length(Favg)
    
    Pe_ss(i)=(Favg(i)-lam*mdot*M_sup*sqrt(g*Rg*Te_ss))/Ae+P_inf;
    P0_ss(i)=Pe_ss(i)*(1+0.5*(g-1)*M_sup^2)^(g/(g-1));
    
end

figure;
plot(Favg,Pe_ss./10^3,'.k');
xlabel('F')
ylabel('P_e_x_i_t');

figure;
plot(Favg,P0_ss./10^3,'.k');
xlabel('F');
ylabel('P_e_x_i_t');




