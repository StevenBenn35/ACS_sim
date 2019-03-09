clc; clear variables; close all;

Avg=[ 1.7025 31.12 188.706 274.78 ];
Unc=[0.1382 0.858 4.21953 1.48111]; 

for i=1:length(Avg)
    
    var_value(i)=Avg(i)+(-Unc(i)+(2*Unc(i))*rand);
    
end

var_value
