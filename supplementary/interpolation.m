function T = interpolation(time,Thrust,targett)

if targett ==0
    T=0;
else
    i=1;
   while targett>time(i)
       i=i+1;
   end

   x=((targett-time(i-1))/(time(i)-time(i-1)));
   T = (Thrust(i-1)+(Thrust(i)-Thrust(i-1))*x);
end