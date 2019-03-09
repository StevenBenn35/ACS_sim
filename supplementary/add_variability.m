function new_vars = add_variability(avg,unc)

    for i=1:length(avg)

        new_vars(i)=avg(i)+(-unc(i)+(2*unc(i))*rand);

    end
    
end

