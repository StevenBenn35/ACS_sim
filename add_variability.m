function structure = add_variability(structure)
    
    avg = [structure.Pc structure.z structure.wn structure.isp];
    unc = [structure.U_Pc structure.U_z structure.U_wn structure.U_isp];
    
    for i=1:length(avg)

        new_vars(i)=avg(i)+(-unc(i)+(2*unc(i))*rand);

    end
    
    structure.Pc=new_vars(1);
    structure.z=new_vars(2);
    structure.wn=new_vars(3);
    structure.isp=new_vars(4);
end


