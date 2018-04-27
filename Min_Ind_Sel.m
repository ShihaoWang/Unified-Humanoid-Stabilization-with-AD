function [Pos_Dist_i,Env_Ind_vec] = Min_Ind_Sel(Pos_Dist_temp)

Pos_Dist_i = Pos_Dist_temp(1);
Env_Ind_i = 1;
for i = 1:length(Pos_Dist_temp)-1
    Pos_Dist_i_l = Pos_Dist_temp(i+1);
    if Pos_Dist_i>Pos_Dist_i_l
        Pos_Dist_i = Pos_Dist_i_l;
        Env_Ind_i = 1 + i;
    end
end
Env_Ind_vec = zeros(length(Pos_Dist_temp),1);

for i = 1:length(Pos_Dist_temp)
    if i == Env_Ind_i
        Env_Ind_vec(i) = 1;       
    end  
end
end
