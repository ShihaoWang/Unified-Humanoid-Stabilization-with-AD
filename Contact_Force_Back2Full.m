function lamda_Full = Contact_Force_Back2Full(lamda_i, Active_Ind)

% This function is used to transform the contact force from the minimal
% case into a full case

lamda_Full = zeros(12,1);

for i = 1:12
    for j = 1:length(Active_Ind)
        Active_Ind_j = Active_Ind(j);
        if i == Active_Ind_j
            lamda_Full(i) = lamda_i(j);
        end
        
    end
end
end

