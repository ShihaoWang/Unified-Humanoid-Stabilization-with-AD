function Jac_Ind = Jac_Ind_Fn(Act_Holo_Ind)
% This function is used to select the candidata active row
Jac_Ind = [];
Nonzero_Ind = find(Act_Holo_Ind);
for i = 1:length(Nonzero_Ind)
    Jac_Ind_i = [2*Nonzero_Ind(i)-1, 2*Nonzero_Ind(i)];
    Jac_Ind = [Jac_Ind Jac_Ind_i];
end
end