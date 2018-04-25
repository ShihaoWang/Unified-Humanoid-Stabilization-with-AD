function [Opt_Var_CF_LowBd, Opt_Var_CF_UppBd] = Contact_Force_Bd_Fn(sigma_i)

% This function is used to compute the contact force bound

if sigma_i(1) == 0  
    % foot AB not in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  0 0 0 0];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd  0 0 0 0];  
end
if sigma_i(1) == 1
    % foot AB in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  -inf 0 -inf 0];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd  inf inf inf inf];
end

if sigma_i(2) == 0  
     % foot CD not in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  0 0 0 0];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd  0 0 0 0];    
    
end

if sigma_i(2) == 1
    % foot AB in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  -inf 0  -inf 0];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd   inf inf inf inf];
end

if sigma_i(3) == 0
    % foot AB not in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  0 0 0 0];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd  0 0 0 0];
    
end
if sigma_i(3) == 1
    % foot AB not in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  -inf -inf -inf -inf];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd  0 inf 0 inf];
end

if sigma_i(4) == 0
    % foot AB not in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  0 0 0 0];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd  0 0 0 0];
    
end
if sigma_i(4) == 1
    % foot AB not in contact
    Opt_Var_CF_LowBd = [Opt_Var_CF_LowBd  -inf -inf -inf -inf];
    Opt_Var_CF_UppBd = [Opt_Var_CF_UppBd  0 inf 0 inf];
end

Opt_Var_CF_LowBd = Opt_Var_CF_LowBd';
Opt_Var_CF_UppBd = Opt_Var_CF_UppBd';

end
