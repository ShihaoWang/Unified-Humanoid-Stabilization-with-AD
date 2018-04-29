function [Flag, Var_Opt] = Nodes_Connectivity_Opt(Node_i, Node_i_child)

global Ctrl_No mini Active_Ind_Init Active_Ind_Tran Active_Ind_Goal sigma_i sigma_i_child sigma_tran sigma_goal time_count
% This function optimizes the inertia shaping strategy within a certain mode
Flag = [];Var_Opt = [];
% The main idea to minimize the kinetic energy
[Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd] = Seed_Guess_Gene(Node_i, Node_i_child);
if Flag ==0
    return
end 

sigma_i = Node_i.mode;
sigma_i_child = Node_i_child.mode;
sigma_i_change = sigma_i_child - sigma_i;
if max(sigma_i_change)==1
    % In this case, it is making contact
    sigma_tran = sigma_i;
    sigma_goal = sigma_i_child;
else
    % In this case, it is retracting contact
    sigma_tran = sigma_i_child;
    sigma_goal = sigma_i_child;    
end

[Flag, Var_Opt, fval] = Real_Optimization(Opt_Seed, Opt_Lowbd, Opt_Uppbd);

end