function [Flag, Var_Opt, Edge_State, Impulse] = Nodes_Connectivity_Opt(Node_i, Node_i_child)
global Ctrl_No mini Active_Ind_Init Active_Ind_Tran Active_Ind_Goal sigma_i sigma_i_child sigma_tran sigma_goal time_count
% This function optimizes the inertia shaping strategy within a certain mode
Flag = [];Var_Opt = []; Edge_State = []; Impulse = [];
% The main idea to minimize the kinetic energy
[Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd] = Seed_Guess_Gene(Node_i, Node_i_child);
if Flag ==0
    return
end 
sigma_i = Node_i.mode;
sigma_i_child = Node_i_child.mode;
sigma_i_change = sigma_i_child - sigma_i;
Impact_Mapping_Flag = 0;
if max(sigma_i_change)==1
    % In this case, it is making contact
    sigma_tran = sigma_i;
    sigma_goal = sigma_i_child;
    Impact_Mapping_Flag = 1;
else
    % In this case, it is retracting contact
    sigma_tran = sigma_i_child;
    sigma_goal = sigma_i_child;    
end
[Flag, Var_Opt, ~] = Real_Optimization(Opt_Seed, Opt_Lowbd, Opt_Uppbd);
if Impact_Mapping_Flag==1
    % This is case there is impact mapping to be included in our discussion
    Previous_End_State = Edge_State_Distill(Var_Opt);   
    [Post_Impact_State, Impulse] = Impact_Mapping_Cal(Previous_End_State, Active_Ind_Goal);
    Edge_State = Post_Impact_State;
else
    Edge_State = Edge_State_Distill(Var_Opt);  
end
end