function [Flag, Var_Opt] = Nodes_Connectivity_Opt(sigma_i, x_i, sigma_i_child, P)

% This function test the connectivity between a certain node and its child
% node

% The current idea is to make sure of the multiple shooting method since in
% this case the dynamics constraints are satisfied automatically

% The main idea to reach the sigma_child at the end step while minimizing the kinetic energy

P.sigma_i = sigma_i;            P.x_i = x_i;            P.sigma_i_child = sigma_i_child; 
P.Ctrl_No = 20;                 P.Tme_Seed = 2;

[Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd, P] = Seed_Guess_Gene(sigma_i, x_i, sigma_i_child, P);

Nodes_Connectivity_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','MaxIterations',inf,'MaxFunctionEvaluations',inf);
Obj_Constraint_Grad_Cal(Opt_Seed, P)
Var_Opt = fmincon(@Nodes_Connectivity_Obj,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,@Nodes_Connectivity_Constraint,Nodes_Connectivity_Opt, P);

end