function [Flag, Var_Opt] = Nodes_Connectivity_Opt(Node_i, Node_i_child)

global Ctrl_No mini Active_Ind_Init Active_Ind_Tran Active_Ind_Goal sigma_i sigma_i_child sigma_tran sigma_goal
% This function optimizes the inertia shaping strategy within a certain mode

% The main idea to minimize the kinetic energy
[Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd] = Seed_Guess_Gene(Node);
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

% setup.order = 1;
% setup.numvar = length(Opt_Seed);
% setup.objective  = 'Nodes_Connectivity_Obj';
% setup.constraint = 'Nodes_Connectivity_Constraint';
% 
% adifuncs = adigatorGenFiles4Fmincon(setup);
tic
Nodes_Connectivity_Init_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','interior-point',...
    'MaxIterations',inf,'MaxFunctionEvaluations',inf,'StepTolerance',1e-4);


[Var_Opt,fval,exitflag] = fmincon(@Nodes_Connectivity_Obj,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
                  @Nodes_Connectivity_Constraint,Nodes_Connectivity_Init_Opt);
Flag = 0;              
if (exitflag==1)||(exitflag==2)
    Flag =1;
    Var_Value = fval;
end
toc

end