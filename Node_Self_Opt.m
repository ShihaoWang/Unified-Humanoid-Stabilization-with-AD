function [Flag, Var_Opt] = Node_Self_Opt(Node)
global Ctrl_No mini Node_i Node_i_child Active_Ind_Init Active_Ind_Tran Active_Ind_Goal
% This function optimizes the inertia shaping strategy within a certain mode

% The main idea to minimize the kinetic energy

[Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd] = Seed_Guess_Gene(Node);

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

auxdata.Ctrl_No = Ctrl_No;                  auxdata.mini = mini;
auxdata.Node_i = Node_i;                    auxdata.Node_i_child = Node_i_child;
auxdata.Active_Ind_Init = Active_Ind_Init;  auxdata.Active_Ind_Tran = Active_Ind_Tran;  auxdata.Active_Ind_Goal = Active_Ind_Goal;
auxdata.sigma_i = sigma_i;                  auxdata.sigma_i_child = sigma_i_child;
auxdata.sigma_tran = sigma_tran;            auxdata.sigma_goal = sigma_goal;

% setup.order = 1;
% setup.numvar = length(Opt_Seed);
% setup.objective  = 'Nodes_Connectivity_Obj';
% setup.constraint = 'Nodes_Connectivity_Constraint';
% setup.auxdata = auxdata;
% 
% adifuncs = adigatorGenFiles4Fmincon(setup);
% Nodes_Connectivity_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','MaxIterations',inf,'MaxFunctionEvaluations',inf);
% 
% [x0,fval0] = fmincon(@Nodes_Connectivity_Obj,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
%                      @Nodes_Connectivity_Constraint,Nodes_Connectivity_Opt);

tic;
options = optimset('Algorithm','sqp');
options = optimset(options,'GradObj','on','GradConstr','on','Display','iter');
options = optimset(options,'MaxFunEvals',inf,'MaxIter',inf);
[x1,fval1] = fmincon(@Nodes_Connectivity_Obj_Grd,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
                     @Nodes_Connectivity_Constraint_Jac,options,auxdata);
time1 = toc;

end