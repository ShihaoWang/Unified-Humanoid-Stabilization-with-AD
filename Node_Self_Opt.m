function [Flag, Var_Opt] = Node_Self_Opt(Node)

% This function optimizes the inertia shaping strategy within a certain mode

% The main idea to minimize the kinetic energy

[Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd] = Seed_Guess_Gene(Node);

setup.order = 1;
setup.numvar = length(Opt_Seed);
setup.objective  = 'Nodes_Connectivity_Obj';
setup.constraint = 'Nodes_Connectivity_Constraint';
adifuncs = adigatorGenFiles4Fmincon(setup);
Nodes_Connectivity_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','MaxIterations',inf,'MaxFunctionEvaluations',inf);

% [x0,fval0] = fmincon(@Nodes_Connectivity_Obj,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
%                      @Nodes_Connectivity_Constraint,Nodes_Connectivity_Opt);

tic;
options = optimset('Algorithm','interior-point');
options = optimset(options,'GradObj','on','GradConstr','on','Display','iter');
options = optimset(options,'MaxFunEvals',inf,'MaxIter',inf);
[x1,fval1] = fmincon(@Nodes_Connectivity_Obj_Grd,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
                     @Nodes_Connectivity_Constraint_Jac,options);
time1 = toc;

end