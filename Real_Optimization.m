function [Flag, Var_Opt, fval] = Real_Optimization(Opt_Seed, Opt_Lowbd, Opt_Uppbd, flag_ref)
global Ctrl_No mini Node_i Node_i_child Active_Ind_Init Active_Ind_Tran Active_Ind_Goal sigma_i sigma_i_child sigma_tran sigma_goal time_count
global flag
flag = flag_ref;

Obj_Fun = @Nodes_Connectivity_Obj;    

tic
fval = [];
Nodes_Connectivity_Init_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','interior-point',...
    'MaxIterations',inf,'MaxFunctionEvaluations',inf,'OutputFcn','Time_Termination_fn');

% Nodes_Connectivity_Init_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','interior-point',...
%     'MaxIterations',inf,'MaxFunctionEvaluations',inf);

Nodes_Connectivity_Fur_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp',...
    'MaxIterations',inf,'MaxFunctionEvaluations',inf,'OutputFcn','Time_Termination_Fur_fn');

% Nodes_Connectivity_Fur_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp',...
%     'MaxIterations',inf,'MaxFunctionEvaluations',inf);
%% Interior-Point first
% time_count = clock;

[Var_Opt,fval,exitflag,output] = fmincon(Obj_Fun,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
    @Nodes_Connectivity_Constraint,Nodes_Connectivity_Fur_Opt);
if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.01)
    Flag =1;
    return
end

%% SQP second
% time_count = clock;
[Var_Opt,fval,exitflag,output] = fmincon(Obj_Fun,Var_Opt,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
    @Nodes_Connectivity_Constraint,Nodes_Connectivity_Fur_Opt);
Flag = 0;
if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.001)
    Flag =1;
    return
else
    if exitflag ==-1
%         time_count = clock;
        [Var_Opt,fval,exitflag,output] = fmincon(Obj_Fun,Var_Opt,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
            @Nodes_Connectivity_Constraint,Nodes_Connectivity_Fur_Opt);
        if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.1)
            Flag =1;
            return
        end
    end
end
toc
end

