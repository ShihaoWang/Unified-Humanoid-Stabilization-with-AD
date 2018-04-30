function [Flag, Var_Opt, fval] = Real_Optimization(Opt_Seed, Opt_Lowbd, Opt_Uppbd, flag)
global Ctrl_No mini Node_i Node_i_child Active_Ind_Init Active_Ind_Tran Active_Ind_Goal sigma_i sigma_i_child sigma_tran sigma_goal time_count

if flag == 0
    Obj_Fun = @Nodes_Connectivity_Obj;
else
    Obj_Fun = @Nodes_Connectivity_Obj_Min;    
end

tic
time_count = clock;
fval = [];
Nodes_Connectivity_Init_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','interior-point',...
    'MaxIterations',inf,'MaxFunctionEvaluations',inf,'OutputFcn','Time_Termination_fn');

[Var_Opt,fval,exitflag,output] = fmincon(Obj_Fun,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
    @Nodes_Connectivity_Constraint,Nodes_Connectivity_Init_Opt);
if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.1)
    Flag =1;
    return
end

time_count = clock;

Nodes_Connectivity_Fur_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp',...
    'MaxIterations',inf,'MaxFunctionEvaluations',inf,'OutputFcn','Time_Termination_Fur_fn');

time_count = clock;
[Var_Opt,fval,exitflag,output] = fmincon(Obj_Fun,Var_Opt,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
    @Nodes_Connectivity_Constraint,Nodes_Connectivity_Fur_Opt);
Flag = 0;
if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.1)
    Flag =1;
    return
else
    if exitflag ==-1
        if (output.constrviolation<1)
            time_count = clock;            
            [Var_Opt, fval, exitflag, output] = fmincon(Obj_Fun,Var_Opt,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
                @Nodes_Connectivity_Constraint,Nodes_Connectivity_Init_Opt);
            if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.5)
                Flag =1;
                return
            else
                if (output.constrviolation<1)
                    time_count = clock;
                    [Var_Opt,fval,exitflag,output] = fmincon(Obj_Fun,Var_Opt,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
                        @Nodes_Connectivity_Constraint,Nodes_Connectivity_Fur_Opt); 
                    if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.5)
                        Flag =1;
                        return                   
                    end
                else
                    Flag = 0;                  
                end
            end
        else
            time_count = clock;
            [Var_Opt,fval,exitflag,output] = fmincon(Obj_Fun,Var_Opt,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
                @Nodes_Connectivity_Constraint,Nodes_Connectivity_Fur_Opt);
            if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.5)
                Flag =1;
                return
            else
                if (output.constrviolation<1)
                    time_count = clock;
                    [Var_Opt, fval, exitflag, output] = fmincon(Obj_Fun,Var_Opt,[],[],[],[],Opt_Lowbd,Opt_Uppbd,...
                        @Nodes_Connectivity_Constraint,Nodes_Connectivity_Fur_Opt);
                    if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.5)
                        Flag =1;
                        return                        
                    end
                else
                    Flag = 0;                   
                end
            end
        end
    end
end
toc
end

