function Simu_Main()
% This is the main program to run the whole simulation
global Tme_Seed Ctrl_No mini mu Node_i Node_i_child Active_Ind_Init Active_Ind_Tran Active_Ind_Goal 
Tme_Seed = 2;       % The default time within each segment
Ctrl_No = 15;       % Control grids within each segment
mini = 0.05;        % An position offset
mu = 0.5;

%% First is to validate the initial condition
[sigma0, x0] = Default_Init('show');
% [sigma0, x0] = Default_Init();
KE0 = Kinetic_Energy_Cal(x0);

% Root node initialization
Node.mode = sigma0;
Node.robotstate = x0;
Node.KE = KE0;
Node.Par_Node = [];
[Node_i_Pos, Node_i_Vel]= End_Effector_Pos_Vel(x0);
Node.End_Pos = Node_i_Pos;
Node.End_Vel = Node_i_Vel;

G.Node = {Node};
G.Edge = [];
R = [{Node}];

while isempty(R)==0
    [Node_i, R, Ind_i] = Frontier_Node_Pop(R);
    [Flag_i, Var_Opt, Var_Value] = Node_Self_Opt(Node_i);
    if (Flag_i ==1)&&(Var_Value<=0.01)
        break;
    end
    
    [Flag, Var_Opt] = Nodes_Connectivity_Opt(Node_i);
end

sigma_children = Node_Expansion_Fn(Node0);

end