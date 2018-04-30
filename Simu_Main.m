function Simu_Main()
% This is the main program to run the whole simulation
global Tme_Seed Ctrl_No mini mu Node_i Node_i_child Active_Ind_Init Active_Ind_Tran Active_Ind_Goal time_count
Tme_Seed = 2;       % The default time within each segment
Ctrl_No = 15;       % Control grids within each segment
mini = 0.05;        % An position offset
mu = 0.5;

%% First is to validate the initial condition
[sigma0, x0] = Default_Init('show');
% [sigma0, x0] = Default_Init();

% Root node initialization
Node.Par_Node = [];
Node.Potentil_Children_Nodes = [];
Node.Feasible_Children_Nodes = [];
Node = Node_Sub(Node, sigma0, x0);

Total_Nodes = [{Node}];
G.Node = [{Node}];  % This structure will save the index of the nodes that are actually in the tree
G.Edge = [];
R = [{Node}];

while isempty(R)==0
    [Node_i, R] = Frontier_Node_Pop(R);
    [Flag_i, Var_Opt, Var_Value] = Node_Self_Opt(Node_i);
    if (Flag_i ==1)&&(Var_Value<=0.01)
        Node_i.Self_StateNControl = Var_Opt;
        break;
    end
    if Flag_i == 1
        % This means that the robot's kinetic energy is not reduced to 0
        % but the value is small
        Node_i.Self_StateNControl = Var_Opt;
        Node_i.robotstate = Edge_State_Distill(Var_Opt);
    end
    sigma_children_modes = Node_Expansion_Fn(Node_i);
    for i = 1:length(sigma_children_modes)
        Node_i_child.mode = sigma_children_modes(i,:);
        Node_i.Potentil_Children_Nodes = [Node_i.Potentil_Children_Nodes; {Node_i_child}];
        [Flag, node_i_child_StateNCtrl, node_i_child_Edge_State,Impulse] = Nodes_Connectivity_Opt(Node_i, Node_i_child);
        if Flag == 1
            % In this case, these two nodes can be connected so
            Node_i_child = Node_Sub(Node_i_child,  Node_i_child.mode, node_i_child_Edge_State);
            Node_i_child.Par_StateNControl = node_i_child_StateNCtrl;
            Node_i_child.Par_Node = Node_i;
            Node_i_child.Impulse = Impulse;
            Node_i.Feasible_Children_Nodes = [ Node_i.Feasible_Children_Nodes; {Node_i_child}];
            G.Node = [G.Node; {Node_i_child}];
            R = [R; {Node_i_child}];
        end
    end
end
end