function Simu_Main()
% This is the main program to run the whole simulation
global Tme_Seed Ctrl_No mini mu Node_i Node_i_child Active_Ind_Tran Active_Ind_Goal time_count
Tme_Seed = 0.5;       % The default time within each segment
Ctrl_No = 15;       % Control grids within each segment
mini = 0.05;        % An position offset
mu = 0.5;

%% First is to validate the initial condition
[sigma0, x0] = Default_Init('show');
% [sigma0, x0] = Default_Init();

% x0 = [    1.2043;
%           0.6687;
%            -0.0289;
%     0.1945;
%     0.5362;
%    -0.7018;
%    -0.6740;
%     0.5191;
%     0.1939;
%    -0.1481;
%          0;
%     1.0470;
%    -0.0534;
%     0.5440;
%    -0.0851;
%     0.4058;
%     1.9368;
%    -3.0000;
%     0.6574;
%     1.5494;
%    -3.0000;
%     2.5598;
%     0.0750;
%     0.5899;
%     1.1762;
%    -2.0424];
% 
% x0 = [    4.3705;
%     0.7239;
%    -0.0084;
%     0.5884;
%     0.4474;
%     0.0084;
%    -0.3018;
%     0.5659;
%    -0.2557;
%    -0.9133;
%    -0.9085;
%     1.0310;
%    -0.3019;
%     0.4844;
%    -0.0049;
%    -0.9953;
%     2.1449;
%    -1.7763;
%    -1.9199;
%     1.6713;
%     0.1990;
%    -0.8751;
%     0.6740;
%     0.4466;
%    -0.3926;
%     2.5565];

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