function [c, ceq] = Seed_Conf_Constraint(z)
c = []; ceq = [];
global Node_i Node_i_child mini

[Node_i_child_Pos, Node_i_child_Vel]= End_Effector_Pos_Vel(z);
Node_i_child_Pos_Dist = Obs_Dist_Fn(Node_i_child_Pos);

% According to the way that sigma_child is generated, this offset vector
% can at most have one nonzero value

sigma_i = Node_i.mode;
sigma_i_child = Node_i_child.mode;

%% 1. Relative distance constraints: a. all distances have to be at least on the surface; b. the desired mode has to be satisfied.
Node_i_child_Vel = reshape(Node_i_child_Vel',16,1);

Eqn_Pos_Matrix = blkdiag(sigma_i_child(1),sigma_i_child(1), sigma_i_child(2),sigma_i_child(2),...
                         sigma_i_child(3),sigma_i_child(4),0,0);
                    
Eqn_Vel_Matrix = blkdiag(sigma_i_child(1),sigma_i_child(1),sigma_i_child(1),sigma_i_child(1),...
                         sigma_i_child(2),sigma_i_child(2),sigma_i_child(2),sigma_i_child(2),...
                         sigma_i_child(3),sigma_i_child(3),sigma_i_child(4),sigma_i_child(4),0,0,0,0);                   
ceq = [ceq; Eqn_Pos_Matrix * Node_i_child_Pos_Dist];
ceq = [ceq; Eqn_Vel_Matrix * Node_i_child_Vel];

Inq_Pos_Matrix = blkdiag(not(sigma_i_child(1)),not(sigma_i_child(1)), not(sigma_i_child(2)),not(sigma_i_child(2)),...
                         not(sigma_i_child(3)),not(sigma_i_child(4)),0,0);

c = [c; - (Inq_Pos_Matrix * Node_i_child_Pos_Dist - Inq_Pos_Matrix * ones(8,1) * mini)];
 
%% 2. Contact Constraint Maintenance: the previous contacts have to be satisfied
Node_i_State = Node_i.robotstate;
[Node_i_Pos, Node_i_Vel]= End_Effector_Pos_Vel(Node_i_State);
Eqn_Maint_Matrix = blkdiag(sigma_i(1) * sigma_i_child(1), sigma_i(1) * sigma_i_child(1),...
                           sigma_i(1) * sigma_i_child(1), sigma_i(1) * sigma_i_child(1),...
                           sigma_i(2) * sigma_i_child(2), sigma_i(2) * sigma_i_child(2),...
                           sigma_i(2) * sigma_i_child(2), sigma_i(2) * sigma_i_child(2),...
                           sigma_i(3) * sigma_i_child(3), sigma_i(3) * sigma_i_child(3),...
                           sigma_i(4) * sigma_i_child(4), sigma_i(4) * sigma_i_child(4),0,0,0,0);
 
 Node_i_Pos = reshape(Node_i_Pos',16,1);
 Node_i_child_Pos = reshape(Node_i_child_Pos',16,1);
 Node_i_Vel = reshape(Node_i_Vel',16,1);
 ceq = [ceq; Eqn_Maint_Matrix * (Node_i_Pos - Node_i_child_Pos)];
 ceq = [ceq; Eqn_Maint_Matrix * (Node_i_Vel - Node_i_child_Vel)];
end