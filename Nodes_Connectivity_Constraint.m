function [c,ceq] = Nodes_Connectivity_Constraint(z, auxdata)

% Auxdata unzip
Ctrl_No = auxdata.Ctrl_No;
mini = auxdata.mini;
Node_i = auxdata.Node_i;
Node_i_child = auxdata.Node_i_child;
Active_Ind_Init = auxdata.Active_Ind_Init;
Active_Ind_Tran = auxdata.Active_Ind_Tran;
Active_Ind_Goal = auxdata.Active_Ind_Goal;
sigma_i = auxdata.sigma_i;
sigma_i_child = auxdata.sigma_i_child;
sigma_tran = auxdata.sigma_tran;
sigma_goal = auxdata.sigma_goal;

c = []; ceq = [];

%% Optimization variables unzip: delta_t, StateNdot_tot, Ctrl_tot, ContactForce_tot
delta_t = z(1)/Ctrl_No;    
stateNdotNCtrl_ref = z(2:end);
StateNdot_tot = stateNdotNCtrl_ref(1:13*2*Ctrl_No,:);
Ctrl_tot = stateNdotNCtrl_ref(1+length(StateNdot_tot):end,:);
StateNdot_tot = reshape(StateNdot_tot, 26, Ctrl_No);
Ctrl_tot = reshape(Ctrl_tot, 10, Ctrl_No);

sigma_base = [sigma_i' sigma_tran' sigma_goal'];

stateNdot_vec = zeros(Ctrl_No,1);

for i = 1:Ctrl_No
    stateNdot_vec_temp = stateNdot_vec;    stateNdot_vec_temp(i) = 1;
    stateNdot_i = StateNdot_tot * stateNdot_vec_temp;      Ctrl_i = Ctrl_tot(:,i);                
    rIx_i = stateNdot_i(1);                rIy_i = stateNdot_i(2);                theta_i = stateNdot_i(3);
    q1_i = stateNdot_i(4);                 q2_i = stateNdot_i(5);                 q3_i = stateNdot_i(6);
    q4_i = stateNdot_i(7);                 q5_i = stateNdot_i(8);                 q6_i = stateNdot_i(9);
    q7_i = stateNdot_i(10);                q8_i = stateNdot_i(11);                q9_i = stateNdot_i(12);
    q10_i = stateNdot_i(13);
    
    rIxdot_i = stateNdot_i(1+13);          rIydot_i = stateNdot_i(2+13);          thetadot_i = stateNdot_i(3+13);
    q1dot_i = stateNdot_i(4+13);           q2dot_i = stateNdot_i(5+13);           q3dot_i = stateNdot_i(6+13);
    q4dot_i = stateNdot_i(7+13);           q5dot_i = stateNdot_i(8+13);           q6dot_i = stateNdot_i(9+13);
    q7dot_i = stateNdot_i(10+13);          q8dot_i = stateNdot_i(11+13);          q9dot_i = stateNdot_i(12+13);
    q10dot_i = stateNdot_i(13+13);
    %% 1. Initial condition satisfaction constraint

    if i == 1
        ceq = [ceq; stateNdot_i - Node_i.robotstate];
    end
    
    xstate_i = stateNdot_i(1:13,:);
    xstatedot_i = stateNdot_i(14:26,:);  
    if i == Ctrl_No
        x_statep1 = xstate_i;
    else
        x_statep1 = StateNdot_tot(1:13,i+1);
    end
    
    qddot = (x_statep1 - xstate_i - xstatedot_i * delta_t)/(1/2 * delta_t^2) ; 
    D_q = D_q_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,theta_i);
    B_q = B_q_fn(1);
    C_q_qdot = C_q_qdot_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,...
                           q10dot_i,q1dot_i,q2dot_i,q3dot_i,q4dot_i,q5dot_i,q6dot_i,q7dot_i,q8dot_i,q9dot_i,thetadot_i,theta_i);    
    Jac_Full = Jac_Full_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,theta_i);
    Jacdot_qdot = Jacdot_qdot_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,...
        q10dot_i,q1dot_i,q2dot_i,q3dot_i,q4dot_i,q5dot_i,q6dot_i,q7dot_i,q8dot_i,q9dot_i,thetadot_i,theta_i);
    Active_In = Active_Ind_Tran;
    if i == 1
        Active_In = Active_Ind_Init;
    end
    if i == Ctrl_No
        Active_In = Active_Ind_Goal;
    end
    Jac_Act = Jac_Full(Active_In,:);
    Jacdot_qdot_Act = Jacdot_qdot(Active_In,:);
    
    lamda_i = (Jac_Act *(D_q\Jac_Act'))\(Jac_Act * (D_q\C_q_qdot) - Jacdot_qdot_Act - Jac_Act * (D_q\(B_q * Ctrl_i)));

    %% 2. Dynamics constraints
    Dyn_Via = D_q * qddot + C_q_qdot - Jac_Act' * lamda_i - B_q * Ctrl_i;
    ceq = [ceq; Dyn_Via];
    
    %% 3. Complementarity constraints: Distance!!!---Node_i_child mode constraints
    [Node_i_child_Pos, Node_i_child_Vel] = End_Effector_Pos_Vel(stateNdot_i);
    [Node_i_child_Pos_Dist, Node_i_child_Norm_Ang]= Obs_Dist_Fn(Node_i_child_Pos);
    Node_i_child_Vel = reshape(Node_i_child_Vel',16,1);
    
    sigma_vec_sel = [0 1 0]';
    
    if i == 1
       sigma_vec_sel = [ 1 0 0]';       
    end
    
    if i == Ctrl_No
        sigma_vec_sel = [ 0 0 1]';
    end
    
    sigma_t = sigma_base * sigma_vec_sel;
   
    Eqn_Pos_Matrix = blkdiag(sigma_t(1),sigma_t(1), sigma_t(2),sigma_t(2),sigma_t(3),sigma_t(4), 0, 0);
    Eqn_Vel_Matrix = blkdiag(sigma_t(1),sigma_t(1),sigma_t(1),sigma_t(1),sigma_t(2),sigma_t(2),sigma_t(2),sigma_t(2),...
                             sigma_t(3),sigma_t(3),sigma_t(4),sigma_t(4), 0, 0, 0, 0);
    ceq = [ceq; Eqn_Pos_Matrix * Node_i_child_Pos_Dist];
    ceq = [ceq; Eqn_Vel_Matrix * Node_i_child_Vel];
    
    Inq_Pos_Matrix = blkdiag(not(sigma_t(1)),not(sigma_t(1)), not(sigma_t(2)),not(sigma_t(2)),not(sigma_t(3)),not(sigma_t(4)), 0, 0);
    
    c = [c; - (Inq_Pos_Matrix * Node_i_child_Pos_Dist - Inq_Pos_Matrix * ones(8,1) * mini)];
    
    %% 4. Complementarity constraints: Contact Force!!!    
    lamda_Full_i = Contact_Force_Back2Full(lamda_i, Active_In);

    Normal_Force = Normal_Force_Cal_fn(Node_i_child_Norm_Ang(1:end-2,:), lamda_Full_i);
    c = [c; -Normal_Force];
    
    %% 5. Contact Constraint Maintenance: the previous contacts have to be maintained
    Node_i_Pos = Node_i.End_Pos;
    Node_i_Vel = Node_i.End_Vel;
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

end
