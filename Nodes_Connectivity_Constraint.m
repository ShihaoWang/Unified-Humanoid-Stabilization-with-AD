function [c,ceq] = Nodes_Connectivity_Constraint(z)
global Ctrl_No mini mu Node_i Node_i_child Active_Ind_Tran Active_Ind_Goal sigma_i sigma_i_child sigma_tran sigma_goal flag
c = []; ceq = [];

%% Optimization variables unzip: delta_t, StateNdot_tot, Ctrl_tot, ContactForce_tot
delta_t = z(1)/Ctrl_No;    
stateNdotNCtrl_ref = z(2:end);
StateNdot_tot = stateNdotNCtrl_ref(1:13*2*(Ctrl_No - 1),:);
Ctrl_tot = stateNdotNCtrl_ref(1+length(StateNdot_tot):end,:);
StateNdot_tot = reshape(StateNdot_tot, 26, (Ctrl_No - 1));
StateNdot_tot = [Node_i.robotstate StateNdot_tot];
Ctrl_tot = reshape(Ctrl_tot, 10, (Ctrl_No - 1));
Ctrl_tot = [zeros(10,1) Ctrl_tot];

sigma_base = [sigma_i' sigma_tran' sigma_goal'];

stateNdot_vec = zeros((Ctrl_No - 1),1);

ratio = 0.25;

for i = 1:Ctrl_No-1
   
    Ctrl_i = Ctrl_tot(:,i+1);
    
    %% The robot state at k
    x0 = StateNdot_tot(:,i); 
    x0_state = x0(1:13,:);
    x0_statedot = x0(14:end,:);
    
    %% The robot state at k+1
    x0p1 = StateNdot_tot(:,i+1);
    rIxp1 = x0p1(1);            rIyp1 = x0p1(2);            thetap1 = x0p1(3);
    q1p1 = x0p1(4);             q2p1 = x0p1(5);             q3p1 = x0p1(6);
    q4p1 = x0p1(7);             q5p1 = x0p1(8);             q6p1 = x0p1(9);
    q7p1 = x0p1(10);            q8p1 = x0p1(11);            q9p1 = x0p1(12);
    q10p1 = x0p1(13);
    rIxdotp1 = x0p1(1+13);          rIydotp1 = x0p1(2+13);          thetadotp1 = x0p1(3+13);
    q1dotp1 = x0p1(4+13);           q2dotp1 = x0p1(5+13);           q3dotp1 = x0p1(6+13);
    q4dotp1 = x0p1(7+13);           q5dotp1 = x0p1(8+13);           q6dotp1 = x0p1(9+13);
    q7dotp1 = x0p1(10+13);          q8dotp1 = x0p1(11+13);          q9dotp1 = x0p1(12+13);
    q10dotp1 = x0p1(13+13);
    x0p1_state = x0p1(1:13,:);
    x0p1_statedot = x0p1(14:end,:);
    
    ceq = [ceq; x0p1_statedot * delta_t + x0_state -  x0p1_state];    
    
    qddot = (x0p1_statedot - x0_statedot)/delta_t ;
    
    ceq = [ceq; qddot];
    D_q = D_q_fn(q1p1,q2p1,q3p1,q4p1,q5p1,q6p1,q7p1,q8p1,q9p1,q10p1,thetap1);
    B_q = B_q_fn();
    C_q_qdot = C_q_qdot_fn(q1p1,q2p1,q3p1,q4p1,q5p1,q6p1,q7p1,q8p1,q9p1,q10p1,q10dotp1,q1dotp1,q2dotp1,q3dotp1,q4dotp1,q5dotp1,q6dotp1,q7dotp1,q8dotp1,q9dotp1,thetadotp1,thetap1);    
    Jac_Full = Jac_Full_fn(q1p1,q2p1,q3p1,q4p1,q5p1,q6p1,q7p1,q8p1,q9p1,q10p1,thetap1);
    Jacdot_qdot = Jacdot_qdot_fn(q1p1,q2p1,q3p1,q4p1,q5p1,q6p1,q7p1,q8p1,q9p1,q10p1,q10dotp1,q1dotp1,q2dotp1,q3dotp1,q4dotp1,q5dotp1,q6dotp1,q7dotp1,q8dotp1,q9dotp1,thetadotp1,thetap1);  
    
    Active_In = Active_Ind_Tran;
    if i == Ctrl_No-1
        Active_In = Active_Ind_Goal;
    end
    Jac_Act = Jac_Full(Active_In,:);
    Jacdot_qdot_Act = Jacdot_qdot(Active_In,:);
    
    lamda_i = (Jac_Act *(D_q\Jac_Act'))\(Jac_Act * (D_q\C_q_qdot) - Jacdot_qdot_Act - Jac_Act * (D_q\(B_q * Ctrl_i)));

    %% 1. Dynamics constraints
    Dyn_Via = D_q * qddot + C_q_qdot - Jac_Act' * lamda_i - B_q * Ctrl_i;
    ceq = [ceq; Dyn_Via];
    
    %% 2. Complementarity constraints: Distance!!!---Node_i_child mode constraints
    [Node_i_child_Pos, Node_i_child_Vel] = End_Effector_Pos_Vel(x0p1);
    [Node_i_child_Pos_Dist, Node_i_child_Norm_Ang]= Obs_Dist_Fn(Node_i_child_Pos);   
    sigma_vec_sel = [0 1 0]';    
    if i == Ctrl_No-1
        sigma_vec_sel = [ 0 0 1]';
    end  
    sigma_t = sigma_base * sigma_vec_sel;   
    if sigma_t(1)==1
        Not_Sigma_t_1 = 0;
    else
        Not_Sigma_t_1 = 1;
    end 
    if sigma_t(2)==1
        Not_Sigma_t_2 = 0;
    else
        Not_Sigma_t_2 = 1;
    end
    if sigma_t(3)==1
        Not_Sigma_t_3 = 0;
    else
        Not_Sigma_t_3 = 1;     
    end
    if sigma_t(4)==1
        Not_Sigma_t_4 = 0;
    else
        Not_Sigma_t_4 = 1;
    end
    Inq_Pos_Matrix = zeros(8,8);
    Inq_Pos_Matrix(1,1) = Not_Sigma_t_1;        Inq_Pos_Matrix(2,2) = Not_Sigma_t_1; 
    Inq_Pos_Matrix(3,3) = Not_Sigma_t_2;        Inq_Pos_Matrix(4,4) = Not_Sigma_t_2; 
    Inq_Pos_Matrix(5,5) = Not_Sigma_t_3;        Inq_Pos_Matrix(6,6) = Not_Sigma_t_4; 

    c = [c; - (Inq_Pos_Matrix * Node_i_child_Pos_Dist - Inq_Pos_Matrix * ones(8,1) * 0.1*mini)];
    
    if i == Ctrl_No-1
        End_Eq_Pos_Matrix = zeros(8,8);
        End_Eq_Pos_Matrix(1,1) = sigma_t(1);        End_Eq_Pos_Matrix(2,2) = sigma_t(1);
        End_Eq_Pos_Matrix(3,3) = sigma_t(2);        End_Eq_Pos_Matrix(4,4) = sigma_t(2);
        End_Eq_Pos_Matrix(5,5) = sigma_t(3);        End_Eq_Pos_Matrix(6,6) = sigma_t(4);   
        ceq = [ceq; End_Eq_Pos_Matrix * Node_i_child_Pos_Dist];     
    end

    %% 3. Complementarity constraints: Contact Force!!!    
    lamda_Full_i = Contact_Force_Back2Full(lamda_i, Active_In);

    [Normal_Force, Tang_Force] = Contact_Force_Cal_fn(Node_i_child_Norm_Ang(1:end-2,:), lamda_Full_i);
    c = [c; -Normal_Force];
    
    %% 4. Contact Constraint Maintenance: the previous contacts have to be maintained
    Node_i_Pos = Node_i.End_Pos;
    Eqn_Maint_Matrix = zeros(16,16);
    Eqn_Maint_Matrix(1,1) = sigma_i(1) * sigma_i_child(1);          Eqn_Maint_Matrix(2,2) = sigma_i(1) * sigma_i_child(1);
    Eqn_Maint_Matrix(3,3) = sigma_i(1) * sigma_i_child(1);          Eqn_Maint_Matrix(4,4) = sigma_i(1) * sigma_i_child(1);
    Eqn_Maint_Matrix(5,5) = sigma_i(2) * sigma_i_child(2);          Eqn_Maint_Matrix(6,6) = sigma_i(2) * sigma_i_child(2);
    Eqn_Maint_Matrix(7,7) = sigma_i(2) * sigma_i_child(2);          Eqn_Maint_Matrix(8,8) = sigma_i(2) * sigma_i_child(2);
    Eqn_Maint_Matrix(9,9) = sigma_i(3) * sigma_i_child(3);          Eqn_Maint_Matrix(10,10) = sigma_i(3) * sigma_i_child(3);
    Eqn_Maint_Matrix(11,11) = sigma_i(4) * sigma_i_child(4);        Eqn_Maint_Matrix(12,12) = sigma_i(4) * sigma_i_child(4);

    Node_i_Pos = reshape(Node_i_Pos',16,1);
    Node_i_child_Pos = reshape(Node_i_child_Pos',16,1);
    Contact_Constraint_Maintenance_All = Eqn_Maint_Matrix * (Node_i_Pos - Node_i_child_Pos);
    ceq = [ceq; Contact_Constraint_Maintenance_All(Active_Ind_Tran)];
    
    %% 5. Friction cone constraints:
    Tang_Force_1 = Tang_Force(1) + Tang_Force(2);
    Norm_Force_1 = Normal_Force(1) + Normal_Force(2);
    Tang_Force_2 = Tang_Force(3) + Tang_Force(4);
    Norm_Force_2 = Normal_Force(3) + Normal_Force(4);
    Tang_Force_3 = Tang_Force(5);
    Norm_Force_3 = Normal_Force(5);
    Tang_Force_4 = Tang_Force(6);
    Norm_Force_4 = Normal_Force(6);
    
    c = [c;  Tang_Force_1 * Tang_Force_1 - mu * mu * Norm_Force_1 * Norm_Force_1];    
    c = [c;  Tang_Force_2 * Tang_Force_2 - mu * mu * Norm_Force_2 * Norm_Force_2];    
    c = [c;  Tang_Force_3 * Tang_Force_3 - mu * mu * Norm_Force_3 * Norm_Force_3];    
    c = [c;  Tang_Force_4 * Tang_Force_4 - mu * mu * Norm_Force_4 * Norm_Force_4];    
    
    %% 6. Kinetic Energy reduction 
    if i == Ctrl_No-1
        if flag ==1
            c = [c; Kinetic_Energy_Cal(x0p1) - 0.1];
        else
            c = [c; Kinetic_Energy_Cal(x0p1) - ratio * Kinetic_Energy_Cal(Node_i.robotstate)];
        end
    end
end

end
