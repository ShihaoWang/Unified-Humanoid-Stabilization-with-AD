function [c,ceq] = Init_Cons(z0)

global sigma0 mini mu

c = []; ceq = [];

% z0(3) = 0;
%% 1. Distance constraint
[End_Pos, End_Vel]= End_Effector_Pos_Vel(z0);

[Pos_Dist, Node_i_child_Norm_Ang] = Obs_Dist_Fn(End_Pos);

End_Vel = reshape(End_Vel',16,1);

Eqn_Pos_Matrix = blkdiag(sigma0(1),sigma0(1), sigma0(2),sigma0(2),...
    sigma0(3),sigma0(4),0,0);

Eqn_Vel_Matrix = blkdiag(sigma0(1),sigma0(1),sigma0(1),sigma0(1),...
    sigma0(2),sigma0(2),sigma0(2),sigma0(2),...
    sigma0(3),sigma0(3),sigma0(4),sigma0(4),0,0,0,0);
ceq = [ceq; Eqn_Pos_Matrix * Pos_Dist];
ceq = [ceq; Eqn_Vel_Matrix * End_Vel];


Inq_Pos_Matrix = blkdiag(not(sigma0(1)),not(sigma0(1)), not(sigma0(2)),not(sigma0(2)),...
    not(sigma0(3)),not(sigma0(4)),0,0);

c = [c; - (Inq_Pos_Matrix * Pos_Dist - Inq_Pos_Matrix * ones(8,1) * mini)];

%% 2. Complementarity constraints: Contact Force!!!

rIx_i = z0(1);                rIy_i = z0(2);                theta_i = z0(3);
q1_i = z0(4);                 q2_i = z0(5);                 q3_i = z0(6);
q4_i = z0(7);                 q5_i = z0(8);                 q6_i = z0(9);
q7_i = z0(10);                q8_i = z0(11);                q9_i = z0(12);
q10_i = z0(13);

rIxdot_i = z0(1+13);          rIydot_i = z0(2+13);          thetadot_i = z0(3+13);
q1dot_i = z0(4+13);           q2dot_i = z0(5+13);           q3dot_i = z0(6+13);
q4dot_i = z0(7+13);           q5dot_i = z0(8+13);           q6dot_i = z0(9+13);
q7dot_i = z0(10+13);          q8dot_i = z0(11+13);          q9dot_i = z0(12+13);
q10dot_i = z0(13+13);

D_q = D_q_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,theta_i);
B_q = B_q_fn(1);
C_q_qdot = C_q_qdot_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,...
    q10dot_i,q1dot_i,q2dot_i,q3dot_i,q4dot_i,q5dot_i,q6dot_i,q7dot_i,q8dot_i,q9dot_i,thetadot_i,theta_i);
Jac_Full = Jac_Full_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,theta_i);
Jacdot_qdot = Jacdot_qdot_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,...
    q10dot_i,q1dot_i,q2dot_i,q3dot_i,q4dot_i,q5dot_i,q6dot_i,q7dot_i,q8dot_i,q9dot_i,thetadot_i,theta_i);
Active_In = Active_In_Cal_fn(Jac_Full, sigma0);

Jac_Act = Jac_Full(Active_In,:);
Jacdot_qdot_Act = Jacdot_qdot(Active_In,:);

lamda_i = (Jac_Act *(D_q\Jac_Act'))\(Jac_Act * (D_q\C_q_qdot) - Jacdot_qdot_Act);

lamda_Full_i = Contact_Force_Back2Full(lamda_i, Active_In);

[Normal_Force, Tang_Force] = Contact_Force_Cal_fn(Node_i_child_Norm_Ang(1:end-2,:), lamda_Full_i);
c = [c; -Normal_Force];

%% 3. Friction cone constraints:
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

end