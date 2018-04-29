function [Post_Impact_State, Impulse] = Impact_Mapping_Cal(Pre_Impact_State, Active_Ind)

% This function is used to calculate the robot state after the impulse

rIx = Pre_Impact_State(1);                rIy = Pre_Impact_State(2);                theta = Pre_Impact_State(3);
q1 = Pre_Impact_State(4);                 q2 = Pre_Impact_State(5);                 q3 = Pre_Impact_State(6);
q4 = Pre_Impact_State(7);                 q5 = Pre_Impact_State(8);                 q6 = Pre_Impact_State(9);
q7 = Pre_Impact_State(10);                q8 = Pre_Impact_State(11);                q9 = Pre_Impact_State(12);
q10 = Pre_Impact_State(13);

rIxdot = Pre_Impact_State(1+13);          rIydot = Pre_Impact_State(2+13);          thetadot = Pre_Impact_State(3+13);
q1dot = Pre_Impact_State(4+13);           q2dot = Pre_Impact_State(5+13);           q3dot = Pre_Impact_State(6+13);
q4dot = Pre_Impact_State(7+13);           q5dot = Pre_Impact_State(8+13);           q6dot = Pre_Impact_State(9+13);
q7dot = Pre_Impact_State(10+13);          q8dot = Pre_Impact_State(11+13);          q9dot = Pre_Impact_State(12+13);
q10dot = Pre_Impact_State(13+13);

xstate = Pre_Impact_State(1:13,:);
xstatedot = Pre_Impact_State(14:26,:);

Pre_Impact_KE = Kinetic_Energy_Cal(Pre_Impact_State);

D_q = D_q_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
% B_q = B_q_fn(1);
% C_q_qdot = C_q_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,...
%     q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);

Jac_Act = Jac_Full(Active_Ind,:);

Impulse_Active = -(Jac_Act * (D_q\Jac_Act'))\(Jac_Act * xstatedot);

Post_Impact_State = xstatedot + D_q\(Jac_Act' * Impulse_Active);

Post_Impact_State = [xstate; Post_Impact_State];

Post_Impact_KE = Kinetic_Energy_Cal(Post_Impact_State);

if Pre_Impact_KE>=Post_Impact_KE
    Impulse = Contact_Force_Back2Full(Impulse_Active, Active_Ind);
else
    Post_Impact_State = Pre_Impact_State;
    Impulse = 0*Contact_Force_Back2Full(Impulse_Active, Active_Ind);
end
end

