function Quasi_Static_Veri()

% THis function is used to find the quasi static state of the robot

global sigma0
sigma0 = [ 1 1 0 0 ]; 
rIx = 1;             rIy = 1;            theta = 0.35;
q1 = 0.65;           q2 = 0.0001;        q3 = -0.3;
q4 = -0.45;          q5 = 0.5;           q6 = 0.27;
q7 = -0.66;          q8 = -0.6251;       q9 = 0.69;         q10 = -0.2951;

x0state = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10]';
u0 = ones(10,1);

init0 = [x0state; u0];

Quasi_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','Maxiterations',inf,'MaxFunctionEvaluations',inf);
[RobotState_LowBd, RobotState_UppBd, Ctrl_LowBd, Ctrl_UppBd] = Optimization_Bounds();
Quasi_LowBd = [RobotState_LowBd(:,1:13),Ctrl_LowBd']';
Quasi_UppBd = [RobotState_UppBd(:,1:13),Ctrl_UppBd']';


[x0,fval0] = fmincon(@Quasi_Obj,init0,[],[],[],[],Quasi_LowBd,Quasi_UppBd,@Quasi_Cons,Quasi_Opt);

z0 = x0(1:13,:);
z0 = [z0; zeros(13,1)];

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

end

