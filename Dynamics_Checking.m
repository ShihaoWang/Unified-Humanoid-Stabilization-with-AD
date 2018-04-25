function Dynamics_Checking()

% This function checks the dynamics of the derivative system

load('Pre_Load_Structure.mat');

rIx = 0;
rIy = 1.5;
theta = 0.21;
q1 = 0.733;
q2 = 1.5;
q3 = 0.733;
q4 = -1.5;
q5 = 2;
q6 = -1.3;
q7 = 1;
q8 = -2;
q9 = -pi/2;
q10 = -1.5;

tspan = linspace(0,1,101);
init_state = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10]'; 
init_statedot = 0 * init_state;
init0 = [init_state; init_statedot];
options = odeset('AbsTol',1e-10, 'RelTol',1e-10, 'Events',@Touch_Down);
[t,z] = ode45(@Dynamics_Checking_Obj, tspan, init0, options, P);
 
end

function zdot = Dynamics_Checking_Obj(t,z,P)

rIx = z(1);         rIy = z(2);         theta = z(3);
q1 = z(4);          q2 = z(5);          q3 = z(6);
q4 = z(7);          q5 = z(8);          q6 = z(9);
q7 = z(10);         q8 = z(11);         q9 = z(12);
q10 = z(13); 

rIxdot = z(1+13);          rIydot = z(2+13);          thetadot = z(3+13);
q1dot = z(4+13);           q2dot = z(5+13);           q3dot = z(6+13);
q4dot = z(7+13);           q5dot = z(8+13);           q6dot = z(9+13);
q7dot = z(10+13);          q8dot = z(11+13);          q9dot = z(12+13);
q10dot = z(13+13); 

xdot = [rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]'; 

D_q_fn = P.D_q_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
B_q_fn = P.B_q_fn();
C_q_qdot_fn = P.C_q_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
u = zeros(10,1);
xddot = D_q_fn\(-C_q_qdot_fn + B_q_fn * u);
zdot = [xdot; xddot]; 
end

function [value,isterminal,direction] = Touch_Down(t,z, P)

rIx = z(1);         rIy = z(2);         theta = z(3);
q1 = z(4);          q2 = z(5);          q3 = z(6);
q4 = z(7);          q5 = z(8);          q6 = z(9);
q7 = z(10);         q8 = z(11);         q9 = z(12);
q10 = z(13);

rA = P.rA_fn(q1,q2,q3,rIx,rIy,theta);
rB = P.rB_fn(q1,q2,q3,rIx,rIy,theta);
rC = P.rC_fn(q4,q5,q6,rIx,rIy,theta);
rD = P.rD_fn(q4,q5,q6,rIx,rIy,theta);
rE = P.rE_fn(q7,q8,rIx,rIy,theta);
rF = P.rF_fn(q9,q10,rIx,rIy,theta);
rG = P.rG_fn(q1,q2,rIx,rIy,theta);
rH = P.rH_fn(q1,rIx,rIy,theta);
rI = P.rI_fn(rIx,rIy);
rJ = P.rJ_fn(q4,q5,rIx,rIy,theta);
rK = P.rK_fn(q4,rIx,rIy,theta);
rL = P.rL_fn(rIx,rIy,theta);
rM = P.rM_fn(q7,rIx,rIy,theta);
rN = P.rN_fn(q9,rIx,rIy,theta);
rT = P.rT_fn(rIx,rIy,theta);
value = [];
value = [value; rA(2)];
value = [value; rB(2)];
value = [value; rC(2)];
value = [value; rD(2)];
value = [value; rE(2)];
value = [value; rF(2)];
value = [value; rG(2)];
value = [value; rH(2)];
value = [value; rI(2)];
value = [value; rJ(2)];
value = [value; rK(2)];
value = [value; rL(2)];
value = [value; rM(2)];
value = [value; rN(2)];
value = [value; rT(2)];

isterminal = ones(15,1);
direction = zeros(15,1);

end

function Energy_Checking(z_array, P)
[m,~] = size(z_array);
Mech_Engy = [];
Mech_Engy_Hor = [];
T_fn = P.T_fn;
V_fn = P.V_fn;

for i = 1:m
    z = z_array(i,:);
    
    rIx = z(1);         rIy = z(2);         theta = z(3);
    q1 = z(4);          q2 = z(5);          q3 = z(6);
    q4 = z(7);          q5 = z(8);          q6 = z(9);
    q7 = z(10);         q8 = z(11);         q9 = z(12);
    q10 = z(13);
    
    rIxdot = z(1+13);          rIydot = z(2+13);          thetadot = z(3+13);
    q1dot = z(4+13);           q2dot = z(5+13);           q3dot = z(6+13);
    q4dot = z(7+13);           q5dot = z(8+13);           q6dot = z(9+13);
    q7dot = z(10+13);          q8dot = z(11+13);          q9dot = z(12+13);
    q10dot = z(13+13);
    
    T_i = T_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);
    V_i = V_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,rIy,theta);
    
    Mech_Engy_i = T_i + V_i;
    Mech_Engy = [Mech_Engy; Mech_Engy_i];
    Mech_Engy_Hor = [Mech_Engy_Hor; i];
    
end

plot(Mech_Engy_Hor, Mech_Engy);
end