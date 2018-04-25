function [P, sigma0, x0_Opt] = Default_Init(varargin)

% This function is used to given a default initialization state

load('Pre_Load_Structure.mat');

P = Envi_Map_Defi(P);

sigma0 = [ 1 0 0 0 ]; 
P.sigma0 = sigma0; 
P.mini = 0.05;

rIx = 0;
rIy = 1;
theta = -0.09;
q1 = 0.45;
q2 = 0.06;
q3 = 0.15;
% q4 = -0.6;
q4 = -1;
q5 = 0.15;
q6 = 0.27;
q7 = -0.66;
q8 = -0.6251;
q9 = 0.69;
q10 = -0.2951;

rIxdot = 0.2;
rIydot = 0.1;
thetadot = -0.21;
q1dot = 0.733;
q2dot = 1.5;
q3dot = 0.733;
q4dot = -1.5;
q5dot = 2;
q6dot = -1.3;
q7dot = 1;
q8dot = -2;
q9dot = -pi/2;
q10dot = -1.5;

x0_init = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10,...
           rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot];
P.init0 = x0_init;

%% Robot state bounds
rIxlow = -Inf;                  rIxupp = Inf;
rIylow = -Inf;                  rIyupp = Inf;
thetalow = -pi;                 thetaupp = pi;
q1low = -2.1817;                q1upp = 0.733;
q2low = 0.0;                    q2upp = 2.618;
q3low = -1.3;                   q3upp = 0.733;
q4low = -2.1817;                q4upp = 0.733;
q5low = 0.0;                    q5upp = 2.618;
q6low = -1.3;                   q6upp = 0.733;
q7low = -3.14;                  q7upp = 1.047;
q8low = -2.391;                 q8upp = 0.0;
q9low = -3.14;                  q9upp = 1.047;
q10low = -2.391;                q10upp = 0.0;

P.Step_Length = P.Leg_Length * (abs(sin(q1low)) + abs(sin(q1upp)));

AngRateLow = -3.0;              AngRateHgh = 3.0;

rIxdotlow = -Inf;               rIxdotupp = Inf;
rIydotlow = -Inf;               rIydotupp = Inf;
thetadotlow = -Inf;             thetadotupp = Inf;
q1dotlow = AngRateLow;          q1dotupp = AngRateHgh;
q2dotlow = AngRateLow;          q2dotupp = AngRateHgh;
q3dotlow = AngRateLow;          q3dotupp = AngRateHgh;
q4dotlow = AngRateLow;          q4dotupp = AngRateHgh;
q5dotlow = AngRateLow;          q5dotupp = AngRateHgh;
q6dotlow = AngRateLow;          q6dotupp = AngRateHgh;
q7dotlow = AngRateLow;          q7dotupp = AngRateHgh;
q8dotlow = AngRateLow;          q8dotupp = AngRateHgh;
q9dotlow = AngRateLow;          q9dotupp = AngRateHgh;
q10dotlow = AngRateLow;         q10dotupp = AngRateHgh;

RobotState_LowBd = [rIxlow rIylow thetalow q1low q2low q3low q4low q5low q6low q7low q8low q9low q10low...
                    rIxdotlow rIydotlow thetadotlow q1dotlow q2dotlow q3dotlow q4dotlow q5dotlow q6dotlow q7dotlow q8dotlow q9dotlow q10dotlow];
RobotState_UppBd = [rIxupp rIyupp thetaupp q1upp q2upp q3upp q4upp q5upp q6upp q7upp q8upp q9upp q10upp...
                    rIxdotupp rIydotupp thetadotupp q1dotupp q2dotupp q3dotupp q4dotupp q5dotupp q6dotupp q7dotupp q8dotupp q9dotupp q10dotupp];
ContactForce_LowBd = -ones(12,1)*Inf;
ContactForce_UppBd = -ContactForce_LowBd;

P.ContactForce_LowBd = ContactForce_LowBd;
P.ContactForce_UppBd = ContactForce_UppBd;
                
Init_Opt = optimoptions(@fmincon,'Display','off','Algorithm','sqp','MaxIterations',inf);


P.RobotState_LowBd = RobotState_LowBd;
P.RobotState_UppBd = RobotState_UppBd;


%% Upper and Lower bounds of the control variables                    
tau1_max = 100;             tau2_max = 100;             tau3_max = 100;
tau4_max = 100;             tau5_max = 100;             tau6_max = 100;
tau7_max = 60;              tau8_max = 50;              tau9_max = 60;             tau10_max = 50;

Ctrl_LowBd = -[tau1_max tau2_max tau3_max tau4_max tau5_max tau6_max tau7_max tau8_max tau9_max tau10_max ]';
Ctrl_UppBd = - Ctrl_LowBd;

P.Ctrl_LowBd = Ctrl_LowBd;
P.Ctrl_UppBd = Ctrl_UppBd;

P.Step_Max = P.Leg_Length * (abs(sin(q1low)) +abs(sin(q1upp)));

x0_Sim = [rIy q1 q2 q3 q4 q5 q6 ...
          rIydot q1dot q2dot q3dot q4dot q5dot q6dot]';

RobotState_LowBd_Sim = [rIylow q1low q2low q3low q4low q5low q6low rIydotlow q1dotlow q2dotlow q3dotlow q4dotlow q5dotlow q6dotlow ];
RobotState_UppBd_Sim = [rIyupp q1upp q2upp q3upp q4upp q5upp q6upp rIydotupp q1dotupp q2dotupp q3dotupp q4dotupp q5dotupp q6dotupp ];
x_Sim_Opt= fmincon(@Init_Obj_Sim,x0_Sim,[],[],[],[],RobotState_LowBd_Sim,RobotState_UppBd_Sim,@Init_Cons_Sim,Init_Opt, P);
x0_Opt = State_Sim2All(x0_init, x_Sim_Opt);
if nargin >0 
    Single_Frame_Plot(x0_Opt, P)
end
rIx = x0_Opt(1);             rIy = x0_Opt(2);             theta = x0_Opt(3);
q1 = x0_Opt(4);              q2 = x0_Opt(5);              q3 = x0_Opt(6);
q4 = x0_Opt(7);              q5 = x0_Opt(8);              q6 = x0_Opt(9);
q7 = x0_Opt(10);             q8 = x0_Opt(11);             q9 = x0_Opt(12);
q10 = x0_Opt(13);
rIxdot = x0_Opt(1+13);          rIydot = x0_Opt(2+13);          thetadot = x0_Opt(3+13);
q1dot = x0_Opt(4+13);           q2dot = x0_Opt(5+13);           q3dot = x0_Opt(6+13);
q4dot = x0_Opt(7+13);           q5dot = x0_Opt(8+13);           q6dot = x0_Opt(9+13);
q7dot = x0_Opt(10+13);          q8dot = x0_Opt(11+13);          q9dot = x0_Opt(12+13);
q10dot = x0_Opt(13+13);

P.rIx_ref = rIx;            P.rIy_ref = rIy;            P.theta_ref = theta;
P.q1_ref = q1;              P.q2_ref = q2;              P.q3_ref = q3;
P.q4_ref = q4;              P.q5_ref = q5;              P.q6_ref = q6;
P.q7_ref = q7;              P.q8_ref = q8;              P.q9_ref = q9;          P.q10_ref = q10;
P.rIxdot_ref = rIxdot;            P.rIydot_ref = rIydot;            P.thetadot_ref = thetadot;
P.q1dot_ref = q1dot;              P.q2dot_ref = q2dot;              P.q3dot_ref = q3dot;
P.q4dot_ref = q4dot;              P.q5dot_ref = q5dot;              P.q6dot_ref = q6dot;
P.q7dot_ref = q7dot;              P.q8dot_ref = q8dot;              P.q9dot_ref = q9dot;          P.q10dot_ref = q10dot;

P.rA_ref = P.rA_fn(q1,q2,q3,rIx,rIy,theta);
P.rB_ref = P.rB_fn(q1,q2,q3,rIx,rIy,theta);
P.rC_ref = P.rC_fn(q4,q5,q6,rIx,rIy,theta);
P.rD_ref = P.rD_fn(q4,q5,q6,rIx,rIy,theta);
P.rE_ref = P.rE_fn(q7,q8,rIx,rIy,theta);
P.rF_ref = P.rF_fn(q9,q10,rIx,rIy,theta);
P.rG_ref = P.rG_fn(q1,q2,rIx,rIy,theta);
P.rH_ref = P.rH_fn(q1,rIx,rIy,theta);
P.rI_ref = P.rI_fn(rIx,rIy);
P.rJ_ref = P.rJ_fn(q4,q5,rIx,rIy,theta);
P.rK_ref = P.rK_fn(q4,rIx,rIy,theta);
P.rL_ref = P.rL_fn(rIx,rIy,theta);
P.rM_ref = P.rM_fn(q7,rIx,rIy,theta);
P.rN_ref = P.rN_fn(q9,rIx,rIy,theta);
P.rT_ref = P.rT_fn(rIx,rIy,theta);

P.vA_ref = P.vA_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
P.vB_ref = P.vB_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
P.vC_ref = P.vC_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
P.vD_ref = P.vD_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
P.vE_ref = P.vE_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
P.vF_ref = P.vF_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
P.vG_ref = P.vG_fn(q1,q2,q1dot,q2dot,rIxdot,rIydot,thetadot,theta);
P.vH_ref = P.vH_fn(q1,q1dot,rIxdot,rIydot,thetadot,theta);
P.vI_ref = P.vI_fn(rIxdot,rIydot);
P.vJ_ref = P.vJ_fn(q4,q5,q4dot,q5dot,rIxdot,rIydot,thetadot,theta);
P.vK_ref = P.vK_fn(q4,q4dot,rIxdot,rIydot,thetadot,theta);
P.vL_ref = P.vL_fn(rIxdot,rIydot,thetadot,theta);
P.vM_ref = P.vM_fn(q7,q7dot,rIxdot,rIydot,thetadot,theta);
P.vN_ref = P.vN_fn(q9,q9dot,rIxdot,rIydot,thetadot,theta);

end

function obj = Init_Obj_Sim(z0, P)

z = P.init0;

rIx = z(1);             rIy = z(2);             theta = z(3);
q1 = z(4);              q2 = z(5);              q3 = z(6);
q4 = z(7);              q5 = z(8);              q6 = z(9);
q7 = z(10);             q8 = z(11);             q9 = z(12);
q10 = z(13);
rIxdot = z(1+13);          rIydot = z(2+13);          thetadot = z(3+13);
q1dot = z(4+13);           q2dot = z(5+13);           q3dot = z(6+13);
q4dot = z(7+13);           q5dot = z(8+13);           q6dot = z(9+13);
q7dot = z(10+13);          q8dot = z(11+13);          q9dot = z(12+13);
q10dot = z(13+13);

rIy = z0(1);            
q1 = z0(2); 
q2 = z0(3); 
q3 = z0(4); 
q4 = z0(5); 
q5 = z0(6); 
q6 = z0(7); 
rIydot = z0(8);            
q1dot = z0(9); 
q2dot = z0(10); 
q3dot = z0(11); 
q4dot = z0(12); 
q5dot = z0(13); 
q6dot = z0(14); 

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

obj = -rT(2);
end
function [c,ceq] = Init_Cons_Sim(z0, P)

c = []; ceq = [];

z = P.init0;

rIx = z(1);             rIy = z(2);             theta = z(3);
q1 = z(4);              q2 = z(5);              q3 = z(6);
q4 = z(7);              q5 = z(8);              q6 = z(9);
q7 = z(10);             q8 = z(11);             q9 = z(12);
q10 = z(13);
rIxdot = z(1+13);          rIydot = z(2+13);          thetadot = z(3+13);
q1dot = z(4+13);           q2dot = z(5+13);           q3dot = z(6+13);
q4dot = z(7+13);           q5dot = z(8+13);           q6dot = z(9+13);
q7dot = z(10+13);          q8dot = z(11+13);          q9dot = z(12+13);
q10dot = z(13+13);

rIy = z0(1);            
q1 = z0(2); 
q2 = z0(3); 
q3 = z0(4); 
q4 = z0(5); 
q5 = z0(6); 
q6 = z0(7); 
rIydot = z0(8);            
q1dot = z0(9); 
q2dot = z0(10); 
q3dot = z0(11); 
q4dot = z0(12); 
q5dot = z0(13); 
q6dot = z0(14); 

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

vA = P.vA_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vB = P.vB_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vC = P.vC_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vD = P.vD_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
% vE = P.vE_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
% vF = P.vF_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
% vG = P.vG_fn(q1,q2,q1dot,q2dot,rIxdot,rIydot,thetadot,theta);
% vH = P.vH_fn(q1,q1dot,rIxdot,rIydot,thetadot,theta);
% vI = P.vI_fn(rIxdot,rIydot);
% vJ = P.vJ_fn(q4,q5,q4dot,q5dot,rIxdot,rIydot,thetadot,theta);
% vK = P.vK_fn(q4,q4dot,rIxdot,rIydot,thetadot,theta);
% vL = P.vL_fn(rIxdot,rIydot,thetadot,theta);
% vM = P.vM_fn(q7,q7dot,rIxdot,rIydot,thetadot,theta);
% vN = P.vN_fn(q9,q9dot,rIxdot,rIydot,thetadot,theta);

sigma0 = P.sigma0;

ceq = [ceq ; sigma0(1) * rA(2)];
ceq = [ceq ; sigma0(1) * rB(2)];
ceq = [ceq ; sigma0(1) * vA];
ceq = [ceq ; sigma0(1) * vB];
c = [c; sigma0(1) * (eps- rC(2))];
c = [c; sigma0(1) * (eps- rD(2))];

ceq = [ceq ; sigma0(2) * rC(2)];
ceq = [ceq ; sigma0(2) * rD(2)];
ceq = [ceq ; sigma0(2) * vC];
ceq = [ceq ; sigma0(2) * vD];
c = [c; sigma0(2) * (eps- rA(2))];
c = [c; sigma0(2) * (eps- rB(2))];
end
function x0_Opt = State_Sim2All(x0_All, x_Sim)

% rIy
x0_All(2) = x_Sim(1);
% q1 
x0_All(4) = x_Sim(2);
% q2 
x0_All(5) = x_Sim(3);
% q3 
x0_All(6) = x_Sim(4);
% q4 
x0_All(7) = x_Sim(5);
% q5 
x0_All(8) = x_Sim(6);
% q6 
x0_All(9) = x_Sim(7);

% rIy
x0_All(2+13) = x_Sim(1+7);
% q1 
x0_All(4+13) = x_Sim(2+7);
% q2 
x0_All(5+13) = x_Sim(3+7);
% q3 
x0_All(6+13) = x_Sim(4+7);
% q4 
x0_All(7+13) = x_Sim(5+7);
% q5 
x0_All(8+13) = x_Sim(6+7);
% q6 
x0_All(9+13) = x_Sim(7+7);

x0_Opt = x0_All;
end