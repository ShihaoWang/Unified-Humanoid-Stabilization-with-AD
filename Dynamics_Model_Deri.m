function Dynamics_Model_Deri()

% This function is used to generate the equation of motion for the 11-link
% humanoid robot. The physical parameters are retrieved from the HRP2 robot

% The item sequence is [foot, shank, thigh, body ,head, forearm, arm]

% The length of the humanoid link
l_shank = 0.325;        l_thigh = 0.325;            l_body = 0.55;
l_arm = 0.25;           l_forearm = 0.45;           
l_foot = 0.225;         l_heel = 0.1;               l_head = 0.15;

l = [l_shank; l_thigh; l_body; l_forearm; l_arm; l_foot];

% The mass of the humanoid link
m_shank = 2.4;          m_thigh = 4;                m_body = 27;
m_arm = 3.5;            m_forearm = 2.1;            
m_foot = 1.6;    

m = [m_shank; m_thigh; m_body; m_forearm; m_arm; m_foot];

I = 1/12 * m .* l.* l;          p.m = m;    p.I = I;

g = 9.81;

syms rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
syms rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot real
syms rIxddot rIyddot thetaddot q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot q9ddot q10ddot real
syms u1 u2 u3 u4 u5 u6 u7 u8 u9 u10 real

% Foot geometry
l_foot_back = sqrt(2) * l_heel;
l_foot_front = ((l_foot - l_heel)^2 + l_heel^2)^(1/2);
alpha = atan((l_foot - l_heel)/l_heel);

AngxIL = -theta+pi/2; 
AngxIT = AngxIL;
AngxIH = -(pi/2 + theta + q1);
AngxHG = AngxIH - q2;
AngxGMAB = AngxHG - q3;
AngxGB = 2 * pi + AngxGMAB - pi/4;
AngxGA = AngxGMAB + alpha;
AngxAB = AngxGMAB + pi/2 + pi;

AngxIK = -(pi/2 + theta + q4);
AngxKJ = AngxIK - q5;
AngxJMCD = AngxKJ - q6;
AngxJD = AngxJMCD - pi/4;
AngxJC = AngxJMCD + alpha;
AngxCD = AngxJMCD - pi/2;

AngxLM = -(pi/2 + theta + q7);
AngxME = AngxLM - q8;

AngxLN = -(pi/2 + theta + q9);
AngxNF = AngxLN - q10;

rI = [rIx, rIy]';

rIH = l_thigh * lamdadirection(AngxIH);
rH = rI + rIH;
rHG = l_shank * lamdadirection(AngxHG);
rG = rH + rHG;
rGB = l_foot_back * lamdadirection(AngxGB);
rB = rG + rGB;
rGA = l_foot_front * lamdadirection(AngxGA);
rA = rG + rGA;

rIK = l_thigh * lamdadirection(AngxIK);
rK = rI + rIK;
rKJ = l_shank * lamdadirection(AngxKJ);
rJ = rK + rKJ;
rJD = l_foot_back * lamdadirection(AngxJD);
rD = rJ + rJD;
rJC = l_foot_front * lamdadirection(AngxJC);
rC = rJ + rJC;

rIL = l_body * lamdadirection(AngxIT);
rL = rI + rIL;
rLT = l_head * lamdadirection(AngxIT);
rT = rL + rLT;

rLM = l_arm * lamdadirection(AngxLM);
rM = rL + rLM;
rME = l_forearm * lamdadirection(AngxME);
rE = rM + rME;

rLN = l_arm * lamdadirection(AngxLN);
rN = rL + rLN;
rNF = l_forearm * lamdadirection(AngxNF);
rF = rN + rNF;

% Here Q is a structure used to save the symbolic expression
Q.rA = rA;
Q.rB = rB;
Q.rC = rC;
Q.rD = rD;
Q.rE = rE;
Q.rF = rF;
Q.rG = rG;
Q.rH = rH;
Q.rI = rI;
Q.rJ = rJ;
Q.rK = rK;
Q.rL = rL;
Q.rM = rM;
Q.rN = rN;
Q.rT = rT;

rCOM_num = ((rA + rB)/2 + (rC + rD)/2)* m_foot + ((rG + rH)/2 + (rK + rJ)/2) * m_shank + ...
        ((rH + rI)/2 + (rI + rK)/2)* m_thigh + (rI + rL)/2 * m_body + ...
        ((rL + rM)/2 + (rL + rN)/2)* m_arm + ((rM + rE)/2 + (rN + rF)/2) * m_forearm;
rCOM_den = 2 * (m_foot + m_shank + m_thigh + m_arm + m_forearm) + m_body;       
rCOM = simplify(rCOM_num/rCOM_den);
matlabFunction(rCOM,'File','rCOM_fn');
Q.rCOM = rCOM;

r_vec = [rA;  rB;  rC;  rD;  rE;  rF;  rG;  rH;  rI;  rJ;  rK;  rL;  rM;  rN];

% This part is the generalized coordinate of the robots
q =     [rIx,     rIy,     theta,     q1,     q2,     q3,     q4,     q5,     q6,     q7,     q8,       q9,         q10]';
qdot =  [rIxdot,  rIydot,  thetadot,  q1dot,  q2dot,  q3dot,  q4dot,  q5dot,  q6dot,  q7dot,  q8dot,    q9dot,      q10dot]';
qddot = [rIxddot, rIyddot, thetaddot, q1ddot, q2ddot, q3ddot, q4ddot, q5ddot, q6ddot, q7ddot, q8ddot,   q9ddot,     q10ddot]';

% Velocity computation
vA = jacobian(rA, q) * qdot; 
vB = jacobian(rB, q) * qdot; 
vC = jacobian(rC, q) * qdot; 
vD = jacobian(rD, q) * qdot; 
vE = jacobian(rE, q) * qdot; 
vF = jacobian(rF, q) * qdot; 
vG = jacobian(rG, q) * qdot; 
vH = jacobian(rH, q) * qdot; 
vI = jacobian(rI, q) * qdot; 
vJ = jacobian(rJ, q) * qdot; 
vK = jacobian(rK, q) * qdot; 
vL = jacobian(rL, q) * qdot; 
vM = jacobian(rM, q) * qdot; 
vN = jacobian(rN, q) * qdot; 

v_vec = [vA; vB; vC; vD; vE; vF; vG; vH; vI; vJ; vK; vL; vM; vN];

Q.vA = vA;
Q.vB = vB;
Q.vC = vC;
Q.vD = vD;
Q.vE = vE;
Q.vF = vF;
Q.vG = vG;
Q.vH = vH;
Q.vI = vI;
Q.vJ = vJ;
Q.vK = vK;
Q.vL = vL;
Q.vM = vM;
Q.vN = vN;

[T_IH, V_IH] = Kinematics_Cal(AngxIH, 'AngxIH', q, qdot, r_vec, v_vec, p);
[T_HG, V_HG] = Kinematics_Cal(AngxHG, 'AngxHG', q, qdot, r_vec, v_vec, p);

[T_IK, V_IK] = Kinematics_Cal(AngxIK, 'AngxIK', q, qdot, r_vec, v_vec, p);
[T_KJ, V_KJ] = Kinematics_Cal(AngxKJ, 'AngxKJ', q, qdot, r_vec, v_vec, p);

[T_IL, V_IL] = Kinematics_Cal(AngxIL, 'AngxIL', q, qdot, r_vec, v_vec, p);

[T_LM, V_LM] = Kinematics_Cal(AngxLM, 'AngxLM', q, qdot, r_vec, v_vec, p);
[T_ME, V_ME] = Kinematics_Cal(AngxME, 'AngxME', q, qdot, r_vec, v_vec, p);

[T_LN, V_LN] = Kinematics_Cal(AngxLN, 'AngxLN', q, qdot, r_vec, v_vec, p);
[T_NF, V_NF] = Kinematics_Cal(AngxNF, 'AngxNF', q, qdot, r_vec, v_vec, p);

[T_AB, V_AB] = Kinematics_Cal(AngxAB, 'AngxAB', q, qdot, r_vec, v_vec, p);
[T_CD, V_CD] = Kinematics_Cal(AngxCD, 'AngxCD', q, qdot, r_vec, v_vec, p);

T = T_IH + T_HG + T_IK + T_KJ + T_IL + T_LM + T_ME + T_LN + T_NF + T_AB + T_CD;
V = V_IH + V_HG + V_IK + V_KJ + V_IL + V_LM + V_ME + V_LN + V_NF + V_AB + V_CD;

T = simplify(T);
V = simplify(V);
matlabFunction(T,'File','T_fn');
matlabFunction(V,'File','V_fn');

Q.T = T;
Q.V = V; 

u = [u1 u2 u3 u4 u5 u6 u7 u8 u9 u10]'; 

L = T - V;
pL_pq = simplify(jacobian(L, q));
pL_pqdot = simplify(jacobian(L,qdot));
d_pL_pqdot_dt = simplify(jacobian(pL_pqdot, [q; qdot]) * [qdot; qddot]);

Eqn = d_pL_pqdot_dt - pL_pq.' - [0; 0; 0; u];
Eqn = simplify(Eqn);
D_q = jacobian(Eqn, qddot);
B_q = -jacobian(Eqn, u);
C_q_qdot = simplify(Eqn - D_q * qddot + B_q * u);

matlabFunction(D_q,'File','D_q_fn'); 
matlabFunction(B_q,'File','B_q_fn');
matlabFunction(C_q_qdot,'File','C_q_qdot_fn'); 

Q.D_q = D_q;
Q.B_q = B_q;
Q.C_q_qdot = C_q_qdot;

% Now let us come to the computation of the full dynamics constraint jacobian matrix

Phi_Eqn = [rA; rB;  rC; rD; rE; rF];
Jac_Full = jacobian(Phi_Eqn, q);
matlabFunction(Jac_Full,'File','Jac_Full_fn');
Q.Jac_Full = Jac_Full;

Jac_Full_dot_Eqn = simplify(jacobian(Jac_Full * qdot, [q;qdot]) * [qdot;qddot]);
Jacdot_qdot = simplify(Jac_Full_dot_Eqn - Jac_Full * qddot);

matlabFunction(Jacdot_qdot,'File','Jacdot_qdot_fn'); 

Q.Jac_Full_dot_Eqn = Jac_Full_dot_Eqn; 
Q.Jacdot_qdot = Jacdot_qdot;

matlabFunction(rA,'File','rA_fn');%@(q1,q2,q3,rIx,rIy,theta)
matlabFunction(rB,'File','rB_fn');%@(q1,q2,rIx,rIy,theta)
matlabFunction(rC,'File','rC_fn');%@(q4,q5,q6,rIx,rIy,theta)
matlabFunction(rD,'File','rD_fn');%@(q4,q5,q6,rIx,rIy,theta)
matlabFunction(rE,'File','rE_fn');%@(q7,q8,rIx,rIy,theta)
matlabFunction(rF,'File','rF_fn');%@(q9,q10,rIx,rIy,theta)
matlabFunction(rG,'File','rG_fn');%@(q1,q2,rIx,rIy,theta)
matlabFunction(rH,'File','rH_fn');%@(q1,rIx,rIy,theta)
matlabFunction(rI,'File','rI_fn');%@(rIx,rIy)
matlabFunction(rJ,'File','rJ_fn');%@(q4,q5,rIx,rIy,theta)
matlabFunction(rK,'File','rK_fn');%@(q4,rIx,rIy,theta)
matlabFunction(rL,'File','rL_fn');%@(rIx,rIy,theta)
matlabFunction(rM,'File','rM_fn');%@(q7,rIx,rIy,theta)
matlabFunction(rN,'File','rN_fn');%@(q9,rIx,rIy,theta)
matlabFunction(rT,'File','rT_fn');%@(rIx,rIy,theta)

matlabFunction(vA,'File','vA_fn');%@(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta)
matlabFunction(vB,'File','vB_fn');%@(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta)  
matlabFunction(vC,'File','vC_fn');%@(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta)  
matlabFunction(vD,'File','vD_fn');%@(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta)
matlabFunction(vE,'File','vE_fn');%@(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta) 
matlabFunction(vF,'File','vF_fn');%@(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta) 
matlabFunction(vG,'File','vG_fn');%@(q1,q2,q1dot,q2dot,rIxdot,rIydot,thetadot,theta) 
matlabFunction(vH,'File','vH_fn');%@(q1,q1dot,rIxdot,rIydot,thetadot,theta) 
matlabFunction(vI,'File','vI_fn');%@(rIxdot,rIydot)
matlabFunction(vJ,'File','vJ_fn');%@(q4,q5,q4dot,q5dot,rIxdot,rIydot,thetadot,theta)  
matlabFunction(vK,'File','vK_fn');%@(q4,q4dot,rIxdot,rIydot,thetadot,theta)  
matlabFunction(vL,'File','vL_fn');%@(rIxdot,rIydot,thetadot,theta)  
matlabFunction(vM,'File','vM_fn');%@(q7,q7dot,rIxdot,rIydot,thetadot,theta)  
matlabFunction(vN,'File','vN_fn');%@(q9,q9dot,rIxdot,rIydot,thetadot,theta)  

save('Symbolic_Structure.mat','Q');
end

function [T_i, V_i] = Kinematics_Cal(Angxi, Angxi_name, q, qdot, r_vec, v_vec, p)
% This function is used to calculate the kinematics of a given link

m_shank = p.m(1);       m_thigh = p.m(2);       m_body = p.m(3);       
m_arm = p.m(4);         m_forearm = p.m(5);     m_foot = p.m(6);  

I_shank = p.I(1);       I_thigh = p.I(2);       I_body = p.I(3);        
I_arm = p.I(4);         I_forearm = p.I(5);     I_foot = p.I(6); 

rA = r_vec(1:2,:);
rB = r_vec(3:4,:);
rC = r_vec(5:6,:);
rD = r_vec(7:8,:);
rE = r_vec(9:10,:);
rF = r_vec(11:12,:);
rG = r_vec(13:14,:);
rH = r_vec(15:16,:);
rI = r_vec(17:18,:);
rJ = r_vec(19:20,:);
rK = r_vec(21:22,:);
rL = r_vec(23:24,:);
rM = r_vec(25:26,:);
rN = r_vec(27:28,:);

vA = v_vec(1:2,:); 
vB = v_vec(3:4,:); 
vC = v_vec(5:6,:); 
vD = v_vec(7:8,:); 
vE = v_vec(9:10,:); 
vF = v_vec(11:12,:); 
vG = v_vec(13:14,:); 
vH = v_vec(15:16,:); 
vI = v_vec(17:18,:); 
vJ = v_vec(19:20,:); 
vK = v_vec(21:22,:); 
vL = v_vec(23:24,:); 
vM = v_vec(25:26,:); 
vN = v_vec(27:28,:); 

mIL = m_body;
mHG = m_shank;          mKJ = mHG;          mIH = m_thigh;          mIK = mIH;
mLM = m_arm;            mLN = mLM;          mME = m_forearm;        mNF = mME;
mAB = m_foot;           mCD = mAB;


IIL = I_body;
IHG = I_shank;          IKJ = IHG;          IIH = I_thigh;          IIK = IIH;
ILM = I_arm;            ILN = ILM;          IME = I_forearm;        INF = IME;
IAB = I_foot;           ICD = IAB;

AngRatexi = jacobian(Angxi, q) * qdot;
i = 1;
while(i<length(Angxi_name))
    Angxi_name_i = Angxi_name(i);
    if Angxi_name_i=='x'
        r1 = Angxi_name(i + 1);
        r2 = Angxi_name(i + 2);
        break;
    end
    i = i + 1;
end

% Then it is to find the right name for this link
temp_name = strcat(r1,r2);
mass_temp_name = strcat('m',temp_name);
if (exist(mass_temp_name,'var') == 0)
    temp_name = strcat(r2,r1);
end

mass_name = strcat('m',temp_name);
Inertia_name = strcat('I',temp_name);
Edge_pos_1 = strcat('r',r1);
Edge_pos_2 = strcat('r',r2);
Edge_vel_1 = strcat('v',r1);
Edge_vel_2 = strcat('v',r2);

evalc(['link_mass = ' mass_name]);
evalc(['link_inertia = ' Inertia_name]);
evalc(['r_pos_1 = ' Edge_pos_1]);
evalc(['r_pos_2 = ' Edge_pos_2]);

evalc(['r_vel_1 = ' Edge_vel_1]);
evalc(['r_vel_2 = ' Edge_vel_2]);

T_i = 1/2 * link_mass * dot(0.5 * (r_vel_1 + r_vel_2), 0.5 * (r_vel_1 + r_vel_2)) + 1/2 * link_inertia * dot(AngRatexi,AngRatexi);
V_i = link_mass * 9.81 * (r_pos_1(2) + r_pos_2(2))/2;

end

function lamda_vec = lamdadirection(theta)

% This function is used to generate the unit direction vector under a given
% angle

% This angle is computed with respect to the horizontal line and the
% counterclockwise direction is the positive direction

lamda_vec = [cos(theta), sin(theta)].';

end