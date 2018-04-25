function sigma_children = Node_Expansion_Fn(sigma_i, x_i, P)

% This function is the main function used to expansion the given node to
% its adjacent nodes without any connectivity test

%%      Inputs;
%           sigma_i:        the contact status at the time i
%           x0:             the robot state at time i
%           P:              the pre-load structure

%%      Output:
%           sigma_children: the updated queue after a node expansion

%%      The main algorithm
%
%       Hand contact: 0-> Try the kinematical maximum step length to test the collision
%                           if collision detected? 
%                               Expanded with adding one hand contact
%                     1-> Since one hand is in contact,
%                               Expanded with adding the other hand contact
%                                             removing the current hand contact
%                     2-> Now two hands are in contact, 
%                               Expanded with retracting either hand contact     
%       Foot contact: 0-> Add either foot contact point next
%                     1-> Since one foot is in contact, adding one or
%                     removing one
%                     2-> Remove either foot contact
%                       

rIx = x_i(1);               rIy = x_i(2);               theta = x_i(3);
q1 = x_i(4);                q2 = x_i(5);                q3 = x_i(6);
q4 = x_i(7);                q5 = x_i(8);                q6 = x_i(9);
q7 = x_i(10);               q8 = x_i(11);               q9 = x_i(12);
q10 = x_i(13);

rIxdot = x_i(1+13);          rIydot = x_i(2+13);        thetadot = x_i(3+13);
q1dot = x_i(4+13);           q2dot = x_i(5+13);         q3dot = x_i(6+13);
q4dot = x_i(7+13);           q5dot = x_i(8+13);         q6dot = x_i(9+13);
q7dot = x_i(10+13);          q8dot = x_i(11+13);        q9dot = x_i(12+13);
q10dot = x_i(13+13);

sigma_children = []; % Initialization to be empty

foot_AB_contas = sigma_i(1);
foot_CD_contas = sigma_i(2);
hand_E_contas = sigma_i(3);
hand_F_contas = sigma_i(4);

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

%% Hand contact expansion
if (hand_E_contas == 0)&&(hand_F_contas == 0)
    %% 1. No contact case
    End_Hori_Max = max([rA(1), rB(1), rC(1), rD(1), rE(1), rF(1)]);
    End_Vert_Min = min([rA(2), rB(2), rC(2), rD(2), rE(2), rF(2)]);    
    if Obs_Dist_Fn([End_Hori_Max + P.Step_Length, End_Vert_Min], P.Envi_Map,'x')<0
        % In this case, there could be a hand collision
        sigma_i_child = sigma_modi(sigma_i, 3, 1);
        sigma_children = [sigma_children; sigma_i_child];
        sigma_i_child = sigma_modi(sigma_i, 4, 1);
        sigma_children = [sigma_children; sigma_i_child];      
    end
else
    %% 2. Two hand contact case
    
    if (hand_E_contas == 1)&&(hand_F_contas == 1)
        sigma_i_child = sigma_modi(sigma_i, 3, 0);
        sigma_children = [sigma_children; sigma_i_child];
        sigma_i_child = sigma_modi(sigma_i, 4, 0);
        sigma_children = [sigma_children; sigma_i_child];
    else
        %% 3. One hand contact case
        
        sigma_i_child = sigma_modi(sigma_i, 3, 0);
        sigma_i_child = sigma_modi(sigma_i_child, 4, 0);
        sigma_children = [sigma_children; sigma_i_child];
        
        sigma_i_child = sigma_modi(sigma_i, 3, 1);
        sigma_i_child = sigma_modi(sigma_i_child, 4, 1);
        sigma_children = [sigma_children; sigma_i_child];        
        
    end
end

%% Foot contact expansion
if (foot_AB_contas == 0)&&(foot_CD_contas == 0)
    %% 1. No contact case
    sigma_i_child = sigma_modi(sigma_i, 1, 1);
    sigma_children = [sigma_children; sigma_i_child];
    sigma_i_child = sigma_modi(sigma_i, 2, 1);
    sigma_children = [sigma_children; sigma_i_child];
else
    %% 2. Two foot contact case
    if (foot_AB_contas == 1)&&(foot_CD_contas == 1)       
        sigma_i_child = sigma_modi(sigma_i, 1, 0);
        sigma_children = [sigma_children; sigma_i_child];
        sigma_i_child = sigma_modi(sigma_i, 2, 0);
        sigma_children = [sigma_children; sigma_i_child];
    else
        %% 3. One foot contact case
        [~,Active_Ind] = max(sigma_i(1,1:2));
        [~,Inactive_Ind] = min(sigma_i(1,1:2));
        sigma_i_child = sigma_modi(sigma_i, Active_Ind, 0);
        sigma_children = [sigma_children; sigma_i_child];
        sigma_i_child = sigma_modi(sigma_i, Inactive_Ind, 1);
        sigma_children = [sigma_children; sigma_i_child];
    end
end
end

function sigma_i_child = sigma_modi(sigma_ref, contas_ind, AddOrRet)
sigma_i_child = sigma_ref;

sigma_i_child(contas_ind) = AddOrRet;
end

