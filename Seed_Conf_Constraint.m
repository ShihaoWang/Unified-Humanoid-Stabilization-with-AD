function [c, ceq] = Seed_Conf_Constraint(z,P)
c = []; ceq = [];
mini = P.mini;
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

rA = P.rA_fn(q1,q2,q3,rIx,rIy,theta);
rB = P.rB_fn(q1,q2,q3,rIx,rIy,theta);
rC = P.rC_fn(q4,q5,q6,rIx,rIy,theta);
rD = P.rD_fn(q4,q5,q6,rIx,rIy,theta);
rE = P.rE_fn(q7,q8,rIx,rIy,theta);
rF = P.rF_fn(q9,q10,rIx,rIy,theta);
rI = P.rI_fn(rIx,rIy);
rT = P.rT_fn(rIx,rIy,theta);

vA = P.vA_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vB = P.vB_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vC = P.vC_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vD = P.vD_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vE = P.vE_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
vF = P.vF_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);

% According to the way that sigma_child is generated, this offset vector
% can at most have one nonzero value
sigma_i = P.sigma_i;
sigma_i_child = P.sigma_i_child;
sigma_offset = sigma_i_child - sigma_i; 
P.sigma_offset = sigma_offset;

sigma_i_AB = sigma_i(1);
sigma_i_CD = sigma_i(2);
sigma_i_E = sigma_i(3);
sigma_i_F = sigma_i(4);

sigma_i_child_AB = sigma_i_child(1);
sigma_i_child_CD = sigma_i_child(2);
sigma_i_child_E = sigma_i_child(3);
sigma_i_child_F = sigma_i_child(4);

Envi_Map = P.Envi_Map;

%% 1. Relative distance constraints: a. all distances have to be at least on the surface; b. the desired mode has to be satisfied.
if (max(sigma_i_child) == min(sigma_i_child))&&max(sigma_i_child) == 0
    c = [c; norm(z(1:13) - P.x_i(1:13)) - 10*mini];    
    c = [c; sigma_offset(1) * vA(2);...
            sigma_offset(1) * vB(2);...
            sigma_offset(2) * vC(2);...
            sigma_offset(2) * vD(2);...
           -sigma_offset(3) * vE(1);...
           -sigma_offset(4) * vF(1)];   
end
c = [c; -Obs_Dist_Fn(rA, Envi_Map) + mini * ~sigma_i_child_AB;...
        -Obs_Dist_Fn(rB, Envi_Map) + mini * ~sigma_i_child_AB;...
        -Obs_Dist_Fn(rC, Envi_Map) + mini * ~sigma_i_child_CD;...
        -Obs_Dist_Fn(rD, Envi_Map) + mini * ~sigma_i_child_CD;...
        -Obs_Dist_Fn(rE, Envi_Map) + mini * ~sigma_i_child_E;...
        -Obs_Dist_Fn(rF, Envi_Map) + mini * ~sigma_i_child_F];
    
ceq = [ceq; sigma_i_child_AB * rA(2); sigma_i_child_AB * vA;...
            sigma_i_child_AB * rB(2); sigma_i_child_AB * vB;...
            sigma_i_child_CD * rC(2); sigma_i_child_CD * vC;...
            sigma_i_child_CD * rD(2); sigma_i_child_CD * vD;...
            sigma_i_child_E * Obs_Dist_Fn(rE, Envi_Map, 'x'); sigma_i_child_E * vE;...
            sigma_i_child_F * Obs_Dist_Fn(rF, Envi_Map, 'x'); sigma_i_child_F * vF];
 
%% 2. Contact Constraint Maintenance: the previous contacts have to be satisfied
if max(sigma_offset)==0  
    % This is a contact reduction
    ceq = [ceq; (sigma_offset(1)==0) * sigma_i_AB * (rA - P.rA_ref);
                (sigma_offset(1)==0) * sigma_i_AB * (rB - P.rB_ref);
                (sigma_offset(2)==0) * sigma_i_CD * (rC - P.rC_ref);
                (sigma_offset(2)==0) * sigma_i_CD * (rD - P.rD_ref);
                (sigma_offset(3)==0) * sigma_i_E *  (rE - P.rE_ref);
                (sigma_offset(4)==0) * sigma_i_F *  (rF - P.rF_ref)];
else
    % This is a contact addition
    ceq = [ceq; sigma_i_AB * (rA - P.rA_ref);
                sigma_i_AB * (rB - P.rB_ref);
                sigma_i_CD * (rC - P.rC_ref);
                sigma_i_CD * (rD - P.rD_ref);
                sigma_i_E * (rE - P.rE_ref);
                sigma_i_F * (rF - P.rF_ref)];
end
%% 3. Heuristic Stability Constraints: rI and rCOM have to be lied within the support polygon
r_Foot_Pos = [rA(1) rB(1);  rC(1) rD(1)];
rCOM = P.rCOM_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,rIx,rIy,theta);
% This is the heuristic part
if (max(sigma_i) == min(sigma_i))&&(max(sigma_i) == 0)
    % There is no contact in the parent mode
    % So the robot is jumping in the air so the idea is to make sure that the center of mass align
    % with the neck position in the horizontal direction
    ceq = [ceq; rCOM(1) - rT(1)];
else
    % There is at least one contact in the parent mode
    if(sigma_i(1)==1)||(sigma_i(2)==1)
        % At least foot contact is involved
        if sum(sigma_offset)>0    
            if find(sigma_offset)<3
                % The change is also foot contact
                if (P.vI_ref(1)>0)
                    % The robot is moving forward
                    [~,n] = find(sigma_offset);
                    Swing_Leg_Ind = n;
                    Stance_Leg_Ind = (Swing_Leg_Ind == 1)+ 1;
                    Swing_Leg_Dis = min(r_Foot_Pos(Swing_Leg_Ind,:));
                    Stance_Leg_Dis = max(r_Foot_Pos(Stance_Leg_Ind,:));
                    c = [c; Stance_Leg_Dis - Swing_Leg_Dis];
                    c = [c; min(r_Foot_Pos(Stance_Leg_Ind,:)) - rI(1);...
                        rI(1) - max(r_Foot_Pos(Swing_Leg_Ind,:))];
                    c = [c; min(r_Foot_Pos(Stance_Leg_Ind,:)) - rCOM(1);...
                        rCOM(1) - max(r_Foot_Pos(Swing_Leg_Ind,:))];
                    c = [c; min(rI(1),rCOM(1)) - rT(1);...
                            rT(1) - max(rI(1),rCOM(1))];
                else
                    [~,n] = find(sigma_offset);
                    Swing_Leg_Ind = n;
                    Stance_Leg_Ind = (Swing_Leg_Ind == 1)+ 1;
                    Swing_Leg_Dis = max(r_Foot_Pos(Swing_Leg_Ind,:));
                    Stance_Leg_Dis = min(r_Foot_Pos(Stance_Leg_Ind,:));
                    c = [c; -Stance_Leg_Dis + Swing_Leg_Dis];
                    c = [c; -max(r_Foot_Pos(Stance_Leg_Ind,:)) + rI(1);...
                        -rI(1) + min(r_Foot_Pos(Swing_Leg_Ind,:))];
                    c = [c; -max(r_Foot_Pos(Stance_Leg_Ind,:)) + rCOM(1);...
                        -rCOM(1) + min(r_Foot_Pos(Swing_Leg_Ind,:))];
                    c = [c; min(rI(1),rCOM(1)) - rT(1);...
                           rT(1) - max(rI(1),rCOM(1))];
                end
            else
                ceq = [ceq; rCOM(1) - rT(1)];
            end          
        else
            if  sum(sigma_offset)== 0
                % In this case, we are doing the same mode optimization and we
                % would like the center of mass and rI to be within the support polygon
                if sum(sigma_i(1:2))==2
                    c = [c; min(r_Foot_Pos(:)) - rI(1);...
                        rI(1) - max(r_Foot_Pos(:))];
                    c = [c; min(r_Foot_Pos(:)) - rCOM(1);...
                        rCOM(1) - max(r_Foot_Pos(:))];
%                     c = [c; min(rI(1),rCOM(1)) - rT(1);...
%                         rT(1) - max(rI(1),rCOM(1))];
                else
                    [~,n] = find(sigma_i(1:2));
                    Stance_Leg_Min = min(r_Foot_Pos(n,:));
                    Stance_Leg_Max = max(r_Foot_Pos(n,:));
                    c = [c; Stance_Leg_Min - rI(1);...
                        rI(1) - Stance_Leg_Max];
                    c = [c; Stance_Leg_Min - rCOM(1);...
                        rCOM(1) - Stance_Leg_Max];
%                     c = [c; min(rI(1),rCOM(1)) - rT(1);...
%                         rT(1) - max(rI(1),rCOM(1))];
                end
            else
                % This is the departure mode, the configuration should be
                % the same while the velocity at certain point should be in
                % the leaving direction
                
                c = [c; sigma_offset(1) * vA(2);...
                        sigma_offset(1) * vB(2);...
                        sigma_offset(2) * vC(2);...
                        sigma_offset(2) * vD(2);...
                        -sigma_offset(3) * vE(1);...
                        -sigma_offset(4) * vF(1)];             
            end
        end
    end
end
end