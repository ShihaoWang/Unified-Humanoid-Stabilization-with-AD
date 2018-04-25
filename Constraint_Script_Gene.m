function Constraint_Script_Gene(mode_sequence, x, Q)

% This function is used to generate the script used for SNOPT C++

syms rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 real
syms rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot real
syms rIxddot rIyddot thetaddot thetaddot q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot real
syms u1 u2 u3 u4 u5 u6 u7 u8 real

% This function is used to formulate the whole body stablization problem
[Seg_No,~] = size(mode_sequence);

gait_balance_flag = Q.gait_balance_flag;
Grids_No = Q.grids_per_segment;

% The default setting for time is 1s
Seg_Time = 1;
Grid_Time = Seg_Time/(Grids_No - 1);

for i = 1:Seg_No
    
    for j = 1:Grids_No
        
        % Initialization of the state
        eval(['syms ','rIx','_',num2str(i),'_',num2str(j),' rIy','_',num2str(i),'_',num2str(j),' theta','_',...
            num2str(i),'_',num2str(j),' q1','_',num2str(i),'_',num2str(j),' q2','_',num2str(i),'_',num2str(j)...
            ,' q3','_',num2str(i),'_',num2str(j),' q4','_',num2str(i),'_',num2str(j),' q5','_',...
            num2str(i),'_',num2str(j),' q6','_',num2str(i),'_',num2str(j),' q7','_',num2str(i),...
            '_',num2str(j),' q8','_',num2str(i),'_',num2str(j),' real']);
        
        % Initialization of the statedot
        eval(['syms ',' rIxdot','_',num2str(i),'_',num2str(j),' rIydot','_',num2str(i),'_',num2str(j),...
            ' thetadot','_',num2str(i),'_',num2str(j),' q1dot','_',num2str(i),'_',num2str(j),...
            ' q2dot','_',num2str(i),'_',num2str(j) ,' q3dot','_',num2str(i),'_',num2str(j),....
            ' q4dot','_',num2str(i),'_',num2str(j),' q5dot','_',num2str(i),'_',num2str(j),...
            ' q6dot','_',num2str(i),'_',num2str(j),' q7dot','_',num2str(i),'_',num2str(j)...
            ,' q8dot','_',num2str(i),'_',num2str(j),' real']);
        
        % Initialization of the lamda
        eval(['syms ',' lamda_Ax','_',num2str(i),'_',num2str(j),' lamda_Ay','_',num2str(i),'_',num2str(j),...
            ' lamda_Bx','_',num2str(i),'_',num2str(j),' lamda_By','_',num2str(i),'_',num2str(j),...
            ' lamda_Cx','_',num2str(i),'_',num2str(j),' lamda_Cy','_',num2str(i),'_',num2str(j),...
            ' lamda_Dx','_',num2str(i),'_',num2str(j),' lamda_Dy','_',num2str(i),'_',num2str(j),...
            ' real']);
        % Initialization of the control
        eval(['syms ',' u1','_',num2str(i),'_',num2str(j),' u2','_',num2str(i),'_',num2str(j),...
            ' u3','_',num2str(i),'_',num2str(j),' u4','_',num2str(i),'_',num2str(j),...
            ' u5','_',num2str(i),'_',num2str(j),' u6','_',num2str(i),'_',num2str(j),...
            ' u7','_',num2str(i),'_',num2str(j),' u8','_',num2str(i),'_',num2str(j),' real']);
    end
    if i<Seg_No
        eval(['syms ',' lambar_Ax','_',num2str(i),' lambar_Ay','_',num2str(i),...
            ' lambar_Bx','_',num2str(i),' lambar_By','_',num2str(i),...
            ' lambar_Cx','_',num2str(i),' lambar_Cy','_',num2str(i),...
            ' lambar_Dx','_',num2str(i),' lambar_Dy','_',num2str(i),...
            ' real']);
    end
end

save('gait_sym.mat')
load('gait_sym.mat')

Snopt_State = [];
Snopt_State_Bound = [];
Snopt_State_Raw = [];

% To make sure that an obvious pattern can be found, we would prefer to
% generate the state one by one

%% First is the generation of the joint angles and angular velocities
for i = 1: Seg_No
    for j = 1:Grids_No
        % 1. State and Statedot
        eval(['rIx_new = rIx_',num2str(i),'_',num2str(j),';']);
        eval(['rIy_new = rIy_',num2str(i),'_',num2str(j),';']);
        eval(['theta_new = theta_',num2str(i),'_',num2str(j),';']);
        eval(['q1_new = q1_',num2str(i),'_',num2str(j),';']);
        eval(['q2_new = q2_',num2str(i),'_',num2str(j),';']);
        eval(['q3_new = q3_',num2str(i),'_',num2str(j),';']);
        eval(['q4_new = q4_',num2str(i),'_',num2str(j),';']);
        eval(['q5_new = q5_',num2str(i),'_',num2str(j),';']);
        eval(['q6_new = q6_',num2str(i),'_',num2str(j),';']);
        eval(['q7_new = q7_',num2str(i),'_',num2str(j),';']);
        eval(['q8_new = q8_',num2str(i),'_',num2str(j),';']);
        
        eval(['rIxdot_new = rIxdot_',num2str(i),'_',num2str(j),';']);
        eval(['rIydot_new = rIydot_',num2str(i),'_',num2str(j),';']);
        eval(['thetadot_new = thetadot_',num2str(i),'_',num2str(j),';']);
        eval(['q1dot_new = q1dot_',num2str(i),'_',num2str(j),';']);
        eval(['q2dot_new = q2dot_',num2str(i),'_',num2str(j),';']);
        eval(['q3dot_new = q3dot_',num2str(i),'_',num2str(j),';']);
        eval(['q4dot_new = q4dot_',num2str(i),'_',num2str(j),';']);
        eval(['q5dot_new = q5dot_',num2str(i),'_',num2str(j),';']);
        eval(['q6dot_new = q6dot_',num2str(i),'_',num2str(j),';']);
        eval(['q7dot_new = q7dot_',num2str(i),'_',num2str(j),';']);
        eval(['q8dot_new = q8dot_',num2str(i),'_',num2str(j),';']);
        
        Snopt_State = [Snopt_State  rIx_new, rIy_new, theta_new, q1_new, q2_new, q3_new, q4_new, q5_new, q6_new, q7_new, q8_new,...
            rIxdot_new, rIydot_new, thetadot_new, q1dot_new, q2dot_new, q3dot_new, q4dot_new, q5dot_new, q6dot_new, q7dot_new, q8dot_new];
                
        % 2. Control variables
        eval(['u1_new = u1_',num2str(i),'_',num2str(j),';']);
        eval(['u2_new = u2_',num2str(i),'_',num2str(j),';']);
        eval(['u3_new = u3_',num2str(i),'_',num2str(j),';']);
        eval(['u4_new = u4_',num2str(i),'_',num2str(j),';']);
        eval(['u5_new = u5_',num2str(i),'_',num2str(j),';']);
        eval(['u6_new = u6_',num2str(i),'_',num2str(j),';']);
        eval(['u7_new = u7_',num2str(i),'_',num2str(j),';']);
        eval(['u8_new = u8_',num2str(i),'_',num2str(j),';']);
        
        Snopt_State = [Snopt_State  u1_new u2_new u3_new u4_new u5_new u6_new u7_new u8_new];
        
        % 3. Contact force variables
        eval(['lamda_Ax_new = lamda_Ax_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Ay_new = lamda_Ay_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Bx_new = lamda_Bx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_By_new = lamda_By_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cx_new = lamda_Cx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cy_new = lamda_Cy_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dx_new = lamda_Dx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dy_new = lamda_Dy_',num2str(i),'_',num2str(j),';']);
        
        Snopt_State = [Snopt_State  lamda_Ax_new lamda_Ay_new lamda_Bx_new lamda_By_new lamda_Cx_new lamda_Cy_new...
                                    lamda_Dx_new lamda_Dy_new];
    end
end

for i = 1:Seg_No-1
    % Here we come to the ending time of the current segments and it is time to add the impact mapping constraint into the new state
    eval(['lambar_i = [',' lambar_Ax_',num2str(i),' lambar_Ay_',num2str(i),...
        ' lambar_Bx_',num2str(i),' lambar_By_',num2str(i),...
        ' lambar_Cx_',num2str(i),' lambar_Cy_',num2str(i),...
        ' lambar_Dx_',num2str(i),' lambar_Dy_',num2str(i),'];']);
    Snopt_State = [Snopt_State  lambar_i];    
end
Snopt_State = Snopt_State.';

Snopt_Variable_Bound = Variable_Low_Upp_Bd_Gene(Seg_No, Grids_No, Q);
end

