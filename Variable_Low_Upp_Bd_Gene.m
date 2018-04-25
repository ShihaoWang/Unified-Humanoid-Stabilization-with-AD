function Snopt_Variable_Bound = Variable_Low_Upp_Bd_Gene(Seg_No, Grids_No, Q)

% This function is used to generate the variable low/upp bound
State_bound_char = char('rIx', 'rIy', 'theta', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8');
Cntrl_bound_char = char('u1', 'u2', 'u3', 'u4', 'u5', 'u6', 'u7', 'u8');
Lamda_bound_char = char('lamda_Ax', 'lamda_Ay', 'lamda_Bx', 'lamda_By', 'lamda_Cx', 'lamda_Cy', ...
    'lamda_Dx', 'lamda_Dy');
Lambar_bound_char = char('lambar_Ax', 'lambar_Ay', 'lambar_Bx', 'lambar_By', 'lambar_Cx', 'lambar_Cy',...
    'lambar_Dx', 'lambar_Dy');

xlow_start = 'xlow[';
xlow_end = '] = ';
xupp_start = ';   xupp[';
xupp_middle = '] = ';
xupp_end = '; \n';

Snopt_Variable_Bound = ['for( int i = 0; i<Seg_No; i++) \n { \n for( int j = 0; j<Grids_No; j++) \n {  '];

State_No = Q.State_No;
Ctrl_No = Q.Ctrl_No;
ConF_No = Q.ConF_No;

State_Ctrl_ConF_Len = 2*State_No + Ctrl_No + ConF_No; 

for i = 1:State_No
    variable_i_low = strcat(State_bound_char(i,:),'_low');
    variable_i_upp = strcat(State_bound_char(i,:),'_upp');
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xlow[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1) '] = ' variable_i_low '; \n    '];
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xupp[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1) '] = ' variable_i_upp '; \n    '];
end

for i = 1:State_No    
    variable_i_low = strcat(State_bound_char(i,:), 'dot_low');
    variable_i_upp = strcat(State_bound_char(i,:), 'dot_upp');
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xlow[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1 + State_No) '] = ' variable_i_low '; \n    '];
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xupp[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1 + State_No) '] = ' variable_i_upp '; \n    '];
end


% In this case, it is the control variable
for i = 1:Ctrl_No    
    variable_i_low = strcat('ctrl_low');
    variable_i_upp = strcat('ctrl_upp');
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xlow[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1 + 2*State_No) '] = ' variable_i_low '; \n    '];
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xupp[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1 + 2*State_No) '] = ' variable_i_upp '; \n    '];
end

% This is the contact force
for i = 1:ConF_No    
    variable_i_low = strcat(Lamda_bound_char(i,:), '_low');
    variable_i_upp = strcat(Lamda_bound_char(i,:), '_upp');    
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xlow[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1 + 2*State_No+Ctrl_No) '] = ' variable_i_low '; \n    '];
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xupp[i*Grids_No*' numstr(State_Ctrl_ConF_Len) '+j*' numstr(State_Ctrl_ConF_Len) '+' num2str(i-1 + 2*State_No+Ctrl_No) '] = ' variable_i_upp '; \n    '];
end

Snopt_Variable_Bound = [Snopt_Variable_Bound '} \n }\n'];

% This is the impulse

Snopt_Variable_Bound = [Snopt_Variable_Bound 'for(int i = 0; i < Seg_No - 1; i++) \n { \n    '];


Index2Impulse = Seg_No * Grids_No * State_Ctrl_ConF_Len;

for i = 1:ConF_No  
    variable_i_low = strcat(Lambar_bound_char(i,:),'_low');
    variable_i_upp = strcat(Lambar_bound_char(i,:),'_upp');      
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xlow[1920+i * ' num2str(ConF_No) ' + ' num2str(i-1) '] = ' variable_i_low '; \n    '];
    Snopt_Variable_Bound = [Snopt_Variable_Bound ' xupp[1920+i * ' num2str(ConF_No) ' + ' num2str(i-1) '] = ' variable_i_upp '; \n    '];  
end

Snopt_Variable_Bound = [Snopt_Variable_Bound  '} \n } \n'];
end