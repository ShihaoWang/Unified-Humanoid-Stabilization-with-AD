function Snopt_Constraint_Formulation()
% This function is used to generate the constraint in a SNOPT-friendly way
load('Pre_Load_Structure.mat');

init_contas = [1 1];  
P.init_contas = init_contas;
rIx        = 0;             rIxlow = -Inf;          rIxupp = Inf;
rIy        = 0.5;           rIylow = 0;             rIyupp = Inf;
theta      = pi/2;        thetalow = 0;           thetaupp = pi;
q1         = pi/10;         q1low = -2*pi/3;        q1upp = pi/2; 
q2         = pi/6;          q2low = 0;              q2upp = pi;
q3         = pi/3;          q3low = -pi/2;          q3upp = 2*pi/3;
q4         = pi/4;          q4low = 0;              q4upp = pi;
q5         = pi/3;          q5low = -pi/2;          q5upp = pi;
q6         = pi/6;          q6low = 0;              q6upp = pi;
q7         = pi/3;          q7low = -pi;            q7upp = pi/2;
q8         = pi/10;         q8low = 0;              q8upp = pi;

rIxdot     = 1;           
rIydot     = 0;
thetadot   = 0.5;
q1dot      = 0.5;
q2dot      = -0.5;
q3dot      = 1;
q4dot      = 0.5;
q5dot      = 0.5;
q6dot      = 1;
q7dot      = -0.5;
q8dot      = 0.5;

x0 = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 ...
    rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot];
P.x0 = x0;
% Single_Frame_Plot(x0, P)
init_lb = [ rIxlow rIylow thetalow q1low q2low q3low q4low q5low q6low q7low q8low ];
init_ub = [ rIxupp rIyupp thetaupp q1upp q2upp q3upp q4upp q5upp q6upp q7upp q8upp ];

fmincon_opt = optimoptions('fmincon','Algorithm','sqp','display','off');
x = fmincon(@Init_Objective,x0,[],[],[],[],init_lb,init_ub,@Init_Constraint,fmincon_opt, P);
% figure
Single_Frame_Plot(x, P);

load('Symbolic_Structure.mat');

% There are three options can be customerized
mode_sequence = [1 0; 0 1];
gait_balance_flag = 1;     % flag: 1 for gait, 2 for stabilization 
grids_per_segment = 12;

Q.gait_balance_flag = gait_balance_flag;
Q.grids_per_segment = grids_per_segment; 
Constraint_Script_Gene(mode_sequence, x, Q);

end
