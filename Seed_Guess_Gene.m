function [Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd]= Seed_Guess_Gene(varargin)

% This function is used to generate the seed used for the optimization
%% The main idea is to generate a sequence of transition configurations then reversely compute the control torques
% Since sigma_i and x_i are selected from the Frontier set, the
% coresponding constraints have already been satisfied.
% The problem becomes how to address the constraint of sigma_child
global Node_i Node_i_child Tme_Seed Ctrl_No Active_Ind_Init Active_Ind_Tran Active_Ind_Goal

if length(varargin)>1
    Node_i = varargin{1};
    Node_i_child = varargin{2};
else
    Node_i = varargin{1};
    Node_i_child = Node_i;
end

sigma_i = Node_i.mode;
sigma_i_child = Node_i_child.mode;
sigma_i_change = sigma_i_child - sigma_i;
if max(sigma_i_change)==1
    % In this case, it is making contact
    sigma_tran = sigma_i;
    sigma_goal = sigma_i_child;
else
    % In this case, it is retracting contact or maintaining the same
    % contact condition
    sigma_tran = sigma_i_child;
    sigma_goal = sigma_i_child;    
end

%% The first step is to generate a feasible robot state that satisfies the sigma_child
[RobotState_LowBd, RobotState_UppBd, Ctrl_LowBd, Ctrl_UppBd] = Optimization_Bounds();
Seed_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','MaxIterations',inf,'MaxFunctionEvaluations',inf);

Flag = 0; Opt_Seed = []; Opt_Lowbd = []; Opt_Uppbd= [];
% Here an iterative optimization idea is adopted to minimize the difference
% between the new optimized translation to the initial translation position

%               1:          First-order optimality measure was less than options.OptimalityTolerance,
%                           and maximum constraint violation was less than options.ConstraintTolerance.
%               0:          Number of iterations exceeded options.MaxIterations or number of function
%                           evaluations exceeded options.MaxFunctionEvaluations.
%               -1:         Stopped by an output function or plot function.
%               -2:         No feasible point was found.
x_i_child = Node_i.robotstate;
[x_i_child,~,exitflag,output]= fmincon(@Seed_Conf_Obj,x_i_child,[],[],[],[],RobotState_LowBd,RobotState_UppBd,@Seed_Conf_Constraint,Seed_Opt);

if (exitflag==1)||(exitflag==2)||(output.constrviolation<0.001)
    Flag = 1;
else
    return
end
% Single_Frame_Plot(x_i_child);
%%  Here the desired configuration has been satisfied, then a seed control needs to be formulated
%   The following optimization is conducted with a transcription approach     
t_span = [0 Tme_Seed];

x_i = Node_i.robotstate;

x_span = [x_i x_i_child];

t_intp_span = linspace(t_span(1),t_span(2),Ctrl_No);

x_intp = spline(t_span, x_span, t_intp_span);

u_array = [];

delta_t = t_intp_span(2) - t_intp_span(1);

for j = 1:Ctrl_No
    % Control formulation at each time
    x0 = x_intp(:,j);   
    rIx = x0(1);             rIy = x0(2);             theta = x0(3);
    q1 = x0(4);              q2 = x0(5);              q3 = x0(6);
    q4 = x0(7);              q5 = x0(8);              q6 = x0(9);
    q7 = x0(10);             q8 = x0(11);             q9 = x0(12);
    q10 = x0(13);    
    x_state = x0(1:13,:);    
    rIxdot = x0(1+13);          rIydot = x0(2+13);          thetadot = x0(3+13);
    q1dot = x0(4+13);           q2dot = x0(5+13);           q3dot = x0(6+13);
    q4dot = x0(7+13);           q5dot = x0(8+13);           q6dot = x0(9+13);
    q7dot = x0(10+13);          q8dot = x0(11+13);          q9dot = x0(12+13);
    q10dot = x0(13+13);    
    if j == Ctrl_No
        xdotp1 = xdot;
        x_statep1 = x_state; 
    else
        x0p1 = x_intp(:,j+1);
        x_statep1 = x0p1(1:13,:);
        rIxdotp1 = x0p1(1+13);          rIydotp1 = x0p1(2+13);          thetadotp1 = x0p1(3+13);
        q1dotp1 = x0p1(4+13);           q2dotp1 = x0p1(5+13);           q3dotp1 = x0p1(6+13);
        q4dotp1 = x0p1(7+13);           q5dotp1 = x0p1(8+13);           q6dotp1 = x0p1(9+13);
        q7dotp1 = x0p1(10+13);          q8dotp1 = x0p1(11+13);          q9dotp1 = x0p1(12+13);
        q10dotp1 = x0p1(13+13);
        xdotp1 = [rIxdotp1 rIydotp1 thetadotp1 q1dotp1 q2dotp1 q3dotp1 q4dotp1 q5dotp1 q6dotp1 q7dotp1 q8dotp1 q9dotp1 q10dotp1]';
    end
    xdot = [rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]';
    qddot = (x_statep1 - x_state - xdot * delta_t)/(1/2 * delta_t^2) ;
    D_q = D_q_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    B_q = B_q_fn();
    C_q_qdot = C_q_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);    
    Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    Jacdot_qdot = Jacdot_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);  
    
    u_i_seed = B_q\(D_q * qddot + C_q_qdot);
    
    if j == 1
        sigma_t = sigma_i;
    else
        if j == Ctrl_No
            sigma_t = sigma_goal;
        else
            sigma_t = sigma_tran;        
        end
    end
    
    Active_In = Active_In_Cal_fn(Jac_Full, sigma_t);
    Jac_Act = Jac_Full(Active_In,:);
    Jacdot_qdot_Act = Jacdot_qdot(Active_In,:);
    
    if j == 1
        Active_Ind_Init = Active_In;
    else
        if j == Ctrl_No
            Active_Ind_Goal = Active_In;
        else
            Active_Ind_Tran = Active_In;
        end
    end

    lamda_i = (Jac_Act *(D_q\Jac_Act'))\(Jac_Act * (D_q\C_q_qdot) - Jacdot_qdot_Act - Jac_Act * (D_q\(B_q * u_i_seed)));
    u_i = B_q\(D_q * qddot + C_q_qdot - Jac_Act' * lamda_i);
    u_array = [u_array u_i];
end
StateNDot_Seed = reshape(x_intp, length(C_q_qdot)*2*Ctrl_No, 1);
Ctrl_Seed = reshape(u_array, length(u_i) *Ctrl_No,1 );
Opt_Seed = [Tme_Seed; StateNDot_Seed; Ctrl_Seed];

Opt_Lowbd = 0.1;
Opt_Uppbd = inf; 

Opt_Lowbd = [Opt_Lowbd; repmat(RobotState_LowBd', [Ctrl_No,1]); repmat(Ctrl_LowBd, [Ctrl_No,1])];
Opt_Uppbd = [Opt_Uppbd; repmat(RobotState_UppBd', [Ctrl_No,1]); repmat(Ctrl_UppBd, [Ctrl_No,1])];

end