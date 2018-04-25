function [Flag, Opt_Seed, Opt_Lowbd, Opt_Uppbd, P]= Seed_Guess_Gene(sigma_i, x_i, sigma_i_child, P)

% This function is used to generate the seed used for the optimization

%% The main idea is to generate a sequence of transition configurations then reversely compute the control torques
% Since sigma_i and x_i are selected from the Frontier set, the
% coresponding constraints have already been satisfied.
% The problem becomes how to address the constraint of sigma_child

%% The first step is to generate a feasible robot state that satisfies the sigma_child
RobotState_LowBd = P.RobotState_LowBd;
RobotState_UppBd = P.RobotState_UppBd;

ContactForce_LowBd = P.ContactForce_LowBd;
ContactForce_UppBd = P.ContactForce_UppBd;

Ctrl_LowBd = P.Ctrl_LowBd;
Ctrl_UppBd = P.Ctrl_UppBd;

Seed_Opt = optimoptions(@fmincon,'Display','off','Algorithm','sqp','MaxIterations',inf,'OptimalityTolerance',1e-8,'MaxFunctionEvaluations',inf);
Flag = 1;
% Here an iterative optimization idea is adopted to minimize the difference
% between the new optimized translation to the initial translation position

%               1:          First-order optimality measure was less than options.OptimalityTolerance,
%                           and maximum constraint violation was less than options.ConstraintTolerance.
%               0:          Number of iterations exceeded options.MaxIterations or number of function
%                           evaluations exceeded options.MaxFunctionEvaluations.
%               -1:         Stopped by an output function or plot function.
%               -2:         No feasible point was found.
[x_i_child,~,exitflag,~]= fmincon(@Seed_Conf_Obj,x_i,[],[],[],[],RobotState_LowBd,RobotState_UppBd,@Seed_Conf_Constraint,Seed_Opt, P);

if (exitflag~=1)&&(exitflag~=2)
    Flag = 0;
    return
end

P.x_i_child = x_i_child;   % This is the goal configuratioin at this step
Single_Frame_Plot(x_i_child, P);

%% Here the desired configuration has been satisfied, then a seed control needs to be formulated
% Then a linear interpolation can be calculated based on the sigma_i
Tme_Seed = P.Tme_Seed;
Ctrl_No = P.Ctrl_No;

% % Initial state of the robot
% rIx_0 = x_i(1);      rIy_0 = x_i(2);      theta_0 = x_i(3);
% q1_0 = x_i(4);       q2_0 = x_i(5);       q3_0 = x_i(6);
% q4_0 = x_i(7);       q5_0 = x_i(8);       q6_0 = x_i(9);
% q7_0 = x_i(10);      q8_0 = x_i(11);      q9_0 = x_i(12);     q10_0 = x_i(13);  
% 
% rIxdot_0 = x_i(1+13);      rIydot_0 = x_i(2+13);      thetadot_0 = x_i(3+13);
% q1dot_0 = x_i(4+13);       q2dot_0 = x_i(5+13);       q3dot_0 = x_i(6+13);
% q4dot_0 = x_i(7+13);       q5dot_0 = x_i(8+13);       q6dot_0 = x_i(9+13);
% q7dot_0 = x_i(10+13);      q8dot_0 = x_i(11+13);      q9dot_0 = x_i(12+13);     q10dot_0 = x_i(13+13);  
% 
% x_0 = [rIx_0 rIy_0 theta_0 q1_0 q2_0 q3_0 q4_0 q5_0 q6_0 q7_0 q8_0 q9_0 q10_0...
%        rIxdot_0 rIydot_0 thetadot_0 q1dot_0 q2dot_0 q3dot_0 q4dot_0 q5dot_0 q6dot_0 q7dot_0 q8dot_0 q9dot_0 q10dot_0]';

% % Desired state of the robot
% x_i_child = P.x_i_child;
% rIx = x_i_child(1);             rIy = x_i_child(2);             theta = x_i_child(3);
% q1 = x_i_child(4);              q2 = x_i_child(5);              q3 = x_i_child(6);
% q4 = x_i_child(7);              q5 = x_i_child(8);              q6 = x_i_child(9);
% q7 = x_i_child(10);             q8 = x_i_child(11);             q9 = x_i_child(12);
% q10 = x_i_child(13);
% 
% rIxdot = x_i_child(1+13);          rIydot = x_i_child(2+13);          thetadot = x_i_child(3+13);
% q1dot = x_i_child(4+13);           q2dot = x_i_child(5+13);           q3dot = x_i_child(6+13);
% q4dot = x_i_child(7+13);           q5dot = x_i_child(8+13);           q6dot = x_i_child(9+13);
% q7dot = x_i_child(10+13);          q8dot = x_i_child(11+13);          q9dot = x_i_child(12+13);
% q10dot = x_i_child(13+13);
% 
% x_des = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10...
%          rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]';

%% The following optimization is conducted with a transcription approach     
t_span = [0 Tme_Seed];

x_span = [x_i' x_i_child'];

sigma_offset = sigma_i_child - sigma_i;

if max(sigma_offset)==1
    % The main idea is that if there is a oontact addition, the sigma_i
    % should be satisfied, or sigma_i_child should be satisfied.
    Holo_Ind = Holo_Ind_fn(sigma_i);
else
    Holo_Ind = Holo_Ind_fn(sigma_i_child);
end

t_intp_span = linspace(t_span(1),t_span(2),Ctrl_No);

x_intp = spline(t_span, x_span, t_intp_span);

u_array = [];

ContactForce_tot = [];

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
    delta_t = t_intp_span(2) - t_intp_span(1);    
    qddot = (x_statep1 - x_state - xdot * delta_t)/(1/2 * delta_t^2) ;
    D_q = P.D_q_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    B_q = P.B_q_fn();
    C_q_qdot = P.C_q_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);     
    if sum(Holo_Ind)==0
        u_i = B_q\(D_q * qddot + C_q_qdot);       
    else
        % Full rank distillation
        Jac_Full = P.Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
        Jacdot_qdot = P.Jacdot_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
        
        Active_Rows = Jac_Ind_Fn(Holo_Ind);
        Jac_Full = Jac_Full(Active_Rows,:);
        Jacdot_qdot_Full = Jacdot_qdot(Active_Rows,:);
        
        Jac_Temp = Jac_Full';
        [~,colind] = rref(Jac_Temp);
        P.Active_In = Active_Rows(colind);
        Jac = Jac_Temp(:, colind)';
        Jacdot_qdot_temp = Jacdot_qdot_Full';
        Jacdot = Jacdot_qdot_temp(:, colind)';   
        
        P.D_q = D_q;
        P.B_q = B_q;
        P.C_q = C_q_qdot;
        P.qddot = qddot;
        P.Jac = Jac;
        P.Jacdot = Jacdot;      
        u_seed_i = B_q\(D_q * qddot + C_q_qdot);
        Contact_Force_i = Colind2ContactForce(colind,u_seed_i, P);
        ContactForce_tot = [ContactForce_tot; Contact_Force_i];
        
        Seed_Sub_Opt = optimoptions(@fmincon,'Display','off','Algorithm','sqp','MaxIterations',inf,'OptimalityTolerance',1e-8,'MaxFunctionEvaluations',inf);
        
        [u_i_opt,~,~,~]= fmincon(@Seed_Ctrl_Obj,u_seed_i,[],[],[],[],Ctrl_LowBd,Ctrl_UppBd,@Seed_Ctrl_Constraint,Seed_Sub_Opt, P);
        u_i = u_i_opt;
    end  
    u_array = [u_array u_i];
end
StateNDot_Seed = reshape(x_intp, length(C_q_qdot)*2*Ctrl_No, 1);
Ctrl_Seed = reshape(u_array, length(u_i) *Ctrl_No,1 );
Opt_Seed = [Tme_Seed; StateNDot_Seed; Ctrl_Seed; ContactForce_tot];

Opt_Lowbd = 0.105;
Opt_Uppbd = inf; 

Opt_Lowbd = [Opt_Lowbd; repmat(RobotState_LowBd', [Ctrl_No,1]); repmat(Ctrl_LowBd, [Ctrl_No,1]); repmat(ContactForce_LowBd, [Ctrl_No, 1])];
Opt_Uppbd = [Opt_Uppbd; repmat(RobotState_UppBd', [Ctrl_No,1]); repmat(Ctrl_UppBd, [Ctrl_No,1]); repmat(ContactForce_UppBd, [Ctrl_No, 1])];

end