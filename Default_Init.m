function [sigma0, x2] = Default_Init(varargin)

% This function is used to given a default initialization state
global sigma0
sigma0 = [ 0 1 0 0 ]; 
rIx = 0;            rIy = 1;            theta = -0.09;
q1 = 0.45;          q2 = 0.06;          q3 = 0.15;
% q4 = -0.6;
q4 = -1;            q5 = 0.15;          q6 = 0.27;
q7 = -0.66;         q8 = -0.6251;       q9 = 0.69;          q10 = -0.2951;

rIxdot = 0.2;       rIydot = 0.1;       thetadot = -0.21;
q1dot = 0.733;      q2dot = 1.5;        q3dot = 0.733;
q4dot = -1.5;       q5dot = 2;          q6dot = -1.3;
q7dot = 1;          q8dot = -2;         q9dot = -pi/2;      q10dot = -1.5;

global x0_init
x0_init = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10,...
           rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]';
  
Init_Opt = optimoptions(@fmincon,'Display','off','Algorithm','sqp','Maxiterations',inf,'MaxFunctionEvaluations',inf);
[RobotState_LowBd, RobotState_UppBd, ~, ~] = Optimization_Bounds();

Grad_Hess_Flag = 0;

if Grad_Hess_Flag == 1
    % --------------------- Call adigatorGenFiles4Fmincon ------------------- %
    setup.order = 2;
    setup.numvar = length(x0_init);
    setup.objective  = 'Init_Obj';
    setup.constraint = 'Init_Cons';   
    adifuncs = adigatorGenFiles4Fmincon(setup);
end
% ------------------- Solve Without Derivatives ----------------------- %
tic;
[x0,fval0] = ...
    fmincon(@Init_Obj,x0_init,[],[],[],[],RobotState_LowBd,RobotState_UppBd,@Init_Cons,Init_Opt);
time0 = toc;

% ------------------- Solve With 1st Derivatives ---------------------- %
tic;
options = optimset('Algorithm','sqp');
options = optimset(options,'GradObj','on','GradConstr','on','Display','off');
[x1,fval1] = fmincon(@Init_Obj_Grd,x0_init,[],[],[],[],RobotState_LowBd,RobotState_UppBd,...
    @Init_Cons_Grd,options);
time1 = toc;

% ------------------- Solve With 2nd Derivatives ---------------------- %
tic;
options = optimset('Algorithm','sqp',...
    'Display','off','GradObj','on','GradConstr','on',...
    'Hessian','user-supplied','HessFcn',@Init_Obj_Hes);
[x2,fval2] = fmincon(@Init_Obj_Grd,x0_init,[],[],[],[],RobotState_LowBd,RobotState_UppBd,...
    @Init_Cons_Grd,options);
time2 = toc;

% -------------------- Display Solve Times ---------------------------- %
fprintf(['Solve time using no derivatives',...
    ':  ',num2str(time0),'\n'])
fprintf(['Solve time using 1st derivatives',...
    ': ',num2str(time1),'\n'])
fprintf(['Solve time using 2nd derivatives',...
    ': ',num2str(time2),'\n'])
if nargin >0 
    Single_Frame_Plot(x2)
end
end

