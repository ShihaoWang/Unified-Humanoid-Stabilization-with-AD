function [sigma0, x0] = Default_Init(varargin)

% This function is used to given a default initialization state
global sigma0
sigma0 = [ 1 1 0 0 ]; 
rIx = 1;            rIy = 1;            theta = 0;
q1 = 0.65;           q2 = 0.0001;             q3 = -0.3;
% q4 = -0.6;
q4 = -0.45;            q5 = 0.5;          q6 = 0.27;
q7 = -0.66;         q8 = -0.6251;       q9 = 0.69;          q10 = -0.2951;

rIxdot = 5.5;       rIydot = 0.1;       thetadot = -3.5;
q1dot = 0.733;      q2dot = 2.5;        q3dot = 0.733;
q4dot = -0.5;       q5dot = .3;          q6dot = -.3;
q7dot = 3;          q8dot = -3;         q9dot = -3;      q10dot = -3;

global x0_init
x0_init = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10,...
           rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]';

       
Init_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','Maxiterations',inf,'MaxFunctionEvaluations',inf);
[RobotState_LowBd, RobotState_UppBd, ~, ~] = Optimization_Bounds();

tic;
[x0,fval0] = ...
    fmincon(@Init_Obj,x0_init,[],[],[],[],RobotState_LowBd,RobotState_UppBd,@Init_Cons,Init_Opt);
time0 = toc;

if nargin >0 
    Single_Frame_Plot(x0)
end
end

