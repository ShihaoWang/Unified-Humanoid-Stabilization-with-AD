function [RobotState_LowBd, RobotState_UppBd, Ctrl_LowBd, Ctrl_UppBd] = Optimization_Bounds()

%% 1. Robot state bounds
rIxlow = -Inf;                  rIxupp = Inf;
rIylow = -Inf;                  rIyupp = Inf;
thetalow = -pi;                 thetaupp = pi;
q1low = -2.1817;                q1upp = 0.733;
q2low = 0.0;                    q2upp = 2.618;
q3low = -1.3;                   q3upp = 0.733;
q4low = -2.1817;                q4upp = 0.733;
q5low = 0.0;                    q5upp = 2.618;
q6low = -1.3;                   q6upp = 0.733;
q7low = -3.14;                  q7upp = 1.047;
q8low = -2.391;                 q8upp = 0.0;
q9low = -3.14;                  q9upp = 1.047;
q10low = -2.391;                q10upp = 0.0;

AngRateLow = -3.0;              AngRateHgh = 3.0;

rIxdotlow = -Inf;               rIxdotupp = Inf;
rIydotlow = -Inf;               rIydotupp = Inf;
thetadotlow = -Inf;             thetadotupp = Inf;
q1dotlow = AngRateLow;          q1dotupp = AngRateHgh;
q2dotlow = AngRateLow;          q2dotupp = AngRateHgh;
q3dotlow = AngRateLow;          q3dotupp = AngRateHgh;
q4dotlow = AngRateLow;          q4dotupp = AngRateHgh;
q5dotlow = AngRateLow;          q5dotupp = AngRateHgh;
q6dotlow = AngRateLow;          q6dotupp = AngRateHgh;
q7dotlow = AngRateLow;          q7dotupp = AngRateHgh;
q8dotlow = AngRateLow;          q8dotupp = AngRateHgh;
q9dotlow = AngRateLow;          q9dotupp = AngRateHgh;
q10dotlow = AngRateLow;         q10dotupp = AngRateHgh;

RobotState_LowBd = [rIxlow rIylow thetalow q1low q2low q3low q4low q5low q6low q7low q8low q9low q10low...
                    rIxdotlow rIydotlow thetadotlow q1dotlow q2dotlow q3dotlow q4dotlow q5dotlow q6dotlow q7dotlow q8dotlow q9dotlow q10dotlow];
RobotState_UppBd = [rIxupp rIyupp thetaupp q1upp q2upp q3upp q4upp q5upp q6upp q7upp q8upp q9upp q10upp...
                    rIxdotupp rIydotupp thetadotupp q1dotupp q2dotupp q3dotupp q4dotupp q5dotupp q6dotupp q7dotupp q8dotupp q9dotupp q10dotupp];

%% 2. Upper and Lower bounds of the control variables                    
tau1_max = 100;             tau2_max = 100;             tau3_max = 100;
tau4_max = 100;             tau5_max = 100;             tau6_max = 100;
tau7_max = 60;              tau8_max = 50;              tau9_max = 60;             tau10_max = 50;

Ctrl_LowBd = -[tau1_max tau2_max tau3_max tau4_max tau5_max tau6_max tau7_max tau8_max tau9_max tau10_max ]';
Ctrl_UppBd = - Ctrl_LowBd;

end

