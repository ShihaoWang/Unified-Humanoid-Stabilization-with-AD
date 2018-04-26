function [End_Pos, End_Vel]= End_Effector_Pos_Vel(RobotState)
% This function is used to calculate the position of the end effector

rIx = RobotState(1);             rIy = RobotState(2);             theta = RobotState(3);
q1 = RobotState(4);              q2 = RobotState(5);              q3 = RobotState(6);
q4 = RobotState(7);              q5 = RobotState(8);              q6 = RobotState(9);
q7 = RobotState(10);             q8 = RobotState(11);             q9 = RobotState(12);
q10 = RobotState(13);

rIxdot = RobotState(1+13);          rIydot = RobotState(2+13);          thetadot = RobotState(3+13);
q1dot = RobotState(4+13);           q2dot = RobotState(5+13);           q3dot = RobotState(6+13);
q4dot = RobotState(7+13);           q5dot = RobotState(8+13);           q6dot = RobotState(9+13);
q7dot = RobotState(10+13);          q8dot = RobotState(11+13);          q9dot = RobotState(12+13);
q10dot = RobotState(13+13);

rA = rA_fn(q1,q2,q3,rIx,rIy,theta);
rB = rB_fn(q1,q2,q3,rIx,rIy,theta);
rC = rC_fn(q4,q5,q6,rIx,rIy,theta);
rD = rD_fn(q4,q5,q6,rIx,rIy,theta);
rE = rE_fn(q7,q8,rIx,rIy,theta);
rF = rF_fn(q9,q10,rIx,rIy,theta);
rT = rT_fn(rIx,rIy,theta);
rCOM = rCOM_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,rIx,rIy,theta);

End_Pos = [rA'; rB'; rC'; rD'; rE'; rF'; rT'; rCOM'];

vA = vA_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vB = vB_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta); 
vC = vC_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vD = vD_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vE = vE_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
vF = vF_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
vT = vT_fn(rIxdot,rIydot,thetadot,theta); 
vCOM = vCOM_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);
 
End_Vel = [vA'; vB'; vC'; vD'; vE'; vF'; vT'; vCOM'];
end

