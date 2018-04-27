function value = Kinetic_Energy_Cal(RobotState)

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

value = T_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);

end

