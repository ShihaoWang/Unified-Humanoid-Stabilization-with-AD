function Obj = Seed_Conf_Obj(z, P)
rIx = z(1);             rIy = z(2);             theta = z(3);
q1 = z(4);              q2 = z(5);              q3 = z(6);
q4 = z(7);              q5 = z(8);              q6 = z(9);
q7 = z(10);             q8 = z(11);             q9 = z(12);
q10 = z(13);
rIxdot = z(1+13);          rIydot = z(2+13);          thetadot = z(3+13);
q1dot = z(4+13);           q2dot = z(5+13);           q3dot = z(6+13);
q4dot = z(7+13);           q5dot = z(8+13);           q6dot = z(9+13);
q7dot = z(10+13);          q8dot = z(11+13);          q9dot = z(12+13);
q10dot = z(13+13);

T = P.T_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);
Obj = T;
end