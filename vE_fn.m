function vE = vE_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta)
%VE_FN
%    VE = VE_FN(Q7,Q8,Q7DOT,Q8DOT,RIXDOT,RIYDOT,THETADOT,THETA)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Apr-2018 15:37:16

t2 = pi.*(1.0./2.0);
t3 = q7+t2+theta;
t4 = sin(t3);
t5 = t4.*(1.0./4.0);
t6 = q7+q8+t2+theta;
t7 = sin(t6);
t8 = t7.*(9.0./2.0e1);
t9 = cos(t3);
t10 = t9.*(1.0./4.0);
t11 = cos(t6);
t12 = t11.*(9.0./2.0e1);
t13 = -t2+theta;
vE = [rIxdot-q7dot.*(t5+t8)-q8dot.*t7.*(9.0./2.0e1)-thetadot.*(t5+t8+sin(t13).*(1.1e1./2.0e1));rIydot-q7dot.*(t10+t12)-q8dot.*t11.*(9.0./2.0e1)-thetadot.*(t10+t12+cos(t13).*(1.1e1./2.0e1))];
