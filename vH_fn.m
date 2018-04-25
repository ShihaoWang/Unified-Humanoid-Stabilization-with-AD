function vH = vH_fn(q1,q1dot,rIxdot,rIydot,thetadot,theta)
%VH_FN
%    VH = VH_FN(Q1,Q1DOT,RIXDOT,RIYDOT,THETADOT,THETA)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Apr-2018 15:37:17

t2 = pi.*(1.0./2.0);
t3 = q1+t2+theta;
t4 = sin(t3);
t5 = cos(t3);
vH = [rIxdot-q1dot.*t4.*(1.3e1./4.0e1)-t4.*thetadot.*(1.3e1./4.0e1);rIydot-q1dot.*t5.*(1.3e1./4.0e1)-t5.*thetadot.*(1.3e1./4.0e1)];
