function rB = rB_fn(q1,q2,q3,rIx,rIy,theta)
%RB_FN
%    RB = RB_FN(Q1,Q2,Q3,RIX,RIY,THETA)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    26-Apr-2018 12:50:51

t2 = pi.*(1.0./2.0);
t3 = q1+t2+theta;
t4 = q1+q2+t2+theta;
t5 = sqrt(2.0);
t6 = q1+q2+q3+theta-pi.*(5.0./4.0);
rB = [rIx+cos(t3).*(1.3e1./4.0e1)+cos(t4).*(1.3e1./4.0e1)+t5.*cos(t6).*(1.0./1.0e1);rIy-sin(t3).*(1.3e1./4.0e1)-sin(t4).*(1.3e1./4.0e1)-t5.*sin(t6).*(1.0./1.0e1)];
