function vCOM = vCOM_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta)
%VCOM_FN
%    VCOM = VCOM_FN(Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q10DOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,RIXDOT,RIYDOT,THETADOT,THETA)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    26-Apr-2018 12:48:27

t2 = q1+q2+q3+theta;
t3 = cos(t2);
t4 = q4+q5+q6+theta;
t5 = cos(t4);
t6 = sin(t2);
t7 = sin(t4);
t8 = q1+theta;
t9 = cos(t8);
t10 = q4+theta;
t11 = cos(t10);
t12 = q7+theta;
t13 = cos(t12);
t14 = q9+theta;
t15 = cos(t14);
t16 = q1+q2+theta;
t17 = cos(t16);
t18 = q4+q5+theta;
t19 = cos(t18);
t20 = q7+q8+theta;
t21 = cos(t20);
t22 = q9+q10+theta;
t23 = cos(t22);
t24 = sqrt(4.1e1);
t25 = q1+q2+q3+theta-8.960553845713439e-1;
t26 = cos(t25);
t27 = q4+q5+q6+theta-8.960553845713439e-1;
t28 = cos(t27);
t29 = q1dot.*t6.*1.476014760147601e-3;
t30 = q2dot.*t6.*1.476014760147601e-3;
t31 = q3dot.*t6.*1.476014760147601e-3;
t32 = q4dot.*t7.*1.476014760147601e-3;
t33 = q5dot.*t7.*1.476014760147601e-3;
t34 = q6dot.*t7.*1.476014760147601e-3;
t35 = t6.*thetadot.*1.476014760147601e-3;
t36 = t7.*thetadot.*1.476014760147601e-3;
t37 = sin(t8);
t38 = sin(t10);
t39 = sin(t12);
t40 = sin(t14);
t41 = sin(t16);
t42 = sin(t18);
t43 = sin(t20);
t44 = sin(t22);
t45 = sin(t25);
t46 = sin(t27);
vCOM = [rIxdot+t29+t30+t31+t32+t33+t34+t35+t36-q10dot.*t23.*8.717712177121771e-3-q1dot.*t3.*1.476014760147601e-3-q2dot.*t3.*1.476014760147601e-3-q3dot.*t3.*1.476014760147601e-3-q4dot.*t5.*1.476014760147601e-3-q1dot.*t9.*3.597785977859779e-2-q5dot.*t5.*1.476014760147601e-3-q6dot.*t5.*1.476014760147601e-3-q4dot.*t11.*3.597785977859779e-2-q1dot.*t17.*1.678966789667897e-2-q2dot.*t17.*1.678966789667897e-2-q7dot.*t13.*1.775830258302583e-2-q4dot.*t19.*1.678966789667897e-2-q5dot.*t19.*1.678966789667897e-2-q9dot.*t15.*1.775830258302583e-2-q7dot.*t21.*8.717712177121771e-3-q8dot.*t21.*8.717712177121771e-3-q9dot.*t23.*8.717712177121771e-3-t3.*thetadot.*1.476014760147601e-3-t5.*thetadot.*1.476014760147601e-3-t9.*thetadot.*3.597785977859779e-2-t11.*thetadot.*3.597785977859779e-2-t13.*thetadot.*1.775830258302583e-2-t15.*thetadot.*1.775830258302583e-2-t17.*thetadot.*1.678966789667897e-2-t19.*thetadot.*1.678966789667897e-2-t21.*thetadot.*8.717712177121771e-3-t23.*thetadot.*8.717712177121771e-3+thetadot.*cos(theta).*2.506457564575646e-1-q1dot.*t24.*t26.*3.690036900369004e-4-q2dot.*t24.*t26.*3.690036900369004e-4-q3dot.*t24.*t26.*3.690036900369004e-4-q4dot.*t24.*t28.*3.690036900369004e-4-q5dot.*t24.*t28.*3.690036900369004e-4-q6dot.*t24.*t28.*3.690036900369004e-4-t24.*t26.*thetadot.*3.690036900369004e-4-t24.*t28.*thetadot.*3.690036900369004e-4;rIydot+t29+t30+t31+t32+t33+t34+t35+t36+q10dot.*t44.*8.717712177121771e-3+q1dot.*t3.*1.476014760147601e-3+q2dot.*t3.*1.476014760147601e-3+q3dot.*t3.*1.476014760147601e-3+q4dot.*t5.*1.476014760147601e-3+q5dot.*t5.*1.476014760147601e-3+q6dot.*t5.*1.476014760147601e-3+q1dot.*t37.*3.597785977859779e-2+q1dot.*t41.*1.678966789667897e-2+q4dot.*t38.*3.597785977859779e-2+q2dot.*t41.*1.678966789667897e-2+q4dot.*t42.*1.678966789667897e-2+q7dot.*t39.*1.775830258302583e-2+q5dot.*t42.*1.678966789667897e-2+q9dot.*t40.*1.775830258302583e-2+q7dot.*t43.*8.717712177121771e-3+q8dot.*t43.*8.717712177121771e-3+q9dot.*t44.*8.717712177121771e-3+t3.*thetadot.*1.476014760147601e-3+t5.*thetadot.*1.476014760147601e-3+t37.*thetadot.*3.597785977859779e-2+t38.*thetadot.*3.597785977859779e-2+t39.*thetadot.*1.775830258302583e-2+t40.*thetadot.*1.775830258302583e-2+t41.*thetadot.*1.678966789667897e-2+t42.*thetadot.*1.678966789667897e-2+t43.*thetadot.*8.717712177121771e-3+t44.*thetadot.*8.717712177121771e-3-thetadot.*sin(theta).*2.506457564575646e-1+q1dot.*t24.*t45.*3.690036900369004e-4+q2dot.*t24.*t45.*3.690036900369004e-4+q3dot.*t24.*t45.*3.690036900369004e-4+q4dot.*t24.*t46.*3.690036900369004e-4+q5dot.*t24.*t46.*3.690036900369004e-4+q6dot.*t24.*t46.*3.690036900369004e-4+t24.*t45.*thetadot.*3.690036900369004e-4+t24.*t46.*thetadot.*3.690036900369004e-4];