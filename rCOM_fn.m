function rCOM = rCOM_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,rIx,rIy,theta)
%RCOM_FN
%    RCOM = RCOM_FN(Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,RIX,RIY,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    09-May-2018 22:13:26

t2 = sqrt(2.0);
t3 = pi.*(1.0./4.0);
t4 = sqrt(4.1e1);
t5 = q1+q2+theta;
t6 = q4+q5+theta;
t7 = q7+q8+theta;
t8 = q9+q10+theta;
t9 = q1+theta;
t10 = q4+theta;
t11 = q7+theta;
t12 = q9+theta;
t13 = q1+q2+q3+theta-8.960553845713439e-1;
t14 = q4+q5+q6+theta-8.960553845713439e-1;
t15 = q1+q2+q3+t3+theta;
t16 = q4+q5+q6+t3+theta;
rCOM = [rIx-sin(t5).*1.678966789667897e-2-sin(t6).*1.678966789667897e-2-sin(t7).*8.717712177121771e-3-sin(t8).*8.717712177121771e-3-sin(t9).*3.597785977859779e-2-sin(t10).*3.597785977859779e-2-sin(t11).*1.775830258302583e-2-sin(t12).*1.775830258302583e-2+sin(theta).*2.506457564575646e-1-t2.*sin(t15).*1.476014760147601e-3-t4.*sin(t13).*3.690036900369004e-4-t2.*sin(t16).*1.476014760147601e-3-t4.*sin(t14).*3.690036900369004e-4;rIy-cos(t5).*1.678966789667897e-2-cos(t6).*1.678966789667897e-2-cos(t7).*8.717712177121771e-3-cos(t8).*8.717712177121771e-3-cos(t9).*3.597785977859779e-2-cos(t10).*3.597785977859779e-2-cos(t11).*1.775830258302583e-2-cos(t12).*1.775830258302583e-2+cos(theta).*2.506457564575646e-1-t2.*cos(t15).*1.476014760147601e-3-t4.*cos(t13).*3.690036900369004e-4-t2.*cos(t16).*1.476014760147601e-3-t4.*cos(t14).*3.690036900369004e-4];
