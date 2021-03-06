function Tdot = Tdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta)
%TDOT_FN
%    TDOT = TDOT_FN(Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q10DOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,RIXDOT,RIYDOT,THETADOT,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    09-May-2018 22:17:51

t2 = q1+q2+q3+theta;
t3 = cos(t2);
t4 = q4+q5+q6+theta;
t5 = cos(t4);
t6 = q1dot.^2;
t7 = q2dot.^2;
t8 = q3dot.^2;
t9 = q4dot.^2;
t10 = q5dot.^2;
t11 = q6dot.^2;
t12 = thetadot.^2;
t13 = sin(t2);
t14 = sin(t4);
t15 = q2+q3;
t16 = cos(t15);
t17 = q5+q6;
t18 = cos(t17);
t19 = q1+theta;
t20 = cos(t19);
t21 = q4+theta;
t22 = cos(t21);
t23 = q7+theta;
t24 = cos(t23);
t25 = q9+theta;
t26 = cos(t25);
t27 = sin(t15);
t28 = sin(t17);
t29 = q9+q10;
t30 = sin(t29);
t31 = q7dot.^2;
t32 = q7+q8;
t33 = sin(t32);
t34 = q9dot.^2;
t35 = sin(t19);
t36 = sin(t21);
t37 = sin(t23);
t38 = sin(t25);
t39 = cos(q3);
t40 = cos(q6);
t41 = q10dot.^2;
t42 = sin(q10);
t43 = sin(q2);
t44 = sin(q3);
t45 = sin(q5);
t46 = sin(q6);
t47 = q8dot.^2;
t48 = sin(q8);
t49 = sin(q7);
t50 = sin(q9);
t51 = q1+q2+theta;
t52 = cos(t51);
t53 = q4+q5+theta;
t54 = cos(t53);
t55 = q7+q8+theta;
t56 = cos(t55);
t57 = q9+q10+theta;
t58 = cos(t57);
t59 = sin(t51);
t60 = sin(t53);
t61 = sin(t55);
t62 = sin(t57);
t63 = sqrt(4.1e1);
t64 = q3-8.960553845713439e-1;
t65 = sin(t64);
t66 = q6-8.960553845713439e-1;
t67 = sin(t66);
t68 = q1+q2+q3+theta-8.960553845713439e-1;
t69 = cos(t68);
t70 = q4+q5+q6+theta-8.960553845713439e-1;
t71 = cos(t70);
t72 = sin(t68);
t73 = sin(t70);
t74 = q2+q3-8.960553845713439e-1;
t75 = sin(t74);
t76 = q5+q6-8.960553845713439e-1;
t77 = sin(t76);
Tdot = rIydot.*t12.*cos(theta).*(-1.3585e1)-rIxdot.*t12.*sin(theta).*1.3585e1+q10dot.*t12.*t30.*4.33125e-1-q10dot.*t12.*t42.*(6.3e1./3.2e2)-q10dot.*t34.*t42.*(6.3e1./3.2e2)-q1dot.*t7.*t16.*(1.3e1./5.0e2)-q2dot.*t6.*t16.*(1.3e1./5.0e2)-q1dot.*t8.*t16.*(1.3e1./5.0e2)-q3dot.*t6.*t16.*(1.3e1./5.0e2)-q2dot.*t12.*t16.*(1.3e1./5.0e2)-q3dot.*t12.*t16.*(1.3e1./5.0e2)-q4dot.*t10.*t18.*(1.3e1./5.0e2)-q5dot.*t9.*t18.*(1.3e1./5.0e2)-q4dot.*t11.*t18.*(1.3e1./5.0e2)-q6dot.*t9.*t18.*(1.3e1./5.0e2)-q1dot.*t7.*t27.*(1.3e1./5.0e2)-q2dot.*t6.*t27.*(1.3e1./5.0e2)-q5dot.*t12.*t18.*(1.3e1./5.0e2)-q1dot.*t8.*t27.*(1.3e1./5.0e2)-q3dot.*t6.*t27.*(1.3e1./5.0e2)-q6dot.*t12.*t18.*(1.3e1./5.0e2)-q2dot.*t12.*t27.*(1.3e1./5.0e2)-q3dot.*t12.*t27.*(1.3e1./5.0e2)-q4dot.*t10.*t28.*(1.3e1./5.0e2)-q5dot.*t9.*t28.*(1.3e1./5.0e2)-q4dot.*t11.*t28.*(1.3e1./5.0e2)-q6dot.*t9.*t28.*(1.3e1./5.0e2)-q5dot.*t12.*t28.*(1.3e1./5.0e2)-q6dot.*t12.*t28.*(1.3e1./5.0e2)-q1dot.*t8.*t39.*(1.3e1./5.0e2)-q3dot.*t6.*t39.*(1.3e1./5.0e2)-q2dot.*t8.*t39.*(1.3e1./5.0e2)-q3dot.*t7.*t39.*(1.3e1./5.0e2)-q1dot.*t7.*t43.*2.9575e-1-q2dot.*t6.*t43.*2.9575e-1+q9dot.*t12.*t30.*4.33125e-1+q7dot.*t12.*t33.*4.33125e-1-q1dot.*t8.*t44.*(1.3e1./5.0e2)-q3dot.*t6.*t44.*(1.3e1./5.0e2)+q8dot.*t12.*t33.*4.33125e-1-q2dot.*t8.*t44.*(1.3e1./5.0e2)-q3dot.*t7.*t44.*(1.3e1./5.0e2)-q3dot.*t12.*t39.*(1.3e1./5.0e2)-q4dot.*t11.*t40.*(1.3e1./5.0e2)-q6dot.*t9.*t40.*(1.3e1./5.0e2)-q5dot.*t11.*t40.*(1.3e1./5.0e2)-q6dot.*t10.*t40.*(1.3e1./5.0e2)-q2dot.*t12.*t43.*2.9575e-1-q6dot.*t12.*t40.*(1.3e1./5.0e2)-q3dot.*t12.*t44.*(1.3e1./5.0e2)-q4dot.*t10.*t45.*2.9575e-1-q5dot.*t9.*t45.*2.9575e-1-q4dot.*t11.*t46.*(1.3e1./5.0e2)-q6dot.*t9.*t46.*(1.3e1./5.0e2)-q5dot.*t11.*t46.*(1.3e1./5.0e2)-q5dot.*t12.*t45.*2.9575e-1-q6dot.*t10.*t46.*(1.3e1./5.0e2)-q6dot.*t12.*t46.*(1.3e1./5.0e2)+q7dot.*t12.*t49.*6.25625e-1-q8dot.*t12.*t48.*(6.3e1./3.2e2)+q9dot.*t12.*t50.*6.25625e-1-q8dot.*t31.*t48.*(6.3e1./3.2e2)-q9dot.*t41.*t42.*(6.3e1./3.2e2)-q7dot.*t47.*t48.*(6.3e1./3.2e2)+rIxdot.*t3.*t6.*(2.0./2.5e1)+rIxdot.*t3.*t7.*(2.0./2.5e1)+rIxdot.*t3.*t8.*(2.0./2.5e1)+rIxdot.*t5.*t9.*(2.0./2.5e1)+rIxdot.*t3.*t12.*(2.0./2.5e1)+rIxdot.*t5.*t10.*(2.0./2.5e1)+rIxdot.*t5.*t11.*(2.0./2.5e1)+rIxdot.*t5.*t12.*(2.0./2.5e1)+rIxdot.*t6.*t13.*(2.0./2.5e1)+rIxdot.*t7.*t13.*(2.0./2.5e1)+rIxdot.*t8.*t13.*(2.0./2.5e1)+rIxdot.*t9.*t14.*(2.0./2.5e1)+rIxdot.*t10.*t14.*(2.0./2.5e1)+rIxdot.*t11.*t14.*(2.0./2.5e1)+rIxdot.*t12.*t13.*(2.0./2.5e1)+rIxdot.*t12.*t14.*(2.0./2.5e1)+rIxdot.*t6.*t35.*(3.9e1./2.0e1)+rIxdot.*t9.*t36.*(3.9e1./2.0e1)+rIxdot.*t12.*t35.*(3.9e1./2.0e1)+rIxdot.*t12.*t36.*(3.9e1./2.0e1)+rIxdot.*t12.*t37.*(9.1e1./8.0e1)+rIxdot.*t12.*t38.*(9.1e1./8.0e1)+rIxdot.*t6.*t59.*(9.1e1./1.0e2)+rIxdot.*t7.*t59.*(9.1e1./1.0e2)+rIxdot.*t31.*t37.*(9.1e1./8.0e1)+rIxdot.*t9.*t60.*(9.1e1./1.0e2)+rIxdot.*t10.*t60.*(9.1e1./1.0e2)+rIxdot.*t12.*t59.*(9.1e1./1.0e2)+rIxdot.*t12.*t60.*(9.1e1./1.0e2)+rIxdot.*t34.*t38.*(9.1e1./8.0e1)+rIxdot.*t12.*t61.*(6.3e1./8.0e1)+rIxdot.*t12.*t62.*(6.3e1./8.0e1)+rIxdot.*t31.*t61.*(6.3e1./8.0e1)+rIxdot.*t34.*t62.*(6.3e1./8.0e1)+rIxdot.*t41.*t62.*(6.3e1./8.0e1)+rIxdot.*t47.*t61.*(6.3e1./8.0e1)+rIydot.*t3.*t6.*(2.0./2.5e1)+rIydot.*t3.*t7.*(2.0./2.5e1)+rIydot.*t3.*t8.*(2.0./2.5e1)+rIydot.*t5.*t9.*(2.0./2.5e1)+rIydot.*t3.*t12.*(2.0./2.5e1)+rIydot.*t5.*t10.*(2.0./2.5e1)+rIydot.*t5.*t11.*(2.0./2.5e1)+rIydot.*t5.*t12.*(2.0./2.5e1)-rIydot.*t6.*t13.*(2.0./2.5e1)-rIydot.*t7.*t13.*(2.0./2.5e1)-rIydot.*t8.*t13.*(2.0./2.5e1)-rIydot.*t9.*t14.*(2.0./2.5e1)-rIydot.*t10.*t14.*(2.0./2.5e1)-rIydot.*t11.*t14.*(2.0./2.5e1)-rIydot.*t12.*t13.*(2.0./2.5e1)+rIydot.*t6.*t20.*(3.9e1./2.0e1)-rIydot.*t12.*t14.*(2.0./2.5e1)+rIydot.*t9.*t22.*(3.9e1./2.0e1)+rIydot.*t12.*t20.*(3.9e1./2.0e1)+rIydot.*t12.*t22.*(3.9e1./2.0e1)+rIydot.*t12.*t24.*(9.1e1./8.0e1)+rIydot.*t12.*t26.*(9.1e1./8.0e1)+rIydot.*t24.*t31.*(9.1e1./8.0e1)+rIydot.*t6.*t52.*(9.1e1./1.0e2)+rIydot.*t7.*t52.*(9.1e1./1.0e2)+rIydot.*t26.*t34.*(9.1e1./8.0e1)+rIydot.*t9.*t54.*(9.1e1./1.0e2)+rIydot.*t10.*t54.*(9.1e1./1.0e2)+rIydot.*t12.*t52.*(9.1e1./1.0e2)+rIydot.*t12.*t54.*(9.1e1./1.0e2)+rIydot.*t12.*t56.*(6.3e1./8.0e1)+rIydot.*t12.*t58.*(6.3e1./8.0e1)+rIydot.*t31.*t56.*(6.3e1./8.0e1)+rIydot.*t34.*t58.*(6.3e1./8.0e1)+rIydot.*t41.*t58.*(6.3e1./8.0e1)+rIydot.*t47.*t56.*(6.3e1./8.0e1)-t7.*t16.*thetadot.*(1.3e1./5.0e2)-t8.*t16.*thetadot.*(1.3e1./5.0e2)-t10.*t18.*thetadot.*(1.3e1./5.0e2)-t11.*t18.*thetadot.*(1.3e1./5.0e2)-t7.*t27.*thetadot.*(1.3e1./5.0e2)-t8.*t27.*thetadot.*(1.3e1./5.0e2)-t10.*t28.*thetadot.*(1.3e1./5.0e2)-t11.*t28.*thetadot.*(1.3e1./5.0e2)-t8.*t39.*thetadot.*(1.3e1./5.0e2)-t7.*t43.*thetadot.*2.9575e-1-t11.*t40.*thetadot.*(1.3e1./5.0e2)-t8.*t44.*thetadot.*(1.3e1./5.0e2)-t10.*t45.*thetadot.*2.9575e-1-t11.*t46.*thetadot.*(1.3e1./5.0e2)+t30.*t34.*thetadot.*4.33125e-1+t31.*t33.*thetadot.*4.33125e-1+t30.*t41.*thetadot.*4.33125e-1+t31.*t49.*thetadot.*6.25625e-1+t33.*t47.*thetadot.*4.33125e-1-t41.*t42.*thetadot.*(6.3e1./3.2e2)+t34.*t50.*thetadot.*6.25625e-1-t47.*t48.*thetadot.*(6.3e1./3.2e2)-q1dot.*q2dot.*q3dot.*t16.*(1.3e1./2.5e2)-q1dot.*q2dot.*q3dot.*t27.*(1.3e1./2.5e2)-q4dot.*q5dot.*q6dot.*t18.*(1.3e1./2.5e2)-q4dot.*q5dot.*q6dot.*t28.*(1.3e1./2.5e2)-q1dot.*q2dot.*q3dot.*t39.*(1.3e1./2.5e2)-q1dot.*q2dot.*q3dot.*t44.*(1.3e1./2.5e2)-q4dot.*q5dot.*q6dot.*t40.*(1.3e1./2.5e2)-q4dot.*q5dot.*q6dot.*t46.*(1.3e1./2.5e2)+q10dot.*q9dot.*rIxdot.*t62.*(6.3e1./4.0e1)+q10dot.*q9dot.*rIydot.*t58.*(6.3e1./4.0e1)+q1dot.*q2dot.*rIxdot.*t3.*(4.0./2.5e1)+q1dot.*q3dot.*rIxdot.*t3.*(4.0./2.5e1)+q2dot.*q3dot.*rIxdot.*t3.*(4.0./2.5e1)+q4dot.*q5dot.*rIxdot.*t5.*(4.0./2.5e1)+q4dot.*q6dot.*rIxdot.*t5.*(4.0./2.5e1)+q1dot.*q2dot.*rIxdot.*t13.*(4.0./2.5e1)+q5dot.*q6dot.*rIxdot.*t5.*(4.0./2.5e1)+q1dot.*q3dot.*rIxdot.*t13.*(4.0./2.5e1)+q2dot.*q3dot.*rIxdot.*t13.*(4.0./2.5e1)+q4dot.*q5dot.*rIxdot.*t14.*(4.0./2.5e1)+q4dot.*q6dot.*rIxdot.*t14.*(4.0./2.5e1)+q5dot.*q6dot.*rIxdot.*t14.*(4.0./2.5e1)+q1dot.*q2dot.*rIxdot.*t59.*(9.1e1./5.0e1)+q4dot.*q5dot.*rIxdot.*t60.*(9.1e1./5.0e1)+q7dot.*q8dot.*rIxdot.*t61.*(6.3e1./4.0e1)+q1dot.*q2dot.*rIydot.*t3.*(4.0./2.5e1)+q1dot.*q3dot.*rIydot.*t3.*(4.0./2.5e1)+q2dot.*q3dot.*rIydot.*t3.*(4.0./2.5e1)+q4dot.*q5dot.*rIydot.*t5.*(4.0./2.5e1)+q4dot.*q6dot.*rIydot.*t5.*(4.0./2.5e1)-q1dot.*q2dot.*rIydot.*t13.*(4.0./2.5e1)+q5dot.*q6dot.*rIydot.*t5.*(4.0./2.5e1)-q1dot.*q3dot.*rIydot.*t13.*(4.0./2.5e1)-q2dot.*q3dot.*rIydot.*t13.*(4.0./2.5e1)-q4dot.*q5dot.*rIydot.*t14.*(4.0./2.5e1)-q4dot.*q6dot.*rIydot.*t14.*(4.0./2.5e1)-q5dot.*q6dot.*rIydot.*t14.*(4.0./2.5e1)+q1dot.*q2dot.*rIydot.*t52.*(9.1e1./5.0e1)+q4dot.*q5dot.*rIydot.*t54.*(9.1e1./5.0e1)+q7dot.*q8dot.*rIydot.*t56.*(6.3e1./4.0e1)+q10dot.*q9dot.*t30.*thetadot.*(6.93e2./8.0e2)-q10dot.*q9dot.*t42.*thetadot.*(6.3e1./1.6e2)-q1dot.*q2dot.*t16.*thetadot.*(1.3e1./2.5e2)-q1dot.*q3dot.*t16.*thetadot.*(1.3e1./2.5e2)-q2dot.*q3dot.*t16.*thetadot.*(1.3e1./2.5e2)-q4dot.*q5dot.*t18.*thetadot.*(1.3e1./2.5e2)-q4dot.*q6dot.*t18.*thetadot.*(1.3e1./2.5e2)-q5dot.*q6dot.*t18.*thetadot.*(1.3e1./2.5e2)-q1dot.*q2dot.*t27.*thetadot.*(1.3e1./2.5e2)-q1dot.*q3dot.*t27.*thetadot.*(1.3e1./2.5e2)-q2dot.*q3dot.*t27.*thetadot.*(1.3e1./2.5e2)-q4dot.*q5dot.*t28.*thetadot.*(1.3e1./2.5e2)-q4dot.*q6dot.*t28.*thetadot.*(1.3e1./2.5e2)-q5dot.*q6dot.*t28.*thetadot.*(1.3e1./2.5e2)-q1dot.*q3dot.*t39.*thetadot.*(1.3e1./2.5e2)-q2dot.*q3dot.*t39.*thetadot.*(1.3e1./2.5e2)-q1dot.*q2dot.*t43.*thetadot.*5.915e-1-q1dot.*q3dot.*t44.*thetadot.*(1.3e1./2.5e2)+q7dot.*q8dot.*t33.*thetadot.*(6.93e2./8.0e2)-q2dot.*q3dot.*t44.*thetadot.*(1.3e1./2.5e2)-q4dot.*q6dot.*t40.*thetadot.*(1.3e1./2.5e2)-q5dot.*q6dot.*t40.*thetadot.*(1.3e1./2.5e2)-q4dot.*q5dot.*t45.*thetadot.*5.915e-1-q4dot.*q6dot.*t46.*thetadot.*(1.3e1./2.5e2)-q5dot.*q6dot.*t46.*thetadot.*(1.3e1./2.5e2)-q7dot.*q8dot.*t48.*thetadot.*(6.3e1./1.6e2)+q10dot.*rIxdot.*t62.*thetadot.*(6.3e1./4.0e1)+q10dot.*rIydot.*t58.*thetadot.*(6.3e1./4.0e1)+q1dot.*rIxdot.*t3.*thetadot.*(4.0./2.5e1)+q2dot.*rIxdot.*t3.*thetadot.*(4.0./2.5e1)+q3dot.*rIxdot.*t3.*thetadot.*(4.0./2.5e1)+q4dot.*rIxdot.*t5.*thetadot.*(4.0./2.5e1)+q5dot.*rIxdot.*t5.*thetadot.*(4.0./2.5e1)+q6dot.*rIxdot.*t5.*thetadot.*(4.0./2.5e1)+q1dot.*rIxdot.*t13.*thetadot.*(4.0./2.5e1)+q2dot.*rIxdot.*t13.*thetadot.*(4.0./2.5e1)+q3dot.*rIxdot.*t13.*thetadot.*(4.0./2.5e1)+q4dot.*rIxdot.*t14.*thetadot.*(4.0./2.5e1)+q5dot.*rIxdot.*t14.*thetadot.*(4.0./2.5e1)+q6dot.*rIxdot.*t14.*thetadot.*(4.0./2.5e1)+q1dot.*rIxdot.*t35.*thetadot.*(3.9e1./1.0e1)+q4dot.*rIxdot.*t36.*thetadot.*(3.9e1./1.0e1)+q7dot.*rIxdot.*t37.*thetadot.*(9.1e1./4.0e1)+q9dot.*rIxdot.*t38.*thetadot.*(9.1e1./4.0e1)+q1dot.*rIxdot.*t59.*thetadot.*(9.1e1./5.0e1)+q2dot.*rIxdot.*t59.*thetadot.*(9.1e1./5.0e1)+q4dot.*rIxdot.*t60.*thetadot.*(9.1e1./5.0e1)+q5dot.*rIxdot.*t60.*thetadot.*(9.1e1./5.0e1)+q7dot.*rIxdot.*t61.*thetadot.*(6.3e1./4.0e1)+q8dot.*rIxdot.*t61.*thetadot.*(6.3e1./4.0e1)+q9dot.*rIxdot.*t62.*thetadot.*(6.3e1./4.0e1)+q1dot.*rIydot.*t3.*thetadot.*(4.0./2.5e1)+q2dot.*rIydot.*t3.*thetadot.*(4.0./2.5e1)+q3dot.*rIydot.*t3.*thetadot.*(4.0./2.5e1)+q4dot.*rIydot.*t5.*thetadot.*(4.0./2.5e1)+q5dot.*rIydot.*t5.*thetadot.*(4.0./2.5e1)+q6dot.*rIydot.*t5.*thetadot.*(4.0./2.5e1)-q1dot.*rIydot.*t13.*thetadot.*(4.0./2.5e1)-q2dot.*rIydot.*t13.*thetadot.*(4.0./2.5e1)-q3dot.*rIydot.*t13.*thetadot.*(4.0./2.5e1)-q4dot.*rIydot.*t14.*thetadot.*(4.0./2.5e1)-q5dot.*rIydot.*t14.*thetadot.*(4.0./2.5e1)-q6dot.*rIydot.*t14.*thetadot.*(4.0./2.5e1)+q1dot.*rIydot.*t20.*thetadot.*(3.9e1./1.0e1)+q4dot.*rIydot.*t22.*thetadot.*(3.9e1./1.0e1)+q7dot.*rIydot.*t24.*thetadot.*(9.1e1./4.0e1)+q9dot.*rIydot.*t26.*thetadot.*(9.1e1./4.0e1)+q1dot.*rIydot.*t52.*thetadot.*(9.1e1./5.0e1)+q2dot.*rIydot.*t52.*thetadot.*(9.1e1./5.0e1)+q4dot.*rIydot.*t54.*thetadot.*(9.1e1./5.0e1)+q5dot.*rIydot.*t54.*thetadot.*(9.1e1./5.0e1)+q7dot.*rIydot.*t56.*thetadot.*(6.3e1./4.0e1)+q8dot.*rIydot.*t56.*thetadot.*(6.3e1./4.0e1)+q9dot.*rIydot.*t58.*thetadot.*(6.3e1./4.0e1)-q1dot.*t8.*t63.*t65.*6.5e-3-q3dot.*t6.*t63.*t65.*6.5e-3-q2dot.*t8.*t63.*t65.*6.5e-3-q3dot.*t7.*t63.*t65.*6.5e-3-q3dot.*t12.*t63.*t65.*6.5e-3-q4dot.*t11.*t63.*t67.*6.5e-3-q6dot.*t9.*t63.*t67.*6.5e-3-q1dot.*t7.*t63.*t75.*6.5e-3-q2dot.*t6.*t63.*t75.*6.5e-3-q5dot.*t11.*t63.*t67.*6.5e-3-q6dot.*t10.*t63.*t67.*6.5e-3-q1dot.*t8.*t63.*t75.*6.5e-3-q3dot.*t6.*t63.*t75.*6.5e-3-q6dot.*t12.*t63.*t67.*6.5e-3-q2dot.*t12.*t63.*t75.*6.5e-3-q3dot.*t12.*t63.*t75.*6.5e-3-q4dot.*t10.*t63.*t77.*6.5e-3-q5dot.*t9.*t63.*t77.*6.5e-3-q4dot.*t11.*t63.*t77.*6.5e-3-q6dot.*t9.*t63.*t77.*6.5e-3-q5dot.*t12.*t63.*t77.*6.5e-3-q6dot.*t12.*t63.*t77.*6.5e-3+rIxdot.*t6.*t63.*t72.*(1.0./5.0e1)+rIxdot.*t7.*t63.*t72.*(1.0./5.0e1)+rIxdot.*t8.*t63.*t72.*(1.0./5.0e1)+rIxdot.*t9.*t63.*t73.*(1.0./5.0e1)+rIxdot.*t10.*t63.*t73.*(1.0./5.0e1)+rIxdot.*t11.*t63.*t73.*(1.0./5.0e1)+rIxdot.*t12.*t63.*t72.*(1.0./5.0e1)+rIxdot.*t12.*t63.*t73.*(1.0./5.0e1)+rIydot.*t6.*t63.*t69.*(1.0./5.0e1)+rIydot.*t7.*t63.*t69.*(1.0./5.0e1)+rIydot.*t8.*t63.*t69.*(1.0./5.0e1)+rIydot.*t9.*t63.*t71.*(1.0./5.0e1)+rIydot.*t10.*t63.*t71.*(1.0./5.0e1)+rIydot.*t12.*t63.*t69.*(1.0./5.0e1)+rIydot.*t11.*t63.*t71.*(1.0./5.0e1)+rIydot.*t12.*t63.*t71.*(1.0./5.0e1)-t8.*t63.*t65.*thetadot.*6.5e-3-t11.*t63.*t67.*thetadot.*6.5e-3-t7.*t63.*t75.*thetadot.*6.5e-3-t8.*t63.*t75.*thetadot.*6.5e-3-t10.*t63.*t77.*thetadot.*6.5e-3-t11.*t63.*t77.*thetadot.*6.5e-3-q1dot.*q2dot.*q3dot.*t63.*t65.*(1.3e1./1.0e3)-q1dot.*q2dot.*q3dot.*t63.*t75.*(1.3e1./1.0e3)-q4dot.*q5dot.*q6dot.*t63.*t67.*(1.3e1./1.0e3)-q4dot.*q5dot.*q6dot.*t63.*t77.*(1.3e1./1.0e3)+q1dot.*q2dot.*rIxdot.*t63.*t72.*(1.0./2.5e1)+q1dot.*q3dot.*rIxdot.*t63.*t72.*(1.0./2.5e1)+q2dot.*q3dot.*rIxdot.*t63.*t72.*(1.0./2.5e1)+q4dot.*q5dot.*rIxdot.*t63.*t73.*(1.0./2.5e1)+q4dot.*q6dot.*rIxdot.*t63.*t73.*(1.0./2.5e1)+q5dot.*q6dot.*rIxdot.*t63.*t73.*(1.0./2.5e1)+q1dot.*q2dot.*rIydot.*t63.*t69.*(1.0./2.5e1)+q1dot.*q3dot.*rIydot.*t63.*t69.*(1.0./2.5e1)+q2dot.*q3dot.*rIydot.*t63.*t69.*(1.0./2.5e1)+q4dot.*q5dot.*rIydot.*t63.*t71.*(1.0./2.5e1)+q4dot.*q6dot.*rIydot.*t63.*t71.*(1.0./2.5e1)+q5dot.*q6dot.*rIydot.*t63.*t71.*(1.0./2.5e1)-q1dot.*q3dot.*t63.*t65.*thetadot.*(1.3e1./1.0e3)-q2dot.*q3dot.*t63.*t65.*thetadot.*(1.3e1./1.0e3)-q4dot.*q6dot.*t63.*t67.*thetadot.*(1.3e1./1.0e3)-q1dot.*q2dot.*t63.*t75.*thetadot.*(1.3e1./1.0e3)-q5dot.*q6dot.*t63.*t67.*thetadot.*(1.3e1./1.0e3)-q1dot.*q3dot.*t63.*t75.*thetadot.*(1.3e1./1.0e3)-q2dot.*q3dot.*t63.*t75.*thetadot.*(1.3e1./1.0e3)-q4dot.*q5dot.*t63.*t77.*thetadot.*(1.3e1./1.0e3)-q4dot.*q6dot.*t63.*t77.*thetadot.*(1.3e1./1.0e3)-q5dot.*q6dot.*t63.*t77.*thetadot.*(1.3e1./1.0e3)+q1dot.*rIxdot.*t63.*t72.*thetadot.*(1.0./2.5e1)+q2dot.*rIxdot.*t63.*t72.*thetadot.*(1.0./2.5e1)+q3dot.*rIxdot.*t63.*t72.*thetadot.*(1.0./2.5e1)+q4dot.*rIxdot.*t63.*t73.*thetadot.*(1.0./2.5e1)+q5dot.*rIxdot.*t63.*t73.*thetadot.*(1.0./2.5e1)+q6dot.*rIxdot.*t63.*t73.*thetadot.*(1.0./2.5e1)+q1dot.*rIydot.*t63.*t69.*thetadot.*(1.0./2.5e1)+q2dot.*rIydot.*t63.*t69.*thetadot.*(1.0./2.5e1)+q3dot.*rIydot.*t63.*t69.*thetadot.*(1.0./2.5e1)+q4dot.*rIydot.*t63.*t71.*thetadot.*(1.0./2.5e1)+q5dot.*rIydot.*t63.*t71.*thetadot.*(1.0./2.5e1)+q6dot.*rIydot.*t63.*t71.*thetadot.*(1.0./2.5e1);
