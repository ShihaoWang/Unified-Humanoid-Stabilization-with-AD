function Kinematics_Checking()

% This function is used to check the kinematics of the newly derived robot
load('Pre_Load_Structure.mat');

rIx = 0;
rIy = 0;
theta = 0.21;
q1 = 0.733;
q2 = 1.5;
q3 = 0.733;
q4 = -1.5;
q5 = 2;
q6 = -1.3;
q7 = 1;
q8 = -2;
q9 = -pi/2;
q10 = -1.5;

q_array_i = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 ]';
Single_Frame_Plot(q_array_i, P);

end

