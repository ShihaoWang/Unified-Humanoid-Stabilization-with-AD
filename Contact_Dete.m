function [value,isterminal,direction] = Contact_Dete(t, z, P)

value = [];             isterminal = [];        direction = [];
r_Coef = zeros(6,1); 

sigma_i = P.sigma_i;  % This is the basic reference contas

% sigma_i_child = P.sigma_i_child; 

rIx_i = z(1);             rIy_i = z(2);             theta_i = z(3);
q1_i = z(4);              q2_i = z(5);              q3_i = z(6);
q4_i = z(7);              q5_i = z(8);              q6_i = z(9);
q7_i = z(10);             q8_i = z(11);             q9_i = z(12);
q10_i = z(13);

rA_i = P.rA_fn(q1_i,q2_i,q3_i,rIx_i,rIy_i,theta_i);
rB_i = P.rB_fn(q1_i,q2_i,q3_i,rIx_i,rIy_i,theta_i);
rC_i = P.rC_fn(q4_i,q5_i,q6_i,rIx_i,rIy_i,theta_i);
rD_i = P.rD_fn(q4_i,q5_i,q6_i,rIx_i,rIy_i,theta_i);
rE_i = P.rE_fn(q7_i,q8_i,rIx_i,rIy_i,theta_i);
rF_i = P.rF_fn(q9_i,q10_i,rIx_i,rIy_i,theta_i);

rA_Rel = Obs_Dist_Fn(rA_i, P.Envi_Map);
rB_Rel = Obs_Dist_Fn(rB_i, P.Envi_Map);
rC_Rel = Obs_Dist_Fn(rC_i, P.Envi_Map);
rD_Rel = Obs_Dist_Fn(rD_i, P.Envi_Map);
rE_Rel = Obs_Dist_Fn(rE_i, P.Envi_Map);
rF_Rel = Obs_Dist_Fn(rF_i, P.Envi_Map);

% The main idea is to stop a the certain integration when a new contact has
% been detacted.
Active_Ind = find(sigma_i);
if isempty(Active_Ind)==0
    for i = 1:length(Active_Ind)
        r_Coef(Active_Ind(i)) = sigma_i(Active_Ind(i));
    end
end
value = [value; rA_Rel];            % The value that we want to be zero
isterminal = [isterminal;r_Coef(1)];        % Halt integration 
direction = [direction; -1];         % The zero can be approached from either direction

value = [value; rB_Rel];            % The value that we want to be zero
isterminal = [isterminal;r_Coef(2)];        % Halt integration 
direction = [direction; -1];         % The zero can be approached from either direction

value = [value; rC_Rel];            % The value that we want to be zero
isterminal = [isterminal;r_Coef(3)];        % Halt integration 
direction = [direction; -1];         % The zero can be approached from either direction

value = [value; rD_Rel];            % The value that we want to be zero
isterminal = [isterminal;r_Coef(4)];        % Halt integration 
direction = [direction; -1];         % The zero can be approached from either direction

value = [value; rE_Rel];            % The value that we want to be zero
isterminal = [isterminal;r_Coef(5)];        % Halt integration 
direction = [direction; -1];         % The zero can be approached from either direction

value = [value; rF_Rel];            % The value that we want to be zero
isterminal = [isterminal;r_Coef(6)];        % Halt integration 
direction = [direction; -1];         % The zero can be approached from either direction
end