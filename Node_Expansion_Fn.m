function sigma_children = Node_Expansion_Fn(Node_i)

% This function is the main function used to expansion the given node to
% its adjacent nodes without any connectivity test

%%      Inputs;
%           sigma_i:        the contact status at the time i

%%      Output:
%           sigma_children: the updated queue after a node expansion

%%      The main algorithm
%
%       Hand contact: 0-> Add hand contact for each end effector
%                     1-> Since one hand is in contact,
%                               Expanded with adding the other hand contact
%                                             removing the current hand contact
%                     2-> Now two hands are in contact, 
%                               Expanded with retracting either hand contact     
%       Foot contact: 0-> Add either foot contact point next
%                     1-> Since one foot is in contact, adding one or
%                     removing one
%                     2-> Remove either foot contact
%

sigma_i = Node_i.mode;
sigma_children = []; % Initialization to be empty

foot_AB_contas = sigma_i(1);
foot_CD_contas = sigma_i(2);
hand_E_contas = sigma_i(3);
hand_F_contas = sigma_i(4);

% %% 1. Hand contact expansion
% if (hand_E_contas == 0)&&(hand_F_contas == 0)
%     %% A. No contact case  
%     % In this case, there assumes to be a hand collision
%     sigma_i_child = sigma_modi(sigma_i, 3, 1);
%     sigma_children = [sigma_children; sigma_i_child];
%     sigma_i_child = sigma_modi(sigma_i, 4, 1);
%     sigma_children = [sigma_children; sigma_i_child];
%     
% else
%     %% B. Two hand contact case
%     if (hand_E_contas == 1)&&(hand_F_contas == 1)
%         sigma_i_child = sigma_modi(sigma_i, 3, 0);
%         sigma_children = [sigma_children; sigma_i_child];
%         sigma_i_child = sigma_modi(sigma_i, 4, 0);
%         sigma_children = [sigma_children; sigma_i_child];
%     else
%         %% C. One hand contact case
%         sigma_i_child = sigma_modi(sigma_i, 3, 0);
%         sigma_i_child = sigma_modi(sigma_i_child, 4, 0);
%         sigma_children = [sigma_children; sigma_i_child];
%         
%         sigma_i_child = sigma_modi(sigma_i, 3, 1);
%         sigma_i_child = sigma_modi(sigma_i_child, 4, 1);
%         sigma_children = [sigma_children; sigma_i_child];              
%     end
% end

%% 2. Foot contact expansion
if (foot_AB_contas == 0)&&(foot_CD_contas == 0)
    %% 1. No contact case
    sigma_i_child = sigma_modi(sigma_i, 1, 1);
    sigma_children = [sigma_children; sigma_i_child];
    sigma_i_child = sigma_modi(sigma_i, 2, 1);
    sigma_children = [sigma_children; sigma_i_child];
else
    %% 2. Two foot contact case
    if (foot_AB_contas == 1)&&(foot_CD_contas == 1)       
        sigma_i_child = sigma_modi(sigma_i, 1, 0);
        sigma_children = [sigma_children; sigma_i_child];
        sigma_i_child = sigma_modi(sigma_i, 2, 0);
        sigma_children = [sigma_children; sigma_i_child];
    else
        %% 3. One foot contact case
        [~,Active_Ind] = max(sigma_i(1,1:2));
        [~,Inactive_Ind] = min(sigma_i(1,1:2));
        sigma_i_child = sigma_modi(sigma_i, Active_Ind, 0);
        sigma_children = [sigma_children; sigma_i_child];
        sigma_i_child = sigma_modi(sigma_i, Inactive_Ind, 1);
        sigma_children = [sigma_children; sigma_i_child];
    end
end
end

function sigma_i_child = sigma_modi(sigma_ref, contas_ind, AddOrRet)
sigma_i_child = sigma_ref;
sigma_i_child(contas_ind) = AddOrRet;
end