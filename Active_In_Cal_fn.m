function Active_In = Active_In_Cal_fn(Jac_t, sigma_t)

% This function is use to calculate the active index for the Jacobian
% matrix given Jac_t and sigma_t: 4 by 1

Active_In_Raw = [];
if sigma_t(1)==1
    Active_In_Raw = [Active_In_Raw 1 2 3 4];
end
if sigma_t(2)==1
    Active_In_Raw = [Active_In_Raw 5 6 7 8];
end
if sigma_t(3) == 1
    Active_In_Raw = [Active_In_Raw 9 10];  
end
if sigma_t(4)==1
    Active_In_Raw = [Active_In_Raw 11 12];     
end

Jac_t = Jac_t(Active_In_Raw,:);

[Xsub,idx]=licols(Jac_t');

% Active_In = [];Jac_t_In = [];
% for i = 1:length(Active_In_Raw)
%     Jac_t_temp = Jac_t(i,:);
%     [Q,R] = lu([Jac_t_In;Jac_t_temp]');
%     [m,n] = size([Jac_t_In;Jac_t_temp]');
%     if rank(R) == n
%         Active_In = [Active_In i];
%         Jac_t_In = [Jac_t_In; Jac_t(i,:)];
%     end
% end

% [~,Active_In] = rref(Jac_t');

% However here this Active_In is not the same index from the original
% matrix to retrieve the active index from the original matrix
Active_In = idx;
Active_In = Active_In_Raw(:,Active_In);

end

