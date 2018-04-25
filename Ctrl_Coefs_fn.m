function Ctrl_Coefs = Ctrl_Coefs_fn(t_intp, u_intp, Order, Ctrl_No)
% [m,n] = size(u_intp);
% This function is used to find the Ctrl_Coefs
Ctrl_Coefs = [];
for i = 1:Ctrl_No
    Intp_ind_init = (Order-1) * (i-1) + 1;
    Intp_ind_finl = Intp_ind_init + (Order-1);
    
    t_intp_i = t_intp(:,Intp_ind_init:Intp_ind_finl);
    u_intp_i = u_intp(:,Intp_ind_init:Intp_ind_finl);
    
    pp = spline(t_intp_i,u_intp_i);
    Ctrl_Coefs_i = pp.coefs;
    Ctrl_Coefs_i = reshape(Ctrl_Coefs_i', Order * 10,1);
    Ctrl_Coefs = [Ctrl_Coefs; Ctrl_Coefs_i];
end
end

