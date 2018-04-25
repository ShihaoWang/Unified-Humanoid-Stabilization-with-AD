function Obj_Constraint_Grad_Cal(z, P)
% This function is used to compute the grad of obj and constraint

v=['[' sprintf('a%d,',1:length(z))];
v(end)=']';
z=sym(v).';

Obj_Sym = Nodes_Connectivity_Obj(z,P);

[c_Sym, ceq_Sym]= Nodes_Connectivity_Constraint(z,P);

c_Sym_Grad = jacobian(c_Sym,z);
ceq_Sym_Grad = jacobian(ceq_Sym,z);

c_Sym_Grad_fn = matlabFunction(c_Sym_Grad,'File','c_Sym_Grad_fn.mat');
ceq_Sym_Grad_fn = matlabFunction(ceq_Sym_Grad,'File','ceq_Sym_Grad_fn.mat');


end

