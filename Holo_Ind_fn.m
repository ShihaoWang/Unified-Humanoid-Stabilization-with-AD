function Holo_Ind = Holo_Ind_fn(sigma_active)

Active_Rows = zeros(6,1);

if sigma_active(1) == 1
    Active_Rows(1) = 1;
    Active_Rows(2) = 1;
end

if sigma_active(2) == 1
    Active_Rows(3) = 1;
    Active_Rows(4) = 1;
end

if sigma_active(3) == 1
    Active_Rows(5) = 1;
end

if sigma_active(4) == 1
    Active_Rows(6) = 1;
end

Holo_Ind = Active_Rows;
end