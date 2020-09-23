function [c] = get_1dof_coefficients(m, d, I)
    c = [9.81*m*d; I + m*d^2];
end