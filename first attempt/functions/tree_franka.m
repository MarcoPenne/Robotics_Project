function [Y_final, u_final, signs_finals] = tree_franka(Y1, Y2, Y3, Y4, Y5, Y6, Y7, u1, u2, u3, u4, u5, u6, u7, indices1, indices2, indices3, indices4, indices5, indices6, indices7, LB, UB, n_trees, maximum_depth)

    n_segments1 = size(indices1,1);
    n_segments2 = size(indices2,1);
    n_segments3 = size(indices3,1);
    n_segments4 = size(indices4,1);
    n_segments5 = size(indices5,1);
    n_segments6 = size(indices6,1);
    n_segments7 = size(indices7,1);
    n_segments = n_segments1 + n_segments2 + n_segments3 + n_segments4 + n_segments5 + n_segments6 + n_segments7;
    
    Y_segments = {};
    u_segments = {};
    for i=1:n_segments1
        Y_segments{i} = Y1(indices1(i, 1):indices1(i, 2), :);
        u_segments{i} = u1(indices1(i, 1):indices1(i, 2));
    end
    
    for i=1:n_segments2
        Y_segments{i+n_segments1} = Y2(indices2(i, 1):indices2(i, 2), :);
        u_segments{i+n_segments1} = u2(indices2(i, 1):indices2(i, 2));
    end
    
    for i=1:n_segments3
        Y_segments{i+n_segments1+n_segments2} = Y3(indices3(i, 1):indices3(i, 2), :);
        u_segments{i+n_segments1+n_segments2} = u3(indices3(i, 1):indices3(i, 2));
    end
    
    for i=1:n_segments4
        Y_segments{i+n_segments1+n_segments2+n_segments3} = Y4(indices4(i, 1):indices4(i, 2), :);
        u_segments{i+n_segments1+n_segments2+n_segments3} = u4(indices4(i, 1):indices4(i, 2));
    end
    
    for i=1:n_segments5
        Y_segments{i+n_segments1+n_segments2+n_segments3+n_segments4} = Y5(indices5(i, 1):indices5(i, 2), :);
        u_segments{i+n_segments1+n_segments2+n_segments3+n_segments4} = u5(indices5(i, 1):indices5(i, 2));
    end
    
    for i=1:n_segments6
        Y_segments{i+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5} = Y6(indices6(i, 1):indices6(i, 2), :);
        u_segments{i+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5} = u6(indices6(i, 1):indices6(i, 2));
    end
    
    for i=1:n_segments7
        Y_segments{i+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6} = Y7(indices7(i, 1):indices7(i, 2), :);
        u_segments{i+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6} = u7(indices7(i, 1):indices7(i, 2));
    end
    
    [Y_final, u_final, signs_finals] = forest_franka(Y_segments, u_segments, LB, UB, n_segments, n_trees, maximum_depth);
    
    signs1 = signs_finals(1:n_segments1);
    signs2 = signs_finals(1+n_segments1:n_segments1+n_segments2);
    signs3 = signs_finals(1+n_segments1+n_segments2:n_segments1+n_segments2+n_segments3);
    signs4 = signs_finals(1+n_segments1+n_segments2+n_segments3:n_segments1+n_segments2+n_segments3+n_segments4);
    signs5 = signs_finals(1+n_segments1+n_segments2+n_segments3+n_segments4:n_segments1+n_segments2+n_segments3+n_segments4+n_segments5);
    signs6 = signs_finals(1+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5:n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6);
    signs7 = signs_finals(1+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6:n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6+n_segments7);
    
    signs_finals = {signs1, signs2, signs3, signs4, signs5, signs6, signs7};
end