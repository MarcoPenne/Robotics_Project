function [Y_final, u_final, signs_finals] = tree_franka(Y1, Y2, Y3, Y4, Y5, Y6, Y7, u1, u2, u3, u4, u5, u6, u7, indices1, indices2, indices3, indices4, indices5, indices6, indices7, LB, UB, maximum_iterations)

    n_segments1 = size(indices1,1);
    n_segments2 = size(indices2,1);
    n_segments3 = size(indices3,1);
    n_segments4 = size(indices4,1);
    n_segments5 = size(indices5,1);
    n_segments6 = size(indices6,1);
    n_segments7 = size(indices7,1);
    n_segments = n_segments1 + n_segments2 + n_segments3 + n_segments4 + n_segments5 + n_segments6 + n_segments7;
    n_top_pool = 3;
    num_x = length(LB);
    TOP_POOL = zeros(n_top_pool, num_x);
    TOP_SIGNS = {zeros(n_segments,1)};
    TOP_RESULTS = {rand(num_x,1).*(UB-LB) + LB};
    TOP_LOSSES = [Inf];
    TOP_Y = {[]};
    TOP_U = {[]};
    signs = ones(n_segments,1);

    permuted_order = randperm(n_segments);
    
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
    
    Y_final = {};
    u_final = {};
    
    LOSSES = [];
    SIGNS = {};
    RESULTS = {};
    Y = {};
    U = {};
    
    iterations = min([maximum_iterations, length(permuted_order)]);
    for k = 1:iterations
        i = permuted_order(k);
        
        string2disp = sprintf("PROCESSING SEGMENT %d/%d  (%d)", k, iterations, i);
        disp(string2disp);
        
        for pool=1:size(TOP_Y, 2)
            Y_now = [TOP_Y{pool}; Y_segments{i}];
            U_now = [TOP_U{pool}; u_segments{i}];
            Y_now = double(Y_now);
            U_now = double(U_now);

            Y{length(Y)+1} = Y_now;
            U{length(U)+1} = U_now;
            
            [loss_pos, solution_pos] = solve_optimization_franka(Y_now, U_now, LB, UB, TOP_RESULTS{pool});
            LOSSES = [LOSSES, loss_pos];
            RESULTS = [RESULTS, solution_pos];
            signs_now = TOP_SIGNS{pool};
            signs_now(i) = 1;
            SIGNS = [SIGNS, signs_now];
            
            Y_now = [TOP_Y{pool}; Y_segments{i}];
            U_now = [TOP_U{pool}; -u_segments{i}];
            
            Y{length(Y)+1} = Y_now;
            U{length(U)+1} = U_now;
            
            [loss_neg, solution_neg] = solve_optimization_franka(Y_now, U_now, LB, UB, TOP_RESULTS{pool});
            LOSSES = [LOSSES, loss_neg];
            RESULTS = [RESULTS, solution_neg];
            signs_now = TOP_SIGNS{pool};
            signs_now(i) = -1;
            SIGNS = [SIGNS, signs_now];
        end
        
        [~,LOSS_sort] = sort(LOSSES);
        Y = Y(LOSS_sort);
        U = U(LOSS_sort);
        RESULTS = RESULTS(LOSS_sort);
        SIGNS = SIGNS(LOSS_sort);
        LOSSES = LOSSES(LOSS_sort);
        
        if n_top_pool<length(LOSSES)
            TOP_Y = Y(1:n_top_pool);
            TOP_U = U(1:n_top_pool);
            TOP_LOSSES = LOSSES(1:n_top_pool);
            TOP_RESULTS = RESULTS(1:n_top_pool);
            TOP_SIGNS = SIGNS(1:n_top_pool);
        else
            TOP_Y = Y;
            TOP_U = U;
            TOP_LOSSES = LOSSES;
            TOP_RESULTS = RESULTS;
            TOP_SIGNS = SIGNS;
        end
        
        LOSSES = [];
        Y = {};
        U = {};
        RESULTS = {};
        SIGNS = {};
            
    end
    
    Y_final = TOP_Y{1};
    u_final = TOP_U{1};
    signs_finals = TOP_SIGNS{1};
    signs1 = signs_finals(1:n_segments1);
    signs2 = signs_finals(1+n_segments1:n_segments1+n_segments2);
    signs3 = signs_finals(1+n_segments1+n_segments2:n_segments1+n_segments2+n_segments3);
    signs4 = signs_finals(1+n_segments1+n_segments2+n_segments3:n_segments1+n_segments2+n_segments3+n_segments4);
    signs5 = signs_finals(1+n_segments1+n_segments2+n_segments3+n_segments4:n_segments1+n_segments2+n_segments3+n_segments4+n_segments5);
    signs6 = signs_finals(1+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5:n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6);
    signs7 = signs_finals(1+n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6:n_segments1+n_segments2+n_segments3+n_segments4+n_segments5+n_segments6+n_segments7);
    
    signs_finals = {signs1, signs2, signs3, signs4, signs5, signs6, signs7};
end