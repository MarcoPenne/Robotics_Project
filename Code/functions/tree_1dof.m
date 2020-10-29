function [Y_final, u_final, signs_finals] = tree_1dof(Y, u, indices, LB, UB)

    n_segments = size(indices,1);
    n_top_pool = 5;
    num_x = length(LB);
    TOP_POOL = zeros(n_top_pool, num_x);
    TOP_SIGNS = {zeros(n_segments,1)};
    TOP_RESULTS = {rand(num_x,1).*(UB-LB) + LB};
    TOP_LOSSES = [Inf];
    TOP_Y = {[]};
    TOP_U = {[]};
    signs = ones(n_segments,1);

    permuted_order = [];
    
    Y_segments = {};
    u_segments = {};
    for i=1:n_segments
        Y_segments{i} = Y(indices(i, 1):indices(i, 2), :);
        u_segments{i} = u(indices(i, 1):indices(i, 2));
    end
    
    best_condition = Inf;
    best_condition_indices = [0, 0];
    for i=1:n_segments-1
        for j=i+1:n_segments
            c = cond([Y_segments{i}; Y_segments{j}]);
            if c<best_condition
                best_condition = c;
                best_condition_indices = [i, j];
            end
        end
    end
    permuted_order(end+1) = best_condition_indices(1);
    permuted_order(end+1) = best_condition_indices(2);
    
    remaining_segments = 1:n_segments;
    remaining_segments(best_condition_indices(1)) = [];
    remaining_segments(best_condition_indices(2)-1) = [];
    Y_now = [Y_segments{best_condition_indices(1)}; Y_segments{best_condition_indices(2)}];
    for k=1:n_segments-2
        best_condition = Inf;
        best_condition_index = 0;
        best_condition_index_reman = 0;
        for i=1:length(remaining_segments)
            j = remaining_segments(i);
            c = cond([Y_now; Y_segments{j}]);
            if c<best_condition
                best_condition = c;
                best_condition_index = j;
                best_condition_index_reman = i;
            end
        end
        permuted_order(end+1) = best_condition_index;
        Y_now = [Y_now; Y_segments{best_condition_index}];
        remaining_segments(best_condition_index_reman) = [];
    end
    
    Y_final = {};
    u_final = {};
    
    LOSSES = [];
    SIGNS = {};
    RESULTS = {};
    Y = {};
    U = {};
    for k = 1:length(permuted_order)
        i = permuted_order(k);
        
        string2disp = sprintf("PROCESSING SEGMENT %d/%d  (%d)", k, n_segments, i);
        disp(string2disp);
        
        for pool=1:size(TOP_Y, 2)
            Y_now = [TOP_Y{pool}; Y_segments{i}];
            U_now = [TOP_U{pool}; u_segments{i}];
            Y_now = double(Y_now);
            U_now = double(U_now);

            Y{length(Y)+1} = Y_now;
            U{length(U)+1} = U_now;
            
            [loss_pos, solution_pos] = solve_optimization_1dof(Y_now, U_now, LB, UB, TOP_RESULTS{pool});
            LOSSES = [LOSSES, loss_pos];
            RESULTS = [RESULTS, solution_pos];
            signs_now = TOP_SIGNS{pool};
            signs_now(i) = 1;
            SIGNS = [SIGNS, signs_now];
            
            Y_now = [TOP_Y{pool}; Y_segments{i}];
            U_now = [TOP_U{pool}; -u_segments{i}];
            
            Y{length(Y)+1} = Y_now;
            U{length(U)+1} = U_now;
            
            [loss_neg, solution_neg] = solve_optimization_1dof(Y_now, U_now, LB, UB, TOP_RESULTS{pool});
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

end