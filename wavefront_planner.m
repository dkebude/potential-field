function [w_path] = wavefront_planner(c_space, alpha, beta, alpha_index, beta_index, alpha_goal_index, beta_goal_index, w_steps)

    wavefront = c_space;
    wavefront(alpha_goal_index,beta_goal_index) = 2;
    adj = [0 -1; -1 0; 1 0; 0 1];
    i_list = [alpha_goal_index, beta_goal_index];

    while size(i_list,1) ~= 0
        % Iterate through cells adjacent to the cell at the top of the open queue:
        for k=1:size(adj,1)
            % Calculate index for current adjacent cell:
            cur_adj = i_list(1,:)+adj(k,:);
            % Make sure adjacent cell is in the map
            if min(cur_adj) < 1 || cur_adj(1) > length(alpha) || cur_adj(2) > length(beta)
              continue
            end
            % Make sure the adjacent cell is not an obstacle 
            if c_space(cur_adj(1), cur_adj(2)) == 1 
                wavefront(cur_adj(1), cur_adj(2)) = 1;
                continue
            end

            % or not iterated
            if wavefront(cur_adj(1), cur_adj(2)) ~= 0 && wavefront(cur_adj(1), cur_adj(2)) ~= 1.5 
                continue
            end
            % Set the cost and add the adjacent to the open set
            wavefront(cur_adj(1), cur_adj(2)) = wavefront(i_list(1,1), i_list(1,2)) + 1;
            i_list(size(i_list,1)+1,:) = cur_adj;
        end

        % Pop the top open cell from the queue
        i_list = i_list(2:end,:);
    end

    % Find a path to the goal.
    w_path = [alpha_index; beta_index];
    for j = 1:w_steps-1

        i = size(w_path,2);
        if w_path(:,i) == [alpha_goal_index; beta_goal_index];
            break
        end

        curc = wavefront(w_path(1,j), w_path(2,j));
        if curc == 1 || curc == 1.5
            % if we're in an obstacle (bad initial state, most likely)
            curc = max(wavefront);
        end

        noMove = 1;    
        for k=1:size(adj,1)
            % Calculate index for current adjacent cell:
            cur_adj = w_path(:,j)+adj(k,:)';
            % Make sure adjacent cell is in the map
            if min(cur_adj) < 1 || cur_adj(1) > length(alpha) || cur_adj(2) > length(beta)
              continue
            end

            % If this adjacent cell reduces cost, add it to the path.
            if wavefront(cur_adj(1),cur_adj(2)) == 1
              % (obstacle)
              continue;
            end
            if wavefront(cur_adj(1),cur_adj(2)) < curc
              noMove = 0;
              w_path(:,j+1) = cur_adj;
              break
            end
        end
        if noMove
            w_path(:,j+1) = [alpha_index; beta_index];
            break
        end
    end
    
end