
%========================== Optimal Solution ========================
q1q2A1C1_norm = [];
q1q2A2C2_norm = [];
q1q2A1C1A2C2_norm = [];
Mode_current=Mode;
%q1q2 = q0q1q2_CurrentStep(:,2:11);
for jj = 1:length(q1q2(:,1))
    % Here, for the first step, modes 1-9 execute once (length(MPOTP_cell) = 2); 
    if  j == 1 && i == col_Self_adjustment(1) && Self_adjustment_Enable_Disable == 1 % || Self_adjustment_Enable_Disable == 2
        % we first confirm the one value of the end point (q0q1q2_required_endpoint) by comparing it with Homeconfiguration
        q0q1q2_matrix_start = q0q1q2_previous_trajpoint;
        q0q1q2_matrix_end = q0q1q2_current_trajpoint;
        
        %========= Here we must judge five-bar or three-bar sparately;=====
        % Because Five/Three-bar should be judge as whole, and other mode should judge sparate
        if Mode_current == 8 || Mode_current == 9
            if length(q0q1q2_matrix_end(:,1)) > 4
                q0q1q2_matrix_end = [];
                if PosOri_current{1} < 0
                    kk = 0;
                    for k = 1:length(q0q1q2_current_trajpoint(:,1))
                        if q0q1q2_current_trajpoint(k,2) > 0
                            kk = kk + 1;
                            q0q1q2_matrix_end(kk,:) = q0q1q2_current_trajpoint(k,:);
                        end
                    end
                elseif PosOri_current{1} > 0
                    kk = 0;
                    for k = 1:length(q0q1q2_current_trajpoint(:,1))
                        if q0q1q2_current_trajpoint(k,2) < 0
                            kk = kk + 1;
                            q0q1q2_matrix_end(kk,:) = q0q1q2_current_trajpoint(k,:);
                        end
                    end
                end
            end
            for k = 1:length(q0q1q2_matrix_end(:,1))
                q1q2_matrix_norm(k) = norm(q0q1q2_matrix_end(k,2:11) - q0q1q2_matrix_start(1,2:11));
            end
            [rowsq1q2,colsq1q2] = find(q1q2_matrix_norm == min(min(q1q2_matrix_norm)));
            SolutionRow_q1q2 = colsq1q2(1);
            q0q1q2_required_endpoint = [q0q1q2_matrix_end(1), q0q1q2_matrix_end(SolutionRow_q1q2,2:11)];
        elseif Mode_current == 3 || Mode_current == 4
            % Only deal with Mode5 to Mode 4
            % Because first step q11 = +/-pi/2 and the precision of norm
            % has a 1e-16 difference, so it should be judge the correct
            % value by using the next step's x position value of Moving
            % Platform
            q0q1q2_matrix_end = [];
            if Mode_current == 3 
                if PosOri_current{1} < 0
                    for k = 1:length(q0q1q2_current_trajpoint(:,1))
                        if q0q1q2_current_trajpoint(k,7) < 0
                            q0q1q2_matrix_end = [q0q1q2_matrix_end; q0q1q2_current_trajpoint(k,:)];
                        end
                    end
                elseif PosOri_current{1} > 0
                    for k = 1:length(q0q1q2_current_trajpoint(:,1))
                        if q0q1q2_current_trajpoint(k,7) > 0
                            q0q1q2_matrix_end = [q0q1q2_matrix_end; q0q1q2_current_trajpoint(k,:)];
                        end
                    end
                end
            elseif Mode_current == 4
                if PosOri_current{1} < 0
                    for k = 1:length(q0q1q2_current_trajpoint(:,1))
                        if q0q1q2_current_trajpoint(k,2) > 0
                            q0q1q2_matrix_end = [q0q1q2_matrix_end; q0q1q2_current_trajpoint(k,:)];
                        end
                    end
                elseif PosOri_current{1} > 0
                    for k = 1:length(q0q1q2_current_trajpoint(:,1))
                        if q0q1q2_current_trajpoint(k,2) < 0
                            q0q1q2_matrix_end = [q0q1q2_matrix_end; q0q1q2_current_trajpoint(k,:)];
                        end
                    end
                end
            end
            for k = 1:length(q0q1q2_matrix_end(:,1))
                q1_matrix_norm(k) = norm(q0q1q2_matrix_end(k,2:6) - q0q1q2_matrix_start(1,2:6));
                q2_matrix_norm(k) = norm(q0q1q2_matrix_end(k,7:11) - q0q1q2_matrix_start(1,7:11));
            end
            [rowsq1,colsq1] = find(q1_matrix_norm == min(min(q1_matrix_norm)));
            [rowsq2,colsq2] = find(q2_matrix_norm == min(min(q2_matrix_norm)));
            SolutionRow_q1 = colsq1(1);
            SolutionRow_q2 = colsq2(1);
            q0q1q2_required_endpoint = [q0q1q2_matrix_end(1), q0q1q2_matrix_end(SolutionRow_q1,2:6), q0q1q2_matrix_end(SolutionRow_q2,7:11)];
        else
            for k = 1:length(q0q1q2_matrix_end(:,1))
                q1_matrix_norm(k) = norm(q0q1q2_matrix_end(k,2:6) - q0q1q2_matrix_start(1,2:6));
                q2_matrix_norm(k) = norm(q0q1q2_matrix_end(k,7:11) - q0q1q2_matrix_start(1,7:11));
            end
            [rowsq1,colsq1] = find(q1_matrix_norm == min(min(q1_matrix_norm)));
            [rowsq2,colsq2] = find(q2_matrix_norm == min(min(q2_matrix_norm)));
            SolutionRow_q1 = colsq1(1);
            SolutionRow_q2 = colsq2(1);
            q0q1q2_required_endpoint = [q0q1q2_matrix_end(1), q0q1q2_matrix_end(SolutionRow_q1,2:6), q0q1q2_matrix_end(SolutionRow_q2,7:11)];
        end
        %==================================================================
        q0q1q2_Optimal_SingleRow = q0q1q2_previous_trajpoint;
    elseif j == 2 && i == col_Self_adjustment(1) && Self_adjustment_Enable_Disable == 1% || Self_adjustment_Enable_Disable == 2
        % Secondly, confirm the second value that has minimum norm value with the end point (q0q1q2_required_endpoint)
        %========= Here we must judge five-bar or three-bar sparately;=====
        if Mode_current == 8 || Mode_current == 9
            q1q2A1C1A2C2_norm(jj) = norm(q1q2(jj,1:10) - q0q1q2_required_endpoint(2:11));
        elseif Mode_current == 3
            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 2:6));
            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_required_endpoint(7:11));
        elseif Mode_current == 4
            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_required_endpoint(2:6));
            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 7:11));
        else
            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_required_endpoint(2:6));
            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_required_endpoint(7:11));
        end
        %==================================================================
    else
        % Thirdly, confirm the following value that has minimum norm value with the Previous step value (q0q1q2_mat(PreviousOneStep))
        %========= Here we must judge five-bar or three-bar sparately;=====
        if Mode_current == 8 || Mode_current == 9
            if j == 1 && (i == 1 || Self_adjustment_Enable_Disable == 1 || Self_adjustment_Enable_Disable == 2)
                q0q1q2_Optimal_SingleRow = q0q1q2_previous_trajpoint;
                continue
            else
                q1q2A1C1A2C2_norm(jj) = norm(q1q2(jj,1:10) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 2:11));
            end
        else
            if Self_adjustment_Enable_Disable == 1
                if length(MPOTP_cell) >= 3 && (MPOTP_cell{length(MPOTP_cell)}{1} == 10 || MPOTP_cell{length(MPOTP_cell)}{1} == 11)
                    if j == 1
                        q0q1q2_Optimal_SingleRow = q0q1q2_previous_trajpoint;
                        continue
                    else
                        if i == length(MPOTP_cell)
                            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(NumIntepoPoints*(i-2), 2:6));
                            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(NumIntepoPoints*(i-2), 7:11));
                        else
                            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 2:6));
                            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 7:11));
                        end
                    end
                else%if length(MPOTP_cell) == 2
                    if j == 1 
                        q0q1q2_Optimal_SingleRow = q0q1q2_previous_trajpoint;
                        continue
                    else
                        q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 2:6));
                        q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 7:11));
                    end
                end
            elseif Self_adjustment_Enable_Disable == 2
                if j == 1 && i == 1
                    q0q1q2_Optimal_SingleRow = q0q1q2_previous_trajpoint;
                    continue
                else
                    q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 2:6));
                    q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 7:11));
                end
            else
                if j == 1 && i == 1
                    q0q1q2_Optimal_SingleRow = q0q1q2_previous_trajpoint;
                    continue
                else% Start from the second step
                    q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 2:6));
                    q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1, 7:11));                    
                end
            end
        end
        %==================================================================
    end
    
end

if  j == 1 && Mode_current == 5
    %%%%% If the step it's self-adjustment,the first step is Homeconfiguration
    %%%%%  But we should also consider if it's not the first step, and it's other steps, it should be also possible to handle it
else
    if j == 1 
        %%%%% if the two steps are the same, so, this step is the same as the last step
    else
        if Mode_current == 8 || Mode_current == 9
            [rowsA1C1A2C2,colsA1C1A2C2] = find(q1q2A1C1A2C2_norm == min(min(q1q2A1C1A2C2_norm)));
            SolutionRow_A1C1A2C2 = colsA1C1A2C2(1);
            q0q1q2_Optimal_SingleRow = [q0, q1q2(SolutionRow_A1C1A2C2,1:10)];
        else
            [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
            [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
            SolutionRow_A1C1 = colsA1C1(1);
            SolutionRow_A2C2 = colsA2C2(1);
            q0q1q2_Optimal_SingleRow = [q0, q1q2(SolutionRow_A1C1,1:5), q1q2(SolutionRow_A2C2,6:10)];
        end
    end
end
%============================= End =================================

%========================== Collision Check ============================
%CollisionCheck_LineDisplay;
%============================= End =================================