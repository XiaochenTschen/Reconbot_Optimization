function [ PosOri_Output_Intep, q0q1q2_P2P_Pos_Intep, q0q1q2_P2P_Vel_Intep ,q0q1q2_P2P_Acc_Intep, MP_Pos_Intep, MP_Vel_Intep, MP_Acc_Intep, MP_time_Intep, Mode_det_Jq_Jc_J_Intep ] = ...
    MotionPlanningTransitionModes(Mode_previous, PosOri_previous, q0q1q2_previous_trajpoint,...
    Mode_current,PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, Start_Time, Time_inteval, l1, l2)
% Motion planning and Optimal Solution
%
%      Mode_current_initial: The Selected mode in Main_ModeSelection at current step
%     Mode_previous_initial: The Selected mode in Main_ModeSelection at previous step
%             Mode_previous: The mode (1~12) of previous one step;
%          Posture_previous: The Posture (Position + Orientation + q11 + q21) of previous one step;
%           q0q1q2_previous: The joints value [q0, q11~q15, q21~q25] of previous one step;
%              Mode_current: The mode (1~12) of previous one step;
%           Posture_current: The Posture (Position + Orientation + q11 + q21) of previous one step;
%            q0q1q2_current: The joints value [q0, q11~q15, q21~q25] of previous one step;
%            NumIntepoPoint: Current intepotation number
%                Start_Time: the Intepotation start Time for current step
%              Time_inteval: the time interval for intepotation on each i step,  i = 1:length(MPOTP_cell)
%
%                            MPOTP_cell: Motion planning of Trajectory Points in cell mode with infos: {mode, {position, orientation, q11, q21}}
%        Self_adjustment_Enable_Disable: The correct order of differernt value with ' = 1/2/3/0'
%                                        1: The first to second mode, q11/q12 needs to be adjust
%                                        2: Random Mode to HomePosition, q11/q12 needs to be adjust
%                                        3: After Random Mode to mode, q11/q12 needs to be adjust
%                                        0: No needs to be adjust
%  Self_adjustment_Enable_Disable_Array: All possible self_adjustment assigned in each step
%
%                 Pos_Intep: Position intepotation for each i step
%                 Vel_Intep: Velocity intepotation for each i step
%                 Acc_Intep: Acceleration intepotation for each i step
%                time_Intep: Intepotation Time for each i step
% %
%              MP_Pos_Intep: Position intepotation for all steps of Moving Platform
%              MP_Vel_Intep: Velocity intepotation for all steps of Moving Platform
%              MP_Acc_Intep: Acceleration intepotation for all steps of Moving Platform
%             MP_time_Intep: Intepotation Time for all steps of Moving Platform
% %
%      q0q1q2_P2P_Pos_Intep: Angle intepotation for all steps of all input joint angles
%      q0q1q2_P2P_Vel_Intep: Angular velocity intepotation for all steps of all input joints
%      q0q1q2_P2P_Acc_Intep: Angular acceleration intepotation for all steps of all input joints
%     q0q1q2_P2P_time_Intep: Intepotation Time for all steps of all input joints 
L1 = l1;
L2 = l2;
%% Transition Strategy
[ MPOTP_cell, Self_adjustment_Enable_Disable_Array ] = TransitionStrategy(Mode_previous,PosOri_previous, q0q1q2_previous_trajpoint, Mode_current,PosOri_current, l1, l2);

%%
Pos_Intep = [];
Vel_Intep = []; 
Acc_Intep = []; 
time_Intep = [];

%
MP_Pos_Intep = [];
MP_Vel_Intep = [];
MP_Acc_Intep = [];
MP_time_Intep = [];
%
q0q1q2_P2P_Pos_Intep = [];
q0q1q2_P2P_Vel_Intep = [];
q0q1q2_P2P_Acc_Intep = [];
q0q1q2_P2P_time_Intep = [];
%
det_Jq_Normalized = [];
det_J_Normalized = [];
%
col = []; % Number and position of 'Self_adjustment_Enable_Disable == 1'
col_Self_adjustment = [];
q0q1q2_P2P_Pos_Intep = [];
Mode_previous_initial = Mode_previous;
PosOri_previous_initial = PosOri_previous;
Mode_current_initial = Mode_current;
PosOri_current_initial = PosOri_current;

for i = 1:length(MPOTP_cell)
    %% Assign value to "Self_adjustment_Enable_Disable == 0/1/2/3"
    Self_adjustment_Enable_Disable = Self_adjustment_Enable_Disable_Array(i); 
    % Here uses to get the first 1 for self-adjustment
    [~,col_Self_adjustment] = find(Self_adjustment_Enable_Disable_Array == 1);
    if isempty(col_Self_adjustment)
        col_Self_adjustment = 0;
    end
    %% Assgin "Mode_Previous, Mode_Current, PosOri_Current, PosOri_Previous"
    for OnlyUsedforFoldingThisPart_AssignValueofModeAndPosOri = 1:1
        if i == 1 || i == col_Self_adjustment(1)
            if  Self_adjustment_Enable_Disable == 1
                Mode_previous = MPOTP_cell{i}{1};
                PosOri_previous = MPOTP_cell{i}{2};
                Mode_current = MPOTP_cell{i+1}{1};
                PosOri_current = MPOTP_cell{i+1}{2};
                Mode = Mode_current;
            elseif  Self_adjustment_Enable_Disable == 2
                Mode_previous = MPOTP_cell{i}{1};
                PosOri_previous = MPOTP_cell{i}{2};
                Mode_current = MPOTP_cell{i+1}{1};
                PosOri_current = MPOTP_cell{i+1}{2};
                Mode = Mode_previous;
            elseif  Self_adjustment_Enable_Disable == 3
                Mode_previous;
                PosOri_previous;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2};   
                Mode = Mode_current;
            elseif  Self_adjustment_Enable_Disable == 0
                Mode_previous;
                PosOri_previous;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2};                
                Mode = Mode_current;
            end            
        elseif i > 1
            if  Self_adjustment_Enable_Disable == 1
                Mode_previous;
                PosOri_previous;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = PosOri_self_adjustment_1;
                Mode = Mode_previous;
            elseif  Self_adjustment_Enable_Disable == 2
                Mode_previous;
                PosOri_previous = PosOri_self_adjustment_2;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2};
                Mode = Mode_current;
            elseif  Self_adjustment_Enable_Disable == 3                
                Mode_previous = Mode_current;
                PosOri_previous = PosOri_current;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2}; 
                Mode = Mode_current;
            elseif  Self_adjustment_Enable_Disable == 0
                Mode_previous = Mode_current;
                PosOri_previous = PosOri_current;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2}; 
                Mode = Mode_current;
            end
        end       
                
        if Self_adjustment_Enable_Disable == 0 && Mode_current_initial == 7 && i == length(MPOTP_cell) && ...
                (Mode_previous_initial == 1  || Mode_previous_initial == 2) % Mode1/2 to 7
                PosOri_previous{5} = p(5);            
        end
        
    end
    
    %% Deal with Mode 5(HomePosition) to Mode 10/11(2RSerialAiCi) and  Mode 10/11(2RSerialAiCi) to Mode 5(HomePosition)
    for OnlyUsedforFoldingThisPart_Mode5andMode10and11 = 1:1
        % if random mode to Mode 10/11, so,
        % Firstly, it must recovery to Mode 5(HomePosition);
        % And then, execute Mode 5(HomePosition) to Mode 10/11(2RSerialAiCi)
        
        % Mode 5(HomePosition) to Mode 10/11(2RSerialAiCi)
        if (Self_adjustment_Enable_Disable == 1 || Mode_previous_initial == 5) && (Mode_current_initial == 10 || Mode_current_initial == 11)
            if Mode_current_initial == 10 
                q0q1q2_current_trajpoint = [0 MPOTP_cell{length(MPOTP_cell)-1}{2}{7} 0.139358669266875 3.14159265358979 -0.971371068617276 MPOTP_cell{length(MPOTP_cell)-1}{2}{7} 0 0.966880969134940 1.94661464276441 -0.603915357659957 0];
            elseif Mode_current_initial == 11 
                q0q1q2_current_trajpoint = [0 0 0.966880969134940 1.94661464276441 -0.603915357659957 0 MPOTP_cell{length(MPOTP_cell)-1}{2}{8} 0.139359146095574 3.14159265358979 -0.971371545445974 MPOTP_cell{length(MPOTP_cell)-1}{2}{8}];
            end
            %
            Length_Mode10or11 = length(MPOTP_cell);
            if i == Length_Mode10or11 - 2 && Mode_previous_initial ~= 5
                Mode_previous = MPOTP_cell{i}{1};
                PosOri_previous = MPOTP_cell{i}{2};
                Mode_current = MPOTP_cell{i+1}{1};
                PosOri_current = MPOTP_cell{i+1}{2};
            elseif i == Length_Mode10or11
                Mode_previous = MPOTP_cell{i}{1};
                if MPOTP_cell{i}{1} == 10                    
                    PosOri_previous = {MPOTP_cell{Length_Mode10or11 - 1}{2}{1}, MPOTP_cell{Length_Mode10or11 - 1}{2}{2}, [], [], [] ,[], q0q1q2_OptimalRow(NumIntepoPoints*(i-2),3)};
                    if Length_Mode10or11 > 3
                        PosOri_previous{7} = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),3);
                    end
                elseif MPOTP_cell{i}{1} == 11
                    PosOri_previous = {MPOTP_cell{Length_Mode10or11 - 1}{2}{1}, MPOTP_cell{Length_Mode10or11 - 1}{2}{2}, [], [], [] ,[], q0q1q2_OptimalRow(NumIntepoPoints*(i-1),8)};
                    if Length_Mode10or11 > 2
                        PosOri_previous{7} = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),8);                        
                    end
                end
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2};                
                for OnlyUsedforFoldingThisPart_Mode5toMode10and11 = 1:1
                    % ----------------------------------------------------------------------
                    % ---------- Adjust the q12/q22 from mode 5 to mode 10/11---------------
                    if  MPOTP_cell{i}{1} == 10 && Length_Mode10or11 > 1
                        % Keep the last two angles  q12 and q14  equals to the last second angles  q12 and q14
                        %--- Here we assign the value by extrapolation ----
                        % Delta = ((last third value - last Second Value) + (last Forth value - last third Value))/2
                        % Here we get the new "q0q1q2_OptimalRow(LastRowValue,7:11)" value by Extrapolation due to the mode switch (Singularity)
                        if Length_Mode10or11 == 3
                            Lenth = NumIntepoPoints;
                        else
                            Lenth = NumIntepoPoints* (Length_Mode10or11-2);
                        end
                        delta_q0q1q2 = (q0q1q2_OptimalRow(Lenth-2,:)-q0q1q2_OptimalRow(Lenth-1,:))/4 ;
                        q0q1q2_OptimalRow(Lenth,3) = q0q1q2_OptimalRow(Lenth-1,3) - delta_q0q1q2(3);
                        PosOri_previous{7} = q0q1q2_OptimalRow(Lenth,3);
                        po = {PosOri_previous{1}, PosOri_previous{2}, [], [], [], [], PosOri_previous{7}};
                        obj2RserialA1C1 = RCB2RserialA1C1(po,[],l1,l2);
                        [~, ~, ~, q1q2, ~] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
                        for jj = 1:length(q1q2(:,1))
                            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(Lenth-1,2:6));
                            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(Lenth-1,7:11));
                        end
                        [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
                        [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
                        q0q1q2_OptimalRow(Lenth,:) = [q0, q1q2(colsA1C1(1),1:5), q1q2(colsA2C2(1),6:10)];
                    elseif MPOTP_cell{i}{1} == 11 && Length_Mode10or11 > 1
                        % Keep the last two angles  q22 and q24  equals to the last second angles  q22 and q24
                        %--- Here we assign the value by extrapolation ----
                        % Delta = ((last third value - last Second Value) + (last Forth value - last third Value))/2
                        % Here we get the new "q0q1q2_OptimalRow(LastRowValue,7:11)" value by Extrapolation due to the mode switch (Singularity)
                        if Length_Mode10or11 == 2
                            Lenth = NumIntepoPoints;
                        elseif Length_Mode10or11 == 3
                            Lenth = NumIntepoPoints* (Length_Mode10or11-1);
                        else
                            Lenth = NumIntepoPoints* (Length_Mode10or11-2);
                        end
                        delta_q0q1q2 = (q0q1q2_OptimalRow(Lenth-2,:)-q0q1q2_OptimalRow(Lenth-1,:))/4 ;
                        q0q1q2_OptimalRow(Lenth,8) = q0q1q2_OptimalRow(Lenth-1,8) - delta_q0q1q2(8);
                        PosOri_previous{7} = q0q1q2_OptimalRow(Lenth,8);
                        po = {PosOri_previous{1}, PosOri_previous{2}, [], [], [], [], PosOri_previous{7}};
                        obj2RserialA2C2 = RCB2RserialA2C2(po,[],l1,l2);
                        [~, ~, ~, q1q2, ~] = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
                        for jj = 1:length(q1q2(:,1))
                            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(Lenth-1,2:6));
                            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(Lenth-1,7:11));
                        end
                        [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
                        [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
                        q0q1q2_OptimalRow(Lenth,1:11) = [q0, q1q2(colsA1C1(1),1:5), q1q2(colsA2C2(1),6:10)];
                    end
                    % -------------------------------------------------------------------
                end
                
                % Assign the correct value for the last self-adjustment to
                % right position
                Mode = Mode_current;
                if Mode == 10 || (Mode == 11 && Length_Mode10or11 == 4)
                    q0q1q2_previous_trajpoint = q0q1q2_OptimalRow(NumIntepoPoints*(length(MPOTP_cell)-2),:);
                end
                %
            end            
        end
            
        % Mode 10/11(2RSerialAiCi) to Mode 5(HomePosition)
        if Self_adjustment_Enable_Disable == 2 && (Mode_previous_initial == 10 || Mode_previous_initial == 11)
            if Mode_previous_initial == 10 && Mode_current_initial ~= 5
                q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0 MPOTP_cell{length(MPOTP_cell)-1}{2}{7} 0.139358669266875 3.14159265358979 -0.971371068617276 MPOTP_cell{length(MPOTP_cell)-1}{2}{7} -3.14159265358979 0.966880969134940 1.94661464276441 -0.603915357659957 -3.14159265358979];
                if MPOTP_cell{length(MPOTP_cell)-1}{1} == 5
                    q0q1q2_previous_trajpoint = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_previous_trajpoint];
                end
            elseif Mode_previous_initial == 11 && Mode_current_initial ~= 5
                q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0 0 0.966880969134940 1.94661464276441 -0.603915357659957 0 MPOTP_cell{length(MPOTP_cell)-1}{2}{8} 0.139358669266875 3.14159265358979 -0.971371068617276 MPOTP_cell{length(MPOTP_cell)-1}{2}{8}];
                if MPOTP_cell{length(MPOTP_cell)-1}{1} == 5 || Mode_current_initial == 7
                    q0q1q2_previous_trajpoint = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_previous_trajpoint];
                end
            end
            if i == 1
                % Here we deliver two values on each step end of q0q1q2_mat to q0q1q2_previous_trajpoint
                q0q1q2_previous_i_1 = q0q1q2_previous_trajpoint(1,:);
                q0q1q2_previous_i_2 = q0q1q2_previous_trajpoint(2,:);
                q0q1q2_previous_trajpoint = q0q1q2_previous_i_2;
                Mode_previous = MPOTP_cell{1}{1};
                PosOri_previous = MPOTP_cell{1}{2};
                Mode_current = MPOTP_cell{1}{1};
                if MPOTP_cell{1}{1} == 10
                    PosOri_current = {MPOTP_cell{2}{2}{1}, MPOTP_cell{2}{2}{2}, [], [], [], [], q0q1q2_previous_i_1(3)};
                elseif MPOTP_cell{1}{1} == 11
                    PosOri_current = {MPOTP_cell{2}{2}{1}, MPOTP_cell{2}{2}{2}, [], [], [], [], q0q1q2_previous_i_1(8)};
                end
            elseif i == 2
                Mode_previous = MPOTP_cell{2}{1};
                PosOri_previous = MPOTP_cell{2}{2};
                Mode_current = MPOTP_cell{3}{1};
                PosOri_current = MPOTP_cell{3}{2};
                q0q1q2_previous_trajpoint = q0q1q2_previous_i_1;
            end
        end
         
    end
            
    %% Intepotation and Trajectory Planning
    for OnlyUsedforFoldingThisPart_TrajectoryPlanning = 1:1
        Time = [(i-1) * Time_inteval, i * Time_inteval] + Start_Time * [1 1];
        
        [ Pos_Intep, Vel_Intep, Acc_Intep, time_Intep ] =  ...
            IntepotationP2P(Mode, PosOri_previous,q0q1q2_previous_trajpoint, PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, Time, l1, l2);
        
        % Position, Velocity, Acceleration Calcuation
        if length(Pos_Intep(:,1)) == 6
            Pos_Intep(7:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Vel_Intep(7:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Acc_Intep(7:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)]; 
        elseif length(Pos_Intep(:,1)) == 7
            Pos_Intep(8:10,:) = [ zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Vel_Intep(8:10,:) = [ zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Acc_Intep(8:10,:) = [ zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];              
        elseif length(Pos_Intep(:,1)) == 8
            Pos_Intep(9:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Vel_Intep(9:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Acc_Intep(9:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];    
        end
        MP_Pos_Intep = [MP_Pos_Intep; Pos_Intep']; %MP: moving Platform
        MP_Vel_Intep = [MP_Vel_Intep; Vel_Intep'];
        MP_Acc_Intep = [MP_Acc_Intep; Acc_Intep'];
        MP_time_Intep = [MP_time_Intep; time_Intep'];
    end
    
    %% Iterative IK solution of all intepolation points
    for j = 1:NumIntepoPoints
        %
        if Mode == Mode_current
            PosOri_Effect = PosOri_current;
        elseif Mode == Mode_previous
            PosOri_Effect = PosOri_previous;
        end
        PosOri = {};
        for k = 1:length(PosOri_Effect)
            if isempty(PosOri_Effect{k}) ~= 1
                PosOri{k} = Pos_Intep(k,j);
            else
                PosOri{k} = [];
            end
        end

        for OnlyUsedforFoldingThisPart_TrajectoryPlanning = 1:1
            % We assign the q11 and q21 with previous one step value
            if i == 1
                if j == 1
                    % only used for the first step from HomePosition to other mode
                    q0 = q0q1q2_previous_trajpoint(1);
                    q11 = q0q1q2_previous_trajpoint(2);
                    q21 = q0q1q2_previous_trajpoint(7);
                    q0q1q2_OptimalRow(1,:) = q0q1q2_previous_trajpoint;
                else
                    %if self-adjustment exists
                    q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
                    if Mode == 3
                        q11 = PosOri{7};
                        q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                    elseif Mode == 4
                        q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                        q21 = PosOri{8};
                    elseif Mode == 5
                        if Mode_previous == 5 && Mode_previous == Mode_current
                            q11 = PosOri{7};
                            q21 = PosOri{8};
                        else
                            q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                            q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                        end
                    else
                        q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                        q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                    end
                end
            else
                q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
                if j == 1
                    if Self_adjustment_Enable_Disable == 1
                        if Self_adjustment_Enable_Disable_Array(i-1) ~= 1
                            q0q1q2_previous_trajpoint = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),:);
                            q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_previous_trajpoint;
                        else
                            q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_previous_trajpoint;
                        end
                    else
                        if Self_adjustment_Enable_Disable_Array(i-1) == 1
                            % Here we deal with the situation of this Self_adjustment_Enable_Disable = 1 (Should be [1 1] first), next Self_adjustment_Enable_Disable = 2/3/0
                            q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_OptimalRow(NumIntepoPoints*(i-2),:);
                        else
                            q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),:);
                        end
                        q0q1q2_Optimal_SingleRow = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:);
                    end
                end
                if length(PosOri_Effect) == 7 && Mode == 10
                    q11 = PosOri{7};
                    q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                elseif length(PosOri_Effect) == 7 && Mode == 11
                    q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                    q21 = PosOri{7};
                elseif length(PosOri_Effect) == 6
                    q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                    q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                else
                    if isempty(PosOri{7}) == 1
                       PosOri{7} = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,2);
                    end
                    if isempty(PosOri{8}) == 1
                       PosOri{8} = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,7);
                    end
                    q11 = PosOri{7};
                    q21 = PosOri{8};
                end
            end
        end
        
        % IK solution
        [ p, ~, ~, q1q2, ~ ] = IK(Mode, PosOri, q0, q11, q21, q0q1q2_OptimalRow(length(q0q1q2_OptimalRow(:,1)),:), l1, l2);
        q0q1q2_CurrentStep = [zeros(length(q1q2(:,1)),1),q1q2];

        % Optimal Joints Solution
        OptimalJointsSolution;
        q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j,:) = q0q1q2_Optimal_SingleRow;   
        %
        PosOri_Output{NumIntepoPoints*(i-1)+j,1} = PosOri;
        MP_Pos_Intep(NumIntepoPoints*(i-1)+j,1:6) = p;
        
        % ==================  Jacobian Matrix  ====================
        q1q2_Optimal = q0q1q2_Optimal_SingleRow(:,2:11);
        q11 = q1q2_Optimal(1); q12 = q1q2_Optimal(2); q13 = q1q2_Optimal(3); q14 = q1q2_Optimal(4); q15 = q1q2_Optimal(5);
        q21 = q1q2_Optimal(6); q22 = q1q2_Optimal(7); q23 = q1q2_Optimal(8); q24 = q1q2_Optimal(9); q25 = q1q2_Optimal(10);
        
        if  Mode == 1
            Enable_Mode_JacoMat = 1;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq1_Ob_3T1R = det(Jq1_Ob_3T1R)-------
            det_Jq(j,:) = det(Jq1_Ob_3T2R); 
            %-------det_J_Ob_3T1R(NumIntepoPoints*(i-1)+j,:) = det(J_Ob_3T1R)----
            det_J(j,:) = 0;%det(J_Ob_3T2R);
            % ------Jc_Ob_3T1R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 0;
        elseif  Mode == 2
            Enable_Mode_JacoMat = 2;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq1_Ob_3T1R = det(Jq1_Ob_3T1R)-------
            det_Jq(j,:) = det(Jq1_Ob_3T1R); 
            %-------det_J_Ob_3T1R(NumIntepoPoints*(i-1)+j,:) = det(J_Ob_3T1R)----
            det_J(j,:) = det(J_Ob_3T1R);
            % ------Jc_Ob_3T1R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = det(Jc_Ob_3T1R);
        elseif  Mode == 5
            Enable_Mode_JacoMat = 5;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq1_Ob_3T1R = det(Jq1_Ob_3T1R)-------
            det_Jq(j,:) = 0; % scaled
            %-------det_J_Ob_3T1R(NumIntepoPoints*(i-1)+j,:) = det(J_Ob_3T1R)----
            det_J(j,:) = 0;
            % ------Jc_Ob_3T1R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = det(Jc_Ob_HomePosition);
        elseif Mode == 6 || Mode == 7 || Mode == 8 || Mode == 9
            Enable_Mode_JacoMat = 6;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq2_Ob_2T2R = det(Jq2_Ob_2T2Rsixbar)-------
            det_Jq(j,:) = det(Jq2_Ob_2T2Rsixbar); % scaled
            %-------det_J_Ob_2T2R = det(J_Ob_2T2R)-------
            det_J(j,:) = det(J_Ob_2T2Rsixbar);
            % ------Jc_Ob_2T2R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        elseif Mode == 10
            Enable_Mode_JacoMat = 10;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq2_Ob_2T2R = det(Jq2_Ob_2Rsixbar)-------
            det_Jq(j,:) = det(Jq2_Ob_2RSerialChainA1C1); % scaled
            %-------det_J_Ob_2R = det(J_Ob_2T2R)-------
            det_J(j,:) = det(J_Ob_2RSerialChainA1C1);
            % ------Jc_Ob_2T2R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        elseif Mode == 11
            Enable_Mode_JacoMat = 11;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq2_Ob_2T2R = det(Jq2_Ob_2Rsixbar)-------
            det_Jq(j,:) = det(Jq2_Ob_2RSerialChainA2C2); % scaled
            %-------det_J_Ob_2R = det(J_Ob_2T2R)-------
            det_J(j,:) = det(J_Ob_2RSerialChainA2C2);
            % ------Jc_Ob_2T2R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        else
            det_Jq(j,:) = 1;
            det_J(j,:) = 1;
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        end
        
        if abs(det_J(j,:)) < 1e-12
            det_J(j,:) = 0;
        end
                
        Mode_Ideal(NumIntepoPoints*(i-1)+j,:) = Mode;

        %------ Show Center point of Moving Platform -------
        %Displacement = [250,250,165.88];
        %p_Base = p(1:3) + Displacement;
        %plot3(p_Base(1),p_Base(2),p_Base(3),'r.');
    end    
    
    %------------------------------------------------------ 
    % det_Jq_Normalized =  det_Jq / max(abs(det_Jq))
    [row,~] = find(abs(det_Jq) == max(abs(det_Jq)));
    if max(abs(det_Jq)) == 0
        det_Jq_Normalized = [det_Jq_Normalized; det_Jq];
    else
        det_Jq_Normalized = [det_Jq_Normalized; det_Jq/abs(det_Jq(row(1)))];
    end
    % det_J_Normalized =  det_J / max(abs(det_J))
    [row,~] = find(abs(det_J) == max(abs(det_J)));
    if max(abs(det_J)) == 0
        det_J_Normalized = [det_J_Normalized; det_J];
    else
        det_J_Normalized = [det_J_Normalized; det_J/abs(det_J(row(1)))];
    end    
    
    %------------------------------------------------------ 
    % Output current acutall modes
    % We define:
    for j = 1:NumIntepoPoints
        if ( Mode ~= 5 && abs(det_Jq_Normalized(NumIntepoPoints*(i-1)+j,:) ) < 0.3) ||...
                ( Mode ~= 1 && abs(det_Jc(NumIntepoPoints*(i-1)+j,:)) < 0.3 )
            Mode_Actual(NumIntepoPoints*(i-1)+j,:) = 0;
        else
            Mode_Actual(NumIntepoPoints*(i-1)+j,:) = Mode;
        end
    end
    % ===========================  End  ============================
    
    %% Self_adjustment mode contain three types:
    for OnlyUsedforFoldingThisPart_Self_adjustment = 1:1
        % 1. 'Self_adjustment_Enable_Disable == 1': From HomePosition to other mode
        % 2. 'Self_adjustment_Enable_Disable == 2': From other mode to HomePosition
        % 3. 'Self_adjustment_Enable_Disable == 3': Random Mode to Random Mode (without the situations 1 and 2)
        % 4. 'Self_adjustment_Enable_Disable == 0': No self-adjustment
        if  Self_adjustment_Enable_Disable == 1             
            % Here is used for executing 'Self_adjustment' only once, length(MPOTP_cell) > 3 means Modes10/11;
            if col_Self_adjustment(1) == 1
                if Mode == 3
                    PosOri_current{7} = PosOri_previous{7};
                    PosOri_current{8} = q0q1q2_OptimalRow(2,7);
                elseif Mode == 4
                    PosOri_current{7} = q0q1q2_OptimalRow(2,2);
                    PosOri_current{8} = PosOri_previous{8};
                else
                    PosOri_current{7} = q0q1q2_OptimalRow(2,2);
                    PosOri_current{8} = q0q1q2_OptimalRow(2,7);
                end
            elseif col_Self_adjustment(1) == length(MPOTP_cell) - length(col_Self_adjustment) + 1
                if Mode_current_initial == 10 || Mode_current_initial == 11
                    PosOri_current{7} = q0q1q2_OptimalRow((i-1)*NumIntepoPoints + 2,2);
                    PosOri_current{8} = q0q1q2_OptimalRow((i-1)*NumIntepoPoints + 2,7);
                    PosOri_previous{7} = q0q1q2_OptimalRow((i-1)*NumIntepoPoints,2);
                    PosOri_previous{8} = q0q1q2_OptimalRow((i-1)*NumIntepoPoints,7);
                else
                    PosOri_current{7} = q0q1q2_OptimalRow((i-1)*NumIntepoPoints + 2,2);
                    PosOri_current{8} = q0q1q2_OptimalRow((i-1)*NumIntepoPoints + 2,7);
                end
            end
            PosOri_self_adjustment_1 = {PosOri_previous{1:6},PosOri_current{7:8}};
            if i > 1 % Here we deal with the situation of this Self_adjustment_Enable_Disable = 1 (Should be [1 1] first), next Self_adjustment_Enable_Disable = 2/3/0 
                PosOri_current = {MPOTP_cell{i}{2}{1}, MPOTP_cell{i}{2}{2}, MPOTP_cell{i}{2}{3}, MPOTP_cell{i}{2}{4}, MPOTP_cell{i}{2}{5}, MPOTP_cell{i}{2}{6}};
                if isempty(find(Self_adjustment_Enable_Disable_Array(i:length(MPOTP_cell)) == 2, 1)) ~= 1
                    PosOri_self_adjustment_2 = {PosOri_current{1:6},q0q1q2_OptimalRow(NumIntepoPoints*(i-1),2), q0q1q2_OptimalRow(NumIntepoPoints*(i-1),7)};
                elseif isempty(find(Self_adjustment_Enable_Disable_Array(i:length(MPOTP_cell)) == 3, 1)) ~= 1
                    PosOri_current = {PosOri_current{1:6},q0q1q2_OptimalRow(NumIntepoPoints*(i-1),2), q0q1q2_OptimalRow(NumIntepoPoints*(i-1),7)};
                end
            end
            if i == col_Self_adjustment(1) % Modes 1/2 to Modes 10/11
                                           % Mode 3 to Mode 7
                q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+2,:);
            end
        elseif Self_adjustment_Enable_Disable == 2        
            if Mode == 3
                q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
                q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
            elseif Mode == 4
                q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
                q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
            elseif Mode == 7
                q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
                q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
                q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
                q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
            elseif Mode_previous_initial == 11 && Mode_current_initial == 7  % Mode 11 to Mode 7                 
                if i == 1
                    MPOTP_cell_Target = MPOTP_cell{3};
                    MPOTP_cell{3}{1} = 5;
                    MPOTP_cell{3}{2} = {0, 0, 208.879343162506 0 [] 0, 0, 0};
                elseif i == 2
                    MPOTP_cell{3}{1} = 7;
                    MPOTP_cell{3}{2} = MPOTP_cell_Target{2};
                end
            elseif Mode_previous_initial == 11 && Mode_current_initial == 5  % Mode 11 to Mode 5                 
                if i == 1
                    MPOTP_cell_Target = MPOTP_cell{3};
                    MPOTP_cell{3}{1} = 5;
                    MPOTP_cell{3}{2} = {0, 0, 208.879343162506 0 [] 0, 0, 0};
                elseif i == 2
                    MPOTP_cell{3}{1} = 5;
                    MPOTP_cell{3}{2} = MPOTP_cell_Target{2};
                end
            end
            PosOri_current{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
            PosOri_current{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            if i > 1 %Here we deal with the situation of this Self_adjustment_Enable_Disable = 2, next Self_adjustment_Enable_Disable = 1 
                PosOri_self_adjustment_1 = {PosOri_previous{1:6},q0q1q2_OptimalRow((i-1)*NumIntepoPoints + 2,2), q0q1q2_OptimalRow((i-1)*NumIntepoPoints + 2,7)};
            end
            PosOri_self_adjustment_2 = PosOri_current;
        elseif Self_adjustment_Enable_Disable == 3
            if Mode_current_initial == 3
                if i > 1 && Self_adjustment_Enable_Disable_Array(i-1) == 1
                    % if previous step is Self_adjustment_Enable_Disable == 1,
                    % Namely, Mode 3/4 to Mode 4/3
                else
                    q0q1q2_OptimalRow(NumIntepoPoints*i,2) = q0q1q2_OptimalRow(NumIntepoPoints*i - 1,2);
                end
            elseif Mode_current_initial == 4
                if i > 1 && Self_adjustment_Enable_Disable_Array(i-1) == 1
                    % if previous step is Self_adjustment_Enable_Disable == 1,
                    % Namely, Mode 3/4 to Mode 4/3
                else
                    q0q1q2_OptimalRow(NumIntepoPoints*i,7) = q0q1q2_OptimalRow(NumIntepoPoints*i - 1,7);
                end
            elseif Mode_current_initial == 10 || Mode_current_initial == 11
                MPOTP_cell{length(MPOTP_cell)-2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{length(MPOTP_cell)-2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            else                
            end
            PosOri_current{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
            PosOri_current{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7); 
        elseif Self_adjustment_Enable_Disable == 0 
            if Mode_previous_initial == 9 && Mode_current_initial ~= 12
                PosOri_current{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                PosOri_current{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);                
            elseif (Mode_previous_initial == 1 || Mode_previous_initial == 2) &&...
                    (Mode_current_initial == 7 || Mode_current_initial == 8 || Mode_current_initial == 9 || Mode_current_initial == 10 || Mode_current_initial == 11) 
                % Mode 1/2 to Mode 7/8, length(MPOTP_cell)>1
                MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            elseif ((Mode_previous_initial == 3 || Mode_previous_initial == 4) && Mode_current_initial == 7) %|| (Mode_previous_initial == 5 && Mode_current_initial == 11)
                     % Mode 3/4 to Mode 7; Mode5 to Mode 11, length(MPOTP_cell)>1
                MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            elseif Mode_previous_initial == 6 && Mode_current_initial == 7 % Mode 6 to Mode 7, length(MPOTP_cell)>1
                MPOTP_cell{1}{2}{5} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                PosOri_current{5} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
            elseif (Mode_previous_initial ~= 12 && Mode_previous_initial ~= 5) && Mode_current_initial == 12 % Mode 3 to Mode 10, length(MPOTP_cell)>1
                if MPOTP_cell{length(MPOTP_cell)}{1} ~= 12 || (i == 1 && Mode_previous_initial ~= 10 && Mode_previous_initial ~= 11)
                    PosOri_current{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                    PosOri_current{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                    MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                    MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                else
                    if (Mode_previous_initial == 10 || Mode_previous_initial == 11) && i == 1
                        q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints-1,2);
                        q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints-1,2);
                        q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints-1,7);
                        q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints-1,7);
                    else
                        MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                        MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,3);
                        MPOTP_cell{2}{2}{9} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                        MPOTP_cell{2}{2}{10} = q0q1q2_OptimalRow(NumIntepoPoints*i,8);
                    end                    
                    PosOri_current{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                    PosOri_current{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,3);
                    PosOri_current{9} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                    PosOri_current{10} = q0q1q2_OptimalRow(NumIntepoPoints*i,8);
                end                
            elseif (Mode_previous_initial == 3 || Mode_previous_initial == 4) && (Mode_current_initial == 10 || Mode_current_initial == 11) % Mode 3/4 to Mode 10/11, length(MPOTP_cell)>1
                MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            elseif (Mode_previous_initial == 6 || Mode_previous_initial == 7) && (Mode_current_initial == 3 || Mode_current_initial == 4) % Mode 6/7 to Mode 10/11, length(MPOTP_cell)>1
                MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            elseif (Mode_previous_initial == 7 && Mode_current_initial == 2) || ...
                    (Mode_previous_initial == 6 && (Mode_current_initial == 8 || Mode_current_initial == 11)) %Mode 7 to 2; Mode 6 to Modes 8/11
                MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            elseif (Mode_previous_initial == 8 || Mode_previous_initial == 9) && ... 
                    (Mode_current_initial == 3 || Mode_current_initial == 4 || Mode_current_initial == 7 || Mode_current_initial == 9)
                MPOTP_cell{2}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                MPOTP_cell{2}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            elseif (Mode_previous_initial == 10 && Mode_current_initial == 11) || ...
                    (Mode_previous_initial == 11 && Mode_current_initial == 10)                
                if i == 1
                    q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints-1,2);
                    q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints-1,2);
                    q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints-1,7);
                    q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints-1,7);
                end                
                PosOri_current{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                PosOri_current{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,3);
                PosOri_current{9} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                PosOri_current{10} = q0q1q2_OptimalRow(NumIntepoPoints*i,8);
                %----- Mode 11 to Mode 10
                if Mode_previous_initial == 11
                    if i == 2 || i == 3
                        PosOri_current{7} = 0;
                    end
                end
                %-----
            elseif Mode_previous_initial == 12
                if Mode_current_initial == 1 || Mode_current_initial == 2
                    MPOTP_cell{length(MPOTP_cell)}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                    MPOTP_cell{length(MPOTP_cell)}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                elseif Mode_current_initial == 3
                    MPOTP_cell{length(MPOTP_cell)}{2}{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
                elseif Mode_current_initial == 4
                    MPOTP_cell{length(MPOTP_cell)}{2}{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
                elseif Mode_current_initial == 10
                    PosOri_current{7} = 0;
                    MPOTP_cell{1}{2}{7} = 0;
                end
            end
        end
    end
    
end

%% Output current all modes, Jacobian of Jq and J
% Mode_det_Jq_Jc_J = Mode_Actual;
Mode_det_Jq_Jc_J = [Mode_Ideal, Mode_Actual, det_Jq_Normalized, det_Jc, det_J_Normalized];% det_J
Mode_det_Jq_Jc_J_Intep = Mode_det_Jq_Jc_J;

%% Assign the correct order of differernt value of 'Self_adjustment_Enable_Disable = 1/2/3/0'
q0q1q2_P2P_Pos_Intep = q0q1q2_OptimalRow;
PosOri_Output_Intep = PosOri_Output;
MP_Pos = MP_Pos_Intep;
MP_Vel = MP_Vel_Intep;
MP_Acc = MP_Acc_Intep;
[~,col] = find(Self_adjustment_Enable_Disable_Array == 1);
if isempty(col) ~= 1 
    % Position
    q0q1q2_P2P_Pos_Intep(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:) = q0q1q2_OptimalRow((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:);
    q0q1q2_P2P_Pos_Intep((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:) = q0q1q2_OptimalRow(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:);
    % Position
    PosOri_Output_Intep(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:) = PosOri_Output((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:);
    PosOri_Output_Intep((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:) = PosOri_Output(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:);
    % Mode and Jacobian
    Mode_det_Jq_Jc_J_Intep(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:) = Mode_det_Jq_Jc_J((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:);
    Mode_det_Jq_Jc_J_Intep((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:) = Mode_det_Jq_Jc_J(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:);
    
    % Moving Platform
    %Position
    MP_Pos_Intep(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:) = MP_Pos((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:);
    MP_Pos_Intep((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:) = MP_Pos(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:);
    %Velocity
    MP_Vel_Intep(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:) = MP_Vel((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:);
    MP_Vel_Intep((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:) = MP_Vel(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:);
    %Acceleration
    MP_Acc_Intep(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:) = MP_Acc((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:);
    MP_Acc_Intep((NumIntepoPoints*col(1)+1):col(2)*NumIntepoPoints,:) = MP_Acc(NumIntepoPoints*(col(1)-1)+1:NumIntepoPoints*col(1),:);
    
end

%% Position, Velocity, Acceleration Calcuation of joint angles
% Velocity Calcuation
for i = 1:length(MPOTP_cell)
    for j = 1:NumIntepoPoints
        if j == 1 || j == NumIntepoPoints
            q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j,:) =  zeros(1,length(q0q1q2_P2P_Pos_Intep(1,:)));
        else
            q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j,:) = ...
            ( q0q1q2_P2P_Pos_Intep((i-1)*NumIntepoPoints + j + 1,:) - q0q1q2_P2P_Pos_Intep((i-1)*NumIntepoPoints + j,:) ) / ...
            (       MP_time_Intep((i-1)*NumIntepoPoints + j + 1, 1) - MP_time_Intep((i-1)*NumIntepoPoints + j, 1)       ); 
        end
    end
end

% Acceleration Calcuation
for i = 1:length(MPOTP_cell)
    for j = 1:NumIntepoPoints
        if j == 1 || j == NumIntepoPoints
            q0q1q2_P2P_Acc_Intep((i-1)*NumIntepoPoints + j,:) =  zeros(1,length(q0q1q2_P2P_Pos_Intep(1,:)));
        else
            q0q1q2_P2P_Acc_Intep((i-1)*NumIntepoPoints + j,:) = ...
            ( q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j + 1,:) - q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j,:)  ) / ...
            (       MP_time_Intep((i-1)*NumIntepoPoints + j + 1, 1) - MP_time_Intep((i-1)*NumIntepoPoints + j, 1)        ); 
        end
    end
end

end
