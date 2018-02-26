%%
addpath(genpath(pwd));
InitHome;
%%
Mode=10;

l1 = 230.1390;
l2_min = 147.7;
l2_max = l1-10;
L_Rate_min = l2_min/l1;
L_Rate_max=(l1-10)/l1;
p_0 = [250,250,83.5+60.44+(45.5-22),0];

deg = pi/180;

%%
% ---- 3T1R ------
angle1 = [-2*pi/3, 2*pi/3]; angle2 = [0, pi/2];
stepAngle = 30;

OriginalPoint_Displacement = [250,250,83.5+60.44+(45.5-22)];
TransCuboidPath2OriginalPoint = [[rotz(90);0,0,0], [OriginalPoint_Displacement, 1]'];

for i_angle2 = 1:1:stepAngle
        angle2Intepetation(i_angle2) = angle2(1) + i_angle2 * (angle2(2) - angle2(1)) / stepAngle;
        zIntepetation(i_angle2) = l1 * sin(angle2Intepetation(i_angle2)) / 2;
        for i_angle1 = 1:1:stepAngle            
            angle1Intepetation(i_angle1) = angle1(1) + i_angle1 * (angle1(2) - angle1(1)) / stepAngle;
            if Mode == 10 % --- A1C1 ---
                xIntepetation(i_angle1) = - l1 * cos(angle2Intepetation(i_angle2)) * sin(angle1Intepetation(i_angle1)) / 2;
                yIntepetation(i_angle1) = l1 * (cos(angle2Intepetation(i_angle2)) * cos(angle1Intepetation(i_angle1)) - 1) / 2;                
            elseif Mode == 11 % --- A2C2 ---
                xIntepetation(i_angle1) = l1 * cos(angle2Intepetation(i_angle2)) * sin(angle1Intepetation(i_angle1)) / 2;
                yIntepetation(i_angle1) = l1 * (1 - cos(angle2Intepetation(i_angle2)) * cos(angle1Intepetation(i_angle1))) / 2;
            end
        end
        SpherePath((1 + stepAngle * (i_angle2 -1)):i_angle2  * stepAngle,:) = [xIntepetation',yIntepetation',zIntepetation(i_angle2)*ones(stepAngle,1)];
   end
    
pathOP = [SpherePath(:,1:3),ones(length(SpherePath(:,1)),1)] * TransCuboidPath2OriginalPoint'; % pathOP: path output
sizepathOP = size(pathOP);
xyzStepLength = [1, stepAngle, stepAngle];

%% Judge the exsitence of each point with angle consideration

Length_Step_Num = 20;
Jacobi_Val = [];
L_Change_Rate = (L_Rate_max-L_Rate_min)/Length_Step_Num;
h = waitbar(0,'Please wait...');

for Length_Step = 1:Length_Step_Num
    
    L_Rate = L_Rate_min + L_Change_Rate*Length_Step;
    l2 = l1*L_Rate;
    L1 = l1;
    L2 = l2;
    WSvalue = [];
    det_J_SinglePoint = [];
    det_J_Max_Sniglepoint = [];
    step = 0;
    NumExistedPoint = 0;
    
    tic
    for k = 1:1:xyzStepLength(3)
        for j = 1:1:xyzStepLength(2)
            for i = 1:1:xyzStepLength(1)            
                step = step + 1;
                 % 3T1R mode:  [1 1 1 1 0 0]
                 % p = [x, y, z, alpha, [], []];

                po = {SpherePath(step,1), SpherePath(step,2), SpherePath(step,3), [], [], [], 0};
                q11q12q22q13 = [];
                obj2RserialA1C1 = RCB2RserialA1C1_WS(po,q11q12q22q13,l1,l2);
                [p, ~, ~, q1q2, WSvalue_2RserialA1C1] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
                WSvalue(i,j,k) = WSvalue_2RserialA1C1(1);

                if WSvalue(i,j,k) == 1
                    NumExistedPoint = NumExistedPoint + 1;
                    WS_po_value(NumExistedPoint, 1:3) = pathOP(step,1:3);
                    q1q2_row=length(q1q2(:,1));
                    det_J_SinglePoint = [];

                    for q1q2_row_length=1:q1q2_row

                            q11 = q1q2(q1q2_row_length,1); q12 = q1q2(q1q2_row_length,2); q13 = q1q2(q1q2_row_length,3); q14 = q1q2(q1q2_row_length,4); q15 = q1q2(q1q2_row_length,5);
                            q21 = q1q2(q1q2_row_length,6); q22 = q1q2(q1q2_row_length,7); q23 = q1q2(q1q2_row_length,8); q24 = q1q2(q1q2_row_length,9); q25 = q1q2(q1q2_row_length,10);

                            Enable_Mode_JacoMat = 10;
                            UnifiedJacobianMatrix_ScrewTheory;
                            det_J_SinglePoint(q1q2_row_length) = det(J_Ob_2RSerialChainA1C1);

                                if abs(det_J_SinglePoint(q1q2_row_length)) < 1e-12
                                    det_J_SinglePoint(q1q2_row_length) = 0;
                                end

                    end      
                    det_J_Max_Sniglepoint(NumExistedPoint,1)=max(det_J_SinglePoint);
                else
                end
            end
        end
    end
    
    WS_po_value_Points_temp=WS_po_value(:,1:3);
    [WS_po_value_Points,ia,ic]=unique(WS_po_value_Points_temp(:,1:3),'rows');
    
    [k, volume] = boundary(WS_po_value_Points,1);
    
    det_J_avg=sum(det_J_Max_Sniglepoint)/length(det_J_Max_Sniglepoint);
    
    Jacobi_Val(Length_Step,1)=L_Rate;
    Jacobi_Val(Length_Step,2)=det_J_avg;
    Volume(Length_Step,1)=L_Rate;
    Volume(Length_Step,2)=volume;
    
    waitbar(Length_Step/Length_Step_Num)  

end
close(h);

%clearvars -except Jacobi_Val Volume;

%%
figure(1)
subplot(2,1,1)
plot(Jacobi_Val(:,1),Jacobi_Val(:,2),'r');
xlabel('l2/l1');ylabel('det(Jacobian)');
hold on;
subplot(2,1,2)
plot(Volume(:,1),Volume(:,2)/10e6,'b');
xlabel('l2/l1');ylabel('Volumen/dm3');
hold on;

%%
[k, volume] = boundary(WS_po_value_Points,1);
plot3(WS_po_value(:,1), WS_po_value(:,2), WS_po_value(:,3),'b.','MarkerSize',5);
axis equal;
grid on;
hold on; 

 
