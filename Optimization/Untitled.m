All_Points=length(WS_po_value);
det_J_Max_Sniglepoint=[];
h = waitbar(0,'Please wait...');

for j=1:1
    
    po = {WS_po_value(j,1), WS_po_value(j,2), WS_po_value(j,3), WS_po_value(j,4), [], [], [], []};
    q11q12q21q22 = [];
    obj3T1R = RCB3T1R_WS(po, q11q12q21q22, l1, l2);
    [~, ~, ~, q1q2, ~] = obj3T1R.RCB_3T1R_IK;

    q1q2_row=length(q1q2(:,10));
    det_J_SinglePoint=[];

        for i=1:q1q2_row

                q11 = q1q2(i,1); q12 = q1q2(i,2); q13 = q1q2(i,3); q14 = q1q2(i,4); q15 = q1q2(i,5);
                q21 = q1q2(i,6); q22 = q1q2(i,7); q23 = q1q2(i,8); q24 = q1q2(i,9); q25 = q1q2(i,10);

                Enable_Mode_JacoMat = 2;
                UnifiedJacobianMatrix_ScrewTheory;
                det_J_SinglePoint(i) = det(J_Ob_3T1R);

                    if abs(det_J_SinglePoint(i)) < 1e-12
                        det_J_SinglePoint(i) = 0;
                    end

        end      

det_J_Max_Sniglepoint(j,1)=max(det_J_SinglePoint);
waitbar(j/All_Points)

end