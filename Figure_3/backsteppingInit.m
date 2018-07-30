function [ ControlData, Data] = backsteppingInit( Data )
        Data.ControlName='bs';
        ControlData.IsCompensate = 1; % ÊÇ·ñ²¹³¥
        ControlData.Bound = [ 1;1;32;0.65;0.65;0.65]*1;
        ControlData.IsBound =1;
        ControlData.Solve = 1; 
        ControlData.IsFilter = 1;  % ÂË²¨Æ÷¿ª¹Ø
        
        % Backstepping Postion
        ControlData.bsParamsP.ki_1 = [ 3; 3; 3 ]*0.01;
        ControlData.bsParamsP.ki_2 = [ 3; 3; 3 ]*0.01;
        ControlData.bsStatesP.SumZ_1=[ 0;0;0 ];
        ControlData.bsStatesP.SumZ_2=[ 0;0;0 ];
        
        ControlData.bsParamsP.Alpha1 =[ 12; 12; 12 ]*0.001;
        ControlData.bsParamsP.Alpha2 =[ 12; 12; 12 ]*0.01;
        ControlData.bsParamsP.F1=@bsf1_P;
        ControlData.bsParamsP.F2=@bsf2_P;
        ControlData.bsParamsP.G1=@bsg1_P;
        ControlData.bsParamsP.G2=@bsg2_P;
        ControlData.bsParamsP.dt =Data.dt;
        
        % Attitude
        ControlData.bsParamsR.Alpha1 =[12;12;12]*0.001;
        ControlData.bsParamsR.Alpha2 =[12;12;12]*0.01;
        ControlData.bsParamsR.ki_1 = [1;1;1]*0.02;
        ControlData.bsParamsR.ki_2 = [1;1;1]*0.02;
        ControlData.bsStatesR.SumZ_1=[0;0;0];
        ControlData.bsStatesR.SumZ_2=[0;0;0];
        ControlData.bsParamsR.F1=@bsf1_R;
        ControlData.bsParamsR.F2=@bsf2_R;
        ControlData.bsParamsR.G1=@bsg1_R;
        ControlData.bsParamsR.G2=@bsg2_R;
        ControlData.bsParamsR.dt =Data.dt;  
end

