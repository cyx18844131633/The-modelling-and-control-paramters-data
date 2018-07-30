%% 
% m : ������
% n : ����
function [ ControlData, Data] = dcpidInit( Data ,amDyModel)
           Data.ControlName='dc-pid';
           pidStates = pidInitStates(6);
           ControlData.pidStates=pidStates;
          dt=Data.dt;
           % ------���Ʋ���
           pidParams1.Type = 0;  
           pidParams1.calType = 'v'; % 12 6 1.2
           ControlData.IsDyGain = 1 ;  % Ut/Ia Uf/ma;
           pidParams1.pid=[ [ 9.6;            1e-3;      6.0 ],... % x
                            [ 9.6;            1e-3;      6.0 ],... % y
                            [ 9.6;            1e-3;      6.0 ],... % z
                            [ 19.2;            1e-3;      9.0 ],...  % ��x
                            [ 19.2;            1e-3;      9.0 ],... % ��y
                            [ 19.2;            1e-3;      9.0] ,... % ��z
                          ];
           r=amDyModel.Param.ControlTimeRatio;
           Gain=1;
           pidParams1.pid=pidParams1.pid.*[1;r*dt;1/(r*dt)]*Gain;
           ControlData.pidParams=pidParams1;
           ControlData.IsCompensate = 1; % �Ƿ񲹳�
           ControlData.Bound = [ 6;6;40;0.965;0.965; 9.65]*1;
           ControlData.IsBound = 1;
           ControlData.Solve = 1 ; 
           ControlData.IsFilter = 1 ;  % �˲�������yi
end