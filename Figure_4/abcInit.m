%% 
% m : 输入数
% n : 阶数
function [ ControlData, Data] = abcInit( Data )
       
       Data.ControlName='abc';
       ControlData.IsDyGain = 1; 
       ControlData.IsCompensate = 1; % 是否补偿
       ControlData.Bound = [ 6;6;40;0.96;0.96;0.96]*1; %[ 3;3;32;0.65;0.65;0.65]
       ControlData.IsBound = 1;
       ControlData.Solve = 1; 
       ControlData.IsFilter =1;  % 滤波器开关
       
      %% ---------adrc Params Begin  
           n=2;
           m=3;
           Params.n=n;
           Params.m=m;
           h=0.02;
           Params.h = h;
          %% --------- TD
           Params.r0 = ones(1,m ).* 0.001 ./ h^2; %ones(1,m ).* 0.001 ./ h^2;
           Params.h0 = ones(1,m ).*h*3;
           Params.tdtransType='n';
           
          %% --------- ESO
           Params.Alpha = ones(n+1,m).* [1;1/2;1/4];
           w0 = 5.0;%(2018.04.29 :1.2)
           Params.Beta  = ones(n+1,m).* [3*w0; 3*w0^2; w0^3]; %[100; 300; 1000];% .*[1/h; 1/(3*h^2); 1/(64* h^3) ]; % ones(n+1,m)%另有 100 300 1000
           Params.Delta = ones(1,m).*h;
           Params.b0     = [1,1,1];
           Params.esoType ='n' ; % 非线性
           
          %% --------- NFL
           Params.cK = [ 3.9,   3.9,   4.9;...
                         3.6,   3.6,   3.6]; %  
%            Params.cK = ones(n,m).*[19.1; 6.1];
           Params.cAlpha = ones(n,m).*[0.369;12.9]; % 0<α1<1<αi, i=2,3,4....
           Params.cDelta = ones(1,m).*[0.2,0.2,0.62]; % 04.19:[0.2,0.2,0.2];
           
           Params.cr  = [1,1,1];
           Params.cC  = [ 1.8,1.8,1.2]; % 相当于 d
           Params.ch1 = [ 0.12, 0.12, 0.1] ; % 1/h1 相当于p (2018.04.19:[ 0.6, 0.6, 0.3])
           Params.cb0 =  Params.b0;
           Params.nflType = 1;  % (2018.04.19: 2 )
           Params.UoutType = 0;
           
          %% --------- Filter
           Params.k0=0.02;   % TD Filter
           Params.ifAlpha=ones(1,m)*36;    % Inertial Filter
           Params.ifType=1;
%% ---------Params End
           ControlData.Params = Params;
           States = adrcGetStates(ControlData.Params.m);
           ControlData.States = States;
           
%% Backstepping Attitude
           ControlData.bsParamsR.Alpha1 = [ 9; 9; 9];
           ControlData.bsParamsR.Alpha2 = [ 9; 9; 9 ];
           ControlData.bsParamsR.ki_1   = [ 1.0;1.0;1.0]*0.5;
           ControlData.bsParamsR.ki_2   = [ 1.0;1.0;1.0]*0.5;
           ControlData.bsStatesR.SumZ_1=[0;0;0];
           ControlData.bsStatesR.SumZ_2=[0;0;0];
           
           ControlData.bsParamsR.F1=@bsf1_R;
           ControlData.bsParamsR.F2=@bsf2_R;
           ControlData.bsParamsR.G1=@bsg1_R;
           ControlData.bsParamsR.G2=@bsg2_R;
           ControlData.bsParamsR.dt =Data.dt;    
end