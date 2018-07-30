%% 全向旋翼机械臂abc控制全驱任务控制比较

ConditionNum =2; % 测试方法数
PlotNum = 2;    %绘制曲线数
FileNames = { 'abc.mat','dc-pid.mat','adrc.mat','bs.mat'};
FolderName= './';
NameLen=length(FileNames);
DrawErrors = 1;

for i=1:NameLen
    FileNames{i} = [FolderName, FileNames{i}];
end

%% 任务编号
Data.TaskID = 2;%4 ;

%% 仿真参数
amDyModel = getDyDataScript();

Time = 12 ;
dt=amDyModel.Param.dt;
Data.Time=Time;
t=0:dt:Time;
Data.dt=dt;
Data.Num=length(t);

Data.InitEuler =   [0 ; 0; 0]*1;
InitTheta = [ [-pi/2;-pi/2;0;0;0]*1,...% -90 -90 0 0 0
              [-pi/2; 0;0;0;0]*1,...   % -90 0 0 0 0 
              [ 0;0;-pi/2;0;0]*1,...   % 0 0 -90 0 0
              [-pi/2;0;-pi/2;0;0]*1    % -90 0 -90 0 0
              ];
Data.InitTheta = InitTheta(:,1);% [ -90 0 -45 0]
Data.InitPa =   [ 9; 9; 9 ];

SignalsGen = @SignalsCreator;
DataOper = @DataOperator;
Display =  @Show;
          
%% 扰动噪声信号参数
Signals = PerNoise(); 

figure(1);
for i = 1 :  ConditionNum  %1  %
   Data.ArmConfig_ID = i;
   %% ---------运行参数
   switch ( i ) 
       case 2 %dc-PID
           %{
           Data.ControlName='dc-pid';
           Controller=@dcpidController;
           pidStates = pidInitStates(6);
           ControlData.pidStates=pidStates;
          
           % ------控制参数
           pidParams1.Type = 0;  
           pidParams1.calType = 'v'; % 12 6 1.2
           ControlData.IsDyGain = 1 ;  % Ut/Ia Uf/ma;
           pidParams1.pid=[ [ 3.6;            1e-3;      6.0 ],... % x
                            [ 3.6;            1e-3;      6.0 ],... % y
                            [ 6.6;            1e-3;      6.0 ],... % z
                            [ 9.2;            1e-3;      9.0 ],...  % ωx
                            [ 9.2;            1e-3;      9.0 ],... % ωy
                            [ 9.2;            1e-3;      9.0] ,... % ωz
                          ];
           r=amDyModel.Param.ControlTimeRatio;
           Gain=1;
           pidParams1.pid=pidParams1.pid.*[1;r*dt;1/(r*dt)]*Gain;
           ControlData.pidParams=pidParams1;
           ControlData.IsCompensate = 1; % 是否补偿
           ControlData.Bound = [ 6;6;40;0.965;0.965;09.65]*1;
           ControlData.IsBound = 1;
           ControlData.Solve = 1 ; 
           ControlData.IsFilter = 1 ;  % 滤波器开关yi
         %}  
         Controller=@dcpidController;
         [ ControlData, Data] = dcpidInit( Data,amDyModel );
    case 4 % Backstepping
        Controller = @bsController;
        [ ControlData, Data] = backsteppingInit( Data );
     case 1 % abc
     Controller=@abcController;
     [ControlData,Data] = abcInit(Data);
           
     case 3 % adrc
       Data.ControlName='adrc';
       Controller = @adrcController;
       ControlData.IsDyGain = 1; 
       ControlData.IsCompensate = 1; % 是否补偿
       ControlData.Bound = [ 3;3;36;0.65;0.65;0.65]*1;
       ControlData.IsBound = 1;
       ControlData.Solve = 0; 
       ControlData.IsFilter = 1;  % 滤波器开关
       
      %% ---------adrc Params Begin  
           n=2;
           m=6;
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
           w0=3.0;% 3.26
           Params.Beta  = ones(n+1,m).*[3*w0; 3*w0^2; w0^3]; % [1/h; 1/(3*h^2); 1/(64* h^3) ]; % ones(n+1,m).*[100; 300; 1000];% %另有 100 300 1000
           Params.Delta = ones(1,m).*h;
           Params.b0     = [1,1,1,1,1,1]*1;
           Params.esoType ='n' ; % 非线性
           
          %% --------- NFL
           Params.cK = ones(n,m).*[19.1; 5.1];
           Params.cAlpha = ones(n,m).*[0.9;2.9]; % 0<α1<1<αi, i=2,3,4....
           Params.cDelta = ones(1,m).*[0.2,0.2,0.2,0.2,0.2,0.2];
           
           Params.cr  = [1,1,20,1,1,1];
           Params.cC  = [0.01,0.01,0.01,0.01,0.01,0.01]; % 相当于 d
           Params.ch1 = [2,2,2,2,2,2] ; % 1/h1 相当于p
           Params.cb0 =  Params.b0;
           Params.nflType = 2; 
           Params.UoutType = 0;
           
          %% --------- Filter
           Params.k0=0.02;   % TD Filter
           Params.ifAlpha=ones(1,m)*36;    % Inertial Filter
           Params.ifType=1;
%% ---------Params End
           ControlData.Params = Params;
           States = adrcGetStates(ControlData.Params.m);
           ControlData.States = States;     
   end
    % ------States
    ControlData.RefModel = getRefModel(); % getDyDataScript(); % 
         
    %% ---------初始化状态
    amDyModel =  getDyDataScript();
    amDyModel.States.Euler = Data.InitEuler;
    amDyModel.States.Theta = Data.InitTheta;
    amDyModel.States.Pa = Data.InitPa;
    Data.Signals=Signals;

    switch (i)
        case 2
            disp('dc-PID...');
        case 4
            disp('Backstepping...');
        case 1
            disp('abc...');
        case 3
            disp('adrc...');
    end
   
    %% 开始运行程序
    [amDyModel, Data]= amDyRun( amDyModel,Time, Controller, ControlData,SignalsGen, DataOper,Display,Data);

    %% 保存数据
    save( FileNames{i},'Data' );
    pause(1.28);
    
    figure(i*100);
    subplot(2,2,1);
    plot(Data.Noise(1:3,:).');hold on;
    
    subplot(2,2,2);
    plot(Data.Noise(4:6,:).'/pi*180);hold on;
    
    subplot(2,2,3);
    plot(Data.Perb(1:3,:).');hold on;
    
    subplot(2,2,4);
    plot(Data.Perb(4:6,:).'/pi*180);hold on;

end
%% 绘图
% TaskPlot( Data.FileNames );
if(DrawErrors ==1)
%     ResultPlotErrors(FileNames( 1:ConditionNum) ,1);
      ResultPlotErrors_Fig_05(FileNames( 1:PlotNum) ,1);
else
%     ResultPlot(FileNames( 1:ConditionNum) ,1);
      ResultPlot_Fig_05(FileNames( 1:PlotNum) ,1);
end

d3Plot(FileNames( 1:PlotNum));

%% 信号
function [ Data ] = SignalsCreator( i, Data )
  
   %% ------扰动
   Data.Signals.Perb = 0;
   if( rem(i,100)==0 )
       Data.Signals.Perb = normrnd(Data.Signals.PerbMu,Data.Signals.PerbSigma).* Data.Signals.PerbVal;
       %disp(Data.Signals.Perb);
   end
   
   %% ------噪声
   Data.Signals.Noise = normrnd(Data.Signals.NoiseMu,Data.Signals.NoiseSigma).* Data.Signals.NoiseVal;
   
   %% ------期望
   switch(Data.TaskID)
       case 1 % 机械臂运动悬停
           Data.Signals.Yd = [  Data.InitPa(1)+0, ...   % X
                                Data.InitPa(2)+0, ...   % Y
                                Data.InitPa(3)+0, ...   % Z
                                pi/180*0, ...   % EulerX
                                pi/180*0, ...   % EulerY
                                pi/180*0 ];     % EulerZ
                   w = 1;
                   val =0.5;
                   dTheta = [0;0;sin(Data.dt*w*i)*val;0;0];
                   Data.ddTheta  =     ( dTheta - dTheta)/Data.dt;
                   Data.dTheta = dTheta;
       case 2 % 机械臂平台同时运动有倾角悬停
            Data.Signals.Yd = [  Data.InitPa(1)+ 1*1, ...   % X
                                Data.InitPa(2)+  1*1, ...   % Y
                                Data.InitPa(3)+  1*1, ...   % Z
                                pi/180*0, ...   % EulerX
                                pi/180*0, ...   % EulerY
                                pi/180*0 ];     % EulerZ
                   w = 1;
                   val =0.5;
                   dTheta = [0;0;sin(Data.dt*w*i)*val;0;0];
                   Data.ddTheta  =     ( dTheta - dTheta)/Data.dt;
                   Data.dTheta = dTheta;
       case 3 % 螺旋
            Vel=0.2;
            dt =Data.dt;
            x =   cos( dt*i  )*Vel;
            y =   sin( dt*i)*Vel;
            z =   dt*i*Vel*0.1*1;
            Data.Signals.Yd = [  -Vel+Data.InitPa(1)+x, ...   % X
                                 Data.InitPa(2)+y, ...   % Y
                                 Data.InitPa(3)+z, ...   % Z
                                     pi/180*0, ...   % EulerX
                                     pi/180*0, ...   % EulerY
                                     pi/180*0 ];     % EulerZ
             w = 1;
             val =0; % 0.5;
             dTheta = [0;0;sin(Data.dt*w*i)*val;0;0];
             Data.ddTheta  =     ( dTheta - dTheta)/Data.dt;
             Data.dTheta = dTheta;
        case 4 % 直线运动
           Data.Signals.Yd = [  Data.InitPa(1)+1, ...   % X
                                Data.InitPa(2)+1, ...   % Y
                                Data.InitPa(3)+1, ...   % Z
                                pi/180*0, ...   % EulerX
                                pi/180*0, ...   % EulerY
                                pi/180*0 ];     % EulerZ
           Data.dTheta=zeros(5,1);
           Data.ddTheta=zeros(5,1);
        case 5 % 倾斜运动
           Data.Signals.Yd = [  Data.InitPa(1)+0, ...   % X
                                Data.InitPa(2)+0, ...   % Y
                                Data.InitPa(3)+0, ...   % Z
                                pi/180*20, ...   % EulerX
                                -pi/180*20, ...   % EulerY
                                pi/180*0 ];     % EulerZ
           Data.dTheta=zeros(5,1);
           Data.ddTheta=zeros(5,1);
   end
end

%% pid控制函数
function [ amDyModel, ControlData, Data] = dcpidController( amDyModel,ControlData,i,Data)

   amDyModel.Input.dTheta = Data.dTheta;
   amDyModel.Input.ddTheta = Data.ddTheta;
   
   %% 加噪声
   PaN = amDyModel.States.Pa + Data.Signals.Noise(1:3);
   EulerN = amDyModel.States.Euler +  Data.Signals.Noise(4:6);
   RaN = amDyEuler2R( EulerN );

   %% 滤波
   if (ControlData.IsFilter==1)
   if(i==1)
       ControlData.Filter.y0 = [PaN;EulerN];  
   end
   y=[PaN;EulerN];
   ifAlpha =0.5;
   h=0.01;
   alpha=18;
   ControlData.Filter.y0 =  ControlData.Filter.y0 - h.*alpha.*(  ControlData.Filter.y0 - y);
 
   %  ControlData.Filter.y0 = ifAlpha.*y +  ( 1 - ifAlpha ) * ControlData.Filter.y0; 
   PaN = ControlData.Filter.y0(1:3);
   EulerN = ControlData.Filter.y0(4:6);
   RaN = amDyEuler2R( EulerN );
   end 
   
   %% pid控制
   % ------计算状态
   Rd = amDyEuler2R(Data.Signals.Yd(4:6) );  % 期望姿态
   Re = Rd * RaN.';    % 姿态误差
   [ w_e, theta_e ] = R2KRotParam( Re );
   
   % ------更新PID状态
   States= [ Data.Signals.Yd(1:3),[0,0,0]; PaN.', -w_e.'*theta_e ];
   if(i==1)
%        ControlData.pidStates.olde = ControlData.pidStates.e;
   end
   ControlData.pidStates=pidUpdateStates(States, ControlData.pidStates,ControlData.pidParams);
   
   % ------计算pid控制输入
   U = pidU(ControlData.pidStates, ControlData.pidParams);
   U =U.';
   
   % ------增益
   if(ControlData.IsDyGain ==1 )
       O=zeros(3);
       GainMat = [ ControlData.RefModel.Param.Ma*eye(3), O; O, ControlData.RefModel.SolveParam.Ia_0 ];
       U = GainMat*U;
   end
   
   % ------动态补偿器
   % ---同步模型
   [ ControlData.RefModel ] = amDySynModel( amDyModel, ControlData.RefModel);

   if ( ControlData.IsCompensate == 1 )
       Uc = amDyCompensator(   ControlData.RefModel );
       U  = U + Uc;
   end
   
   % ------限值
   if( ControlData.IsBound ==1)
       U = amDyBound(U,ControlData.Bound);
   end

   % ------驱动解算
   if ( ControlData.Solve )
   Fb = [RaN.'* U(1:3);RaN.'* U(4:6)]; % 转Aerial坐标系
   
   if(i==1)
       Data.fb = zeros(6,Data.Num);
   end
   Data.fb(:,i)=Fb;
   
   [ F, W ,fval]= DriverTrans( ControlData.RefModel.Driver.Mw2f,Fb,ControlData.RefModel.Driver.MaxW,...
                               ControlData.RefModel.Options,U,RaN,ControlData.pidStates.e,Uc);% 
   Udr = [ RaN ,zeros(3);zeros(3),RaN ] *F;

   U = Udr;
   if( sum ( W < 0 ) >0 )
       disp('Maybe Erorr......');
   end
%    disp(fval);
   end
 
   %% 加扰动
   U = U + Data.Signals.Perb;
   
   %% 更新驱动力
   if( 0 && i==50)
       Pf = rand(3,1)*15;
       Pt = rand(3,1)*15;
   else
       Pf = 0;
       Pt = 0;
   end
   amDyModel.Input.Uf=  U(1:3) + Pf; 
   amDyModel.Input.Ut=  U(4:6) + Pt;
end

%% 数据操作
function [ amDyModel,Data ]= DataOperator( amDyModel,Data ,i, ControlData)
   if(i==1)
       Data.X=zeros(6,Data.Num);
       Data.U=zeros(6,Data.Num);
       Data.Noise=zeros(6,Data.Num);
       Data.Perb=zeros ( 6,Data.Num);
       Data.Theta = zeros(5,Data.Num);
       Data.Yd = zeros(6,Data.Num);
%        Data.Z = zeros(6,Data.Num);
   end
   
   interval=200;
   if(rem(i-1,interval)==0)
       id = round((i-1)/interval)+1;
       Data.plotTheta(:,id ) = amDyModel.States.Theta;
       Data.plotEuler(:,id ) = amDyModel.States.Euler;
   end

   Data.X( :,i ) = [amDyModel.States.Pa; amDyModel.States.Euler];
   Data.U( :,i ) = [ amDyModel.Input.Uf; amDyModel.Input.Ut];
   Data.Noise(:,i)=Data.Signals.Noise;
   Data.Perb(:,i) = Data.Signals.Perb;
   Data.Theta(:,i) = amDyModel.States.Theta;
   Data.Yd(:,i) = Data.Signals.Yd.';
%    Data.Z(:,i)= ControlData.States.Z(1,:).';
end

%% 显示操作
function [ amDyModel,Data ] = Show( amDyModel,Data ,i)
   
   if(rem(i-1,amDyModel.Param.ControlTimeRatio*20)==0)
       clf;
       amDyDraw3d(amDyModel);
       camlight;
       grid on;
       axis equal;
       SetShowState(0.6,amDyModel.States.Pa);%
       
       drawnow;
   end
end
function [ F, W,fval ] = DriverTrans( Mw2f,Fb, MaxW,Options,U,R,E,Uc)

   Thres=0.01;
   fb=Fb;
   
   [ W ,fval ] = amDyFtoW( Mw2f, fb, MaxW,Options );
   if ( 0|| norm(fval)<Thres)
       F=Mw2f*W;
       return;
   end
   
   Un = U.*[0.8;0.8;1;1;1;1]; % [0.6;0.6;1;1;1;1]; 
   fb = [R.'* Un(1:3);R.'* Un(4:6)];
   M = Mw2f;
   
   [ W ,fval ] = amDyFtoW( M, fb, MaxW,Options);
   F = Mw2f*W;

end
%{
function [ amDyModel, ControlData, Data] = BacksteppingController( amDyModel,ControlData,i,Data)
   
   amDyModel.Input.dTheta = Data.dTheta;
   amDyModel.Input.ddTheta = Data.ddTheta;
   
   %% 加噪声
   PaN = amDyModel.States.Pa + Data.Signals.Noise(1:3);
   EulerN = amDyModel.States.Euler +  Data.Signals.Noise(4:6);
   RaN = amDyEuler2R( EulerN );

   %% 滤波
   if (ControlData.IsFilter==1)
   if(i==1)
       ControlData.Filter.y0 = [PaN;EulerN];  
   end
   y=[PaN;EulerN];
   ifAlpha =0.5;
   h=0.01;
   alpha=18;
   ControlData.Filter.y0 =  ControlData.Filter.y0 - h.*alpha.*(  ControlData.Filter.y0 - y);
   %  ControlData.Filter.y0 = ifAlpha.*y +  ( 1 - ifAlpha ) * ControlData.Filter.y0; 
   
   PaN = ControlData.Filter.y0(1:3);
   EulerN = ControlData.Filter.y0(4:6);
   RaN = amDyEuler2R( EulerN );
   end
   
   % ---同步模型
   [ ControlData.RefModel ] = amDySynModel( amDyModel, ControlData.RefModel);
   
   % ---------Backstepping
   % ------更新状态
   % ---P
    ControlData.bsStatesP.x1=PaN;
    ControlData.bsStatesP.x2=ControlData.RefModel.States.dPa;
    ControlData.bsStatesP.x1_d = Data.Signals.Yd(1:3).';
    ControlData.bsStatesP.dx1_d = zeros(3,1); %( Yt(i)-Yt(max(1,i-1)) )/dt;
    ControlData.bsStatesP.k=i;
    ControlData.bsStatesP.Ma = ControlData.RefModel.Param.Ma;
    [ Uf, ControlData.bsStatesP] = bsU( ControlData.bsStatesP, ControlData.bsParamsP );
   
    % ---R
    ControlData.bsStatesR.x1=EulerN;
    ControlData.bsStatesR.x2=ControlData.RefModel.States.OmegaA;
    ControlData.bsStatesR.x1_d = Data.Signals.Yd(4:6).';
    ControlData.bsStatesR.dx1_d = zeros(3,1); %( Yt(i)-Yt(max(1,i-1)) )/dt;
    ControlData.bsStatesR.k=i;
    ControlData.bsStatesR.Ia_0 = ControlData.RefModel.SolveParam.Ia_0;
    ControlData.bsStatesR.wIw_A = ControlData.RefModel.SolveParam.wIw_A;
    [ Ut, ControlData.bsStatesR] = bsU( ControlData.bsStatesR, ControlData.bsParamsR );
   
   U = [Uf ;Ut];
    
   % ------动态补偿器
   if ( ControlData.IsCompensate == 1 )
       Uc = amDyCompensator(   ControlData.RefModel );
       U  = U + Uc;
   else
       Uc = zeros(6,1);
   end
   
   % ------限值
   if( ControlData.IsBound ==1)
       U = amDyBound(U,ControlData.Bound);
   end

   % ------驱动解算
   if ( ControlData.Solve )
   Fb = [RaN.'* U(1:3);RaN.'* U(4:6)]; % 转Aerial坐标系
   
   if(i==1)
       Data.fb = zeros(6,Data.Num);
   end
   Data.fb(:,i)=Fb;
   
   [ F, W ,fval]= DriverTrans( ControlData.RefModel.Driver.Mw2f,Fb,ControlData.RefModel.Driver.MaxW,...
                               ControlData.RefModel.Options,U,RaN,1,Uc);% 
   Udr = [ RaN ,zeros(3);zeros(3),RaN ] *F;

   U = Udr;
   if( sum ( W < 0 ) >0 )
       disp('Maybe Erorr......');
   end
   disp(fval);
   end
 
   %% 加扰动
   U = U + Data.Signals.PerbVal;
   
   %% 更新驱动力
   amDyModel.Input.Uf=  U(1:3); 
   amDyModel.Input.Ut=  U(4:6);

end
%}

%{
function [ f1 ] = bsf1_P( bsStates,bsParams )
   f1 = 0;
end

function [ f2 ] = bsf2_P( bsStates,bsParams )
   f2 = 0;
end

function [ g1 ] = bsg1_P( bsStates,bsParams )
   g1 = eye(3);
end

function [ g2 ] = bsg2_P( bsStates,bsParams )
   g2 = 1/bsStates.Ma * eye(3);
end


function [ f1 ] = bsf1_R( bsStates,bsParams )
   f1 = 0;
end


function [ f2 ] = bsf2_R( bsStates,bsParams )
   f2 = -bsStates.Ia_0\bsStates.wIw_A;
end

function [ g1 ] = bsg1_R( bsStates,bsParams )
   g1 = amDyEulerJaco( bsStates.x1,'w2e');
end

function [ g2 ] = bsg2_R( bsStates,bsParams )
   g2 = bsStates.Ia_0\eye(3);
end
%}