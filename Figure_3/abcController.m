function [ amDyModel, ControlData, Data] = abcController( amDyModel,ControlData,i,Data)
   
   amDyModel.Input.dTheta = Data.dTheta;
   amDyModel.Input.ddTheta = Data.ddTheta;
   
   %% 加噪声
   PaN = amDyModel.States.Pa + Data.Signals.Noise(1:3);
   EulerN = amDyModel.States.Euler +  Data.Signals.Noise(4:6);
   RaN = amDyEuler2R( EulerN );

   %% 滤波
   if (ControlData.IsFilter==1)
       if( i==1)
           ControlData.Filter.y0 = [PaN;EulerN];  
       end
       y=[PaN;EulerN];
       ifAlpha =0.5;
       h=0.02;
       alpha=18;
       ControlData.Filter.y0 =  ControlData.Filter.y0 - h.*alpha.*(  ControlData.Filter.y0 - y);  
       %  ControlData.Filter.y0 = ifAlpha.*y +  ( 1 - ifAlpha ) * ControlData.Filter.y0;  PaN = ControlData.Filter.y0(1:3);
       EulerN = ControlData.Filter.y0(4:6);
       RaN = amDyEuler2R( EulerN );
       PaN = ControlData.Filter.y0(1:3);
   end 
  
   %% adrc控制
   % ------计算状态
   Rd = amDyEuler2R(Data.Signals.Yd(4:6) );  % 期望姿态
   Re = Rd * RaN.';    % 姿态误差
   
   % ------ADRC
     if( i==1)
         ControlData.States.V_1=PaN.';
         ControlData.States.Z(1,:)=PaN.';
         ControlData.U=zeros(6,1);
         ControlData.ufadrc=[0,0,0];
     end
   
    %% TDTrans
    [ControlData.States, y0]= adrcTDTrans( Data.Signals.Yd(1:3),ControlData.States,ControlData.Params );

    ControlData.States.y0 = PaN.';
    
    %% ESO
    f =0; % 是否加入模型参数
    Uf = ControlData.U(1:3);
    ControlData.States = adrcESO( ControlData.ufadrc,ControlData.States,ControlData.Params,f);
    
    %% NFL
    Uf = adrcNFL( ControlData.States, ControlData.Params);
    
    ControlData.ufadrc=Uf;
    
   %% Backstepping Attitude
   % ---同步模型
   [ ControlData.RefModel ] = amDySynModel( amDyModel, ControlData.RefModel);
   
   % ---R
    ControlData.bsStatesR.x1=EulerN;
    ControlData.bsStatesR.x2=ControlData.RefModel.States.OmegaA;
    ControlData.bsStatesR.x1_d = Data.Signals.Yd(4:6).';
    ControlData.bsStatesR.dx1_d = zeros(3,1); %( Yt(i)-Yt(max(1,i-1)) )/dt;
    ControlData.bsStatesR.k=i;
    ControlData.bsStatesR.Ia_0 = ControlData.RefModel.SolveParam.Ia_0;
    ControlData.bsStatesR.wIw_A = ControlData.RefModel.SolveParam.wIw_A;
    
    [ Ut, ControlData.bsStatesR] = bsU( ControlData.bsStatesR, ControlData.bsParamsR );
   
    U=[Uf.';Ut];

    % ------增益
    O=zeros(3);
    if( ControlData.IsDyGain ==1 )
       
%        GainMat = [ ControlData.RefModel.Param.Ma*eye(3), O; O, ControlData.RefModel.SolveParam.Ia_0 ];
       GainMat = [ ControlData.RefModel.Param.Ma*eye(3), O; O, eye(3) ];
      
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
                               ControlData.RefModel.Options,U,RaN,1,1);% 
   Udr = [ RaN ,zeros(3);zeros(3),RaN ] *F;

   U = Udr;
   if( sum ( W < 0 ) >0 )
       disp('Maybe Erorr......');
   end
%    disp(fval);
   end
 
   %% 加扰动
   U = U + Data.Signals.Perb;
   ControlData.U =U;
   %% 更新驱动力
   amDyModel.Input.Uf=  U(1:3);
   amDyModel.Input.Ut=  U(4:6);
end



% function [ f1 ] = bsf1_P( bsStates,bsParams )
%    f1 = 0;
% end
% 
% function [ f2 ] = bsf2_P( bsStates,bsParams )
%    f2 = 0;
% end
% 
% function [ g1 ] = bsg1_P( bsStates,bsParams )
%    g1 = eye(3);
% end
% 
% function [ g2 ] = bsg2_P( bsStates,bsParams )
%    g2 = 1/bsStates.Ma * eye(3);
% end
% 
% function [ f1 ] = bsf1_R( bsStates,bsParams )
%    f1 = 0;
% end
% 
% function [ f2 ] = bsf2_R( bsStates,bsParams )
%    f2 = -bsStates.Ia_0\bsStates.wIw_A;
% end
% 
% function [ g1 ] = bsg1_R( bsStates,bsParams )
%    g1 = amDyEulerJaco( bsStates.x1,'w2e');
% end
% 
% function [ g2 ] = bsg2_R( bsStates,bsParams )
%    g2 = bsStates.Ia_0\eye(3);
% end

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
   
   %% 计算实际的ω^2 
   [ W ,fval ] = amDyFtoW( M, fb, MaxW,Options);
   
   %% 返回力/矩
   F = Mw2f*W;
end
