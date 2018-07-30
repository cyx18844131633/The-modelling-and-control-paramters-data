function [ amDyModel, ControlData, Data] = bsController( amDyModel,ControlData,i,Data)
   
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
   O=zeros(3);
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

function [ F, W,fval ] = DriverTrans( Mw2f,Fb, MaxW,Options,U,R,E,Uc)
   Thres=0.01;
   fb=Fb;
   
   [ W ,fval ] = amDyFtoW( Mw2f, fb, MaxW,Options );
   if ( 0|| norm(fval)<Thres)
       F=Mw2f*W;
       return;
   end
   Un = U.*[0.6;0.6;1;1;1;1]; % [0.6;0.6;1;1;1;1]; 
   fb = [R.'* Un(1:3);R.'* Un(4:6)];
   M = Mw2f;
   
   %% 计算实际的ω^2 
   [ W ,fval ] = amDyFtoW( M, fb, MaxW,Options);
   
   %% 返回力/矩
   F = Mw2f*W;
end