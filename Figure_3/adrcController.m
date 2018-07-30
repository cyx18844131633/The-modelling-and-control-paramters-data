function [ amDyModel, ControlData, Data] = adrcController( amDyModel,ControlData,i,Data)
   
   amDyModel.Input.dTheta = Data.dTheta;
   amDyModel.Input.ddTheta = Data.ddTheta;
   
   %% ������
   PaN = amDyModel.States.Pa + Data.Signals.Noise(1:3);
   EulerN = amDyModel.States.Euler +  Data.Signals.Noise(4:6);
   RaN = amDyEuler2R( EulerN );

   %% �˲�
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
   end 
  
   %% adrc����
   % ------����״̬
   Rd = amDyEuler2R(Data.Signals.Yd(4:6) );  % ������̬
   Re = Rd * RaN.';    % ��̬���
   [ w_e, theta_e ] = R2KRotParam( Re );
   
   % ------ADRC
     if( i==1)
         ControlData.States.V_1=[PaN.',EulerN.'];
         ControlData.States.Z(1,:)=[PaN.',EulerN.'];
         ControlData.U=zeros(6,1);
         ControlData.Uadrc=[0,0,0,0,0,0];
     end
   
    %% TDTrans
    [ControlData.States, y0]= adrcTDTrans( Data.Signals.Yd(1:6),ControlData.States,ControlData.Params );

    ControlData.States.y0 = [PaN.',EulerN.'];
    
    %% ESO
    f =0; % �Ƿ����ģ�Ͳ���ControlData.Uadrc
    ControlData.States = adrcESO( ControlData.Uadrc,ControlData.States,ControlData.Params,f);
    
    %% NFL
    U = adrcNFL( ControlData.States, ControlData.Params);
    ControlData.Uadrc=U;   
    U=U.';

   % ---ͬ��ģ��
   [ ControlData.RefModel ] = amDySynModel( amDyModel, ControlData.RefModel);
   
    % ------����
    if( ControlData.IsDyGain ==1 )
       O=zeros(3);
       GainMat = [ ControlData.RefModel.Param.Ma*eye(3), O; O, ControlData.RefModel.SolveParam.Ia_0 ];
       U = GainMat*U;
    end
%     ControlData.Uadrc=U.';   
   % ------��̬������
   % ---ͬ��ģ��
   [ ControlData.RefModel ] = amDySynModel( amDyModel, ControlData.RefModel);

   if ( ControlData.IsCompensate == 1 )
       Uc = amDyCompensator(   ControlData.RefModel );
       U  = U + Uc;
   end
   
   % ------��ֵ
   if( ControlData.IsBound ==1)
       U = amDyBound(U,ControlData.Bound);
   end

   % ------��������
   if ( ControlData.Solve )
   Fb = [RaN.'* U(1:3);RaN.'* U(4:6)]; % תAerial����ϵ
   
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
 
   %% ���Ŷ�
   U = U + Data.Signals.Perb;
   ControlData.U =U;
   %% ����������
   amDyModel.Input.Uf=  U(1:3);
   amDyModel.Input.Ut=  U(4:6);
   
end