%% 

function [ Res ] = ResultPlot( FileNames ,Mode,varargin)
   if nargin>2
       legends = varargin{1};
   end
       
   Marker={ '*','o','s','.','+','d'};
   LineStyle={'-','--','-.',':'};
   LineWidth = 1.0;
   subTitles={ 'X','φ',...
               'Y','ψ',...
               'Z','γ'};
   FontSize = 10;
   MarkerSize=0.8;
   if(Mode==1)
      %%
       MethodNum=length(FileNames);
       for i=1:MethodNum
           data = load(FileNames{i});
           mData(i)= data.Data;
           if nargin>2
               MethodNames{i}=legends{i};
           else
               MethodNames{i}=data.Data.ControlName;
           end
       end
      %% 子图
       Row =3;
       Col =2;
       Width =300;
       Height =120;
       GapX =60;
       GapY =75;
       MarginL =75;
       MarginB =78;
       
       [ Size ] = getSubPlotSize( Row,Col,Width,Height,GapX,GapY,MarginL,MarginB);
       Yd = mData(1).Yd;
       
       %% 绘图
        figure(6);
        for i=1:6 % 六个子图
            row=int8(i)/2;
            col=rem(i+1,2)+1;
            ah=axes('Units','pixels', 'Position',Size{row,col});
            maxTime=0;
            maxY = -10000;
            minY =100000;
            
           %% 画期望
            dt=mData(1).dt;
            Time=mData(1).Time;
            t = 0:dt:Time;
            yd = Yd( int8((col-1))*3 + row,:);
            if(col==1)
                yd = yd*1000; % Position
            else
                yd = yd/pi*180; % Attitude
            end     
            maxY = max(maxY,max(yd));
            minY = min(minY,min(yd));
            p=plot(t,yd);hold on;
            p.LineStyle = LineStyle{end};
            p.LineWidth= 1;
            p.Marker = Marker{end};
            p.MarkerSize = 0.8;
            
            for j=1:MethodNum
                 dt = mData(j).dt;
                 Time=mData(j).Time;
                 t = 0:dt:Time;
                 maxTime=max(maxTime,Time);
                 X = mData(j).X( int8((col-1))*3 + row,:);

                 if(col==1)
                     X = X*1000; % Position
                 else
                     X = X/pi*180; % Attitude
                 end
                 maxY = max(maxY,max(X));
                 minY = min(minY,min(X));
                 p=plot(t,X);hold on;
                 p.LineStyle = LineStyle{j};
                 p.LineWidth= LineWidth;
                 p.Marker = Marker{j};
                 p.MarkerSize = MarkerSize;
            end
           
            leg=legend([{'期望'} ,MethodNames ],'Orientation','horizontal');%
            set(leg,'Box','off');
            set(gca,'xtick',[0:1:maxTime]);
           
            if(col==1)
                % ---Position
                minY = minY -100;
                maxY = maxY +300;
                axis([0 maxTime minY maxY ]);
                ylabel('位置/mm');
                scaleNum = 6;
                d= round(max(100,(maxY-minY)/scaleNum));
                set(gca,'ytick',round([minY:d:maxY]));
            else
                % ---Euler
                minY = minY -0.5;
                maxY = maxY +3;
                axis([0 maxTime minY maxY ]);
                ylabel('Euler角/°');
                scaleNum = 6;
                d= round(max(1,(maxY-minY)/scaleNum));
                set(gca,'ytick',round([minY:d:maxY]));
            end
            set(gca,'Fontsize',FontSize);
            xlabel({'时间/s',subTitles{i}});
            grid on;
        end  
   else % Mode==2 
       
   end
   
end