%%  3d¹ì¼£

function [ Res ] =d3Plot( FileNames)
   Marker={ '*','s','o','+','.','d'};
   LineStyle={'-','--',':','-.'};
   LineWidth = 1.0;
   subTitles={ 'X','¦Õ',...
               'Y','¦×',...
               'Z','¦Ã'};
   FontSize = 10;
   MarkerSize=0.8;
      %%
       MethodNum=length(FileNames);
       for i=1:MethodNum
           data = load(FileNames{i});
           mData(i)= data.Data;
           MethodNames{i}=data.Data.ControlName;
       end
      %% ×ÓÍ¼
       Width =600;
       Height =600;
       GapX =60;
       GapY =75;
       MarginL =75;
       MarginB =78;
       
       [ Size ] = getSubPlotSize( 1,1,Width,Height,GapX,GapY,MarginL,MarginB);
       Yd = mData(1).Yd;
       
       %% »æÍ¼
        figure(9);
        ah=axes('Units','pixels', 'Position',Size{1,1});
       %% »­ÆÚÍû
        X=mData(1).Yd;
        X = X*1000; % Position
        x = X( 1,:);
        y = X( 2,:);
        z = X( 3,:);
        p=plot3(x,y,z);hold on;
        p.LineStyle = LineStyle{end};
        p.LineWidth= LineWidth;
        p.Marker = Marker{end};
        p.MarkerSize = MarkerSize;
        for j=1:MethodNum
           X=mData(j).X;
           X = X*1000; % Position
           x = X( 1,:);
           y = X( 2,:);
           z = X( 3,:);
           p=plot3(x,y,z);hold on;
           p.LineStyle = LineStyle{j};
           p.LineWidth= LineWidth;
           p.Marker = Marker{j};
           p.MarkerSize = MarkerSize;
       end
       leg=legend([{'Exp'}, MethodNames ],'Orientation','horizontal');%
       set(leg,'Box','off');
       grid on;
%        axis equal;
end