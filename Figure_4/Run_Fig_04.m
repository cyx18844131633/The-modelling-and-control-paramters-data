ConditionNum = 2; % 测试方法数
FileNames = { 'abc.mat','dc-pid.mat','adrc.mat','bs.mat'};
Type=1;
if(Type ==1)
    % 绘制误差曲线
    ResultPlotErrors_Fig_04(FileNames( 1:ConditionNum) ,1,{'abc','dc-pid'});
else
    % 绘制响应曲线
    ResultPlot_Fig_04(FileNames( 1:ConditionNum) ,1,{'abc','dc-pid'});
end