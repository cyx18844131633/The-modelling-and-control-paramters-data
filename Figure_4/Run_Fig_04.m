ConditionNum = 2; % ���Է�����
FileNames = { 'abc.mat','dc-pid.mat','adrc.mat','bs.mat'};
Type=1;
if(Type ==1)
    % �����������
    ResultPlotErrors_Fig_04(FileNames( 1:ConditionNum) ,1,{'abc','dc-pid'});
else
    % ������Ӧ����
    ResultPlot_Fig_04(FileNames( 1:ConditionNum) ,1,{'abc','dc-pid'});
end