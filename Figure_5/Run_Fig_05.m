ConditionNum = 2; % ���Է�����
FileNames = { 'abc_1.mat','dc-pid_1.mat','adrc.mat','bs.mat'};
Type=1;
if(Type ==1)
    % �����������
    ResultPlotErrors_Fig_05(FileNames( 1:ConditionNum) ,1,{'abc','dc-pid'});
else
    % ������Ӧ����
    ResultPlot_Fig_05(FileNames( 1:ConditionNum) ,1,{'abc','dc-pid'});
end