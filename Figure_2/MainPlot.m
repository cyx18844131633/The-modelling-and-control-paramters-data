
%% ��1������
% FileNames = { 'abc_config_1.mat','abc_config_1.mat','abc_config_3.mat','abc_config_4.mat'};
%% ��2������ better
FileNames={'abc_config_05_1.mat','abc_config_05_2.mat','abc_config_05_3.mat','abc_config_05_4.mat'};
FolderName= './';

Legends={'��1','��2','��3','��4'};
NameLen=length(FileNames);
for i=1:NameLen
    FileNames{i} = [FolderName, FileNames{i}];
end

%% ���������
ResultPlotErrors( FileNames ,1,Legends);

%% ����Ӧ����
% ResultPlot( FileNames ,1,Legends);