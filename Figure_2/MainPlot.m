
%% 第1套数据
% FileNames = { 'abc_config_1.mat','abc_config_1.mat','abc_config_3.mat','abc_config_4.mat'};
%% 第2套数据 better
FileNames={'abc_config_05_1.mat','abc_config_05_2.mat','abc_config_05_3.mat','abc_config_05_4.mat'};
FolderName= './';

Legends={'型1','型2','型3','型4'};
NameLen=length(FileNames);
for i=1:NameLen
    FileNames{i} = [FolderName, FileNames{i}];
end

%% 画误差曲线
ResultPlotErrors( FileNames ,1,Legends);

%% 画响应曲线
% ResultPlot( FileNames ,1,Legends);