function result = import_serial_leg_cad_simscape(xmlPath)
%IMPORT_SERIAL_LEG_CAD_SIMSCAPE Import a SolidWorks-exported Multibody XML.
%   result = import_serial_leg_cad_simscape()
%   result = import_serial_leg_cad_simscape(xmlPath)
%
% The SolidWorks assembly itself (.SLDASM) cannot be passed directly to
% smimport. It must first be exported with Simscape Multibody Link as XML.

repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
importDir = fullfile(repoRoot, 'matlab', 'serial_leg_6dof', 'simscape', 'import');
if ~exist(importDir, 'dir')
    mkdir(importDir);
end

if nargin < 1 || strlength(string(xmlPath)) == 0
    xmlFiles = dir(fullfile(importDir, '*.xml'));
    if isempty(xmlFiles)
        error(['没有找到 Simscape Multibody XML。请先在 SolidWorks 中打开 ', ...
            'D:\Project\SWproject\轮腿\串腿底盘2.8\串腿底盘.SLDASM，', ...
            '使用 Simscape Multibody Link 导出 XML，并放入：', newline, ...
            importDir]);
    end
    xmlPath = fullfile(xmlFiles(1).folder, xmlFiles(1).name);
else
    xmlPath = char(xmlPath);
end

if ~isfile(xmlPath)
    error('XML 文件不存在：%s', xmlPath);
end
[~, ~, ext] = fileparts(xmlPath);
if ~strcmpi(ext, '.xml')
    error('输入文件必须是 .xml：%s', xmlPath);
end

oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir));
cd(fileparts(xmlPath));

fprintf('开始导入 Simscape Multibody XML：%s\n', xmlPath);
[modelHandle, dataFileName] = smimport(xmlPath);
if isobject(modelHandle) && isprop(modelHandle, 'Name')
    modelName = char(modelHandle.Name);
elseif ischar(modelHandle) || isstring(modelHandle)
    modelName = char(modelHandle);
else
    modelName = 'generated Simscape Multibody model';
end

result = struct();
result.xmlPath = xmlPath;
result.modelHandle = modelHandle;
result.dataFileName = dataFileName;
result.modelName = modelName;
result.importDirectory = fileparts(xmlPath);
result.nextStep = ["检查刚体和关节拓扑；", ...
    "再配置关节传感器、轮地接触和执行器。"];
fprintf('导入完成，生成模型：%s\n', modelName);
end
