function result = build_sanitized_right_leg_simscape(runShortSimulation)
%BUILD_SANITIZED_RIGHT_LEG_SIMSCAPE Import and sanitize the right-leg CAD model.

if nargin < 1
    runShortSimulation = true;
end

thisDir = fileparts(mfilename('fullpath'));
importDir = fullfile(thisDir, 'right_leg_import');
xmlPath = fullfile(importDir, 'serial_leg_right.xml');
rawModel = 'serial_leg_right_cad_import';
cleanModel = 'serial_leg_right_cad_sanitized';
dataFileBase = 'serial_leg_right_DataFile';

if ~isfile(xmlPath)
    error('缺少右腿导出 XML：%s', xmlPath);
end

oldDir = pwd;
cleanupDirectory = onCleanup(@() cd(oldDir));
cd(importDir);

closeIfLoaded(rawModel);
closeIfLoaded(cleanModel);

generatedFiles = {
    fullfile(importDir, [rawModel '.slx'])
    fullfile(importDir, [cleanModel '.slx'])
    fullfile(importDir, [dataFileBase '.m'])
    };
for index = 1:numel(generatedFiles)
    if isfile(generatedFiles{index})
        delete(generatedFiles{index});
    end
end

[rawHandle, dataFileName] = smimport(xmlPath, ...
    'ModelName', rawModel, ...
    'DataFileName', dataFileBase, ...
    'ModelSimplification', 'groupRigidBodies');
save_system(rawHandle, [rawModel '.slx']);
close_system(rawHandle, 0);

load_system(rawModel);
save_system(rawModel, [cleanModel '.slx']);
close_system(rawModel, 0);
load_system(cleanModel);

redundantJoints = {
    [cleanModel '/Cylindrical21']
    [cleanModel '/Cylindrical6']
    [cleanModel '/Revolute10']
    [cleanModel '/A_1/Cylindrical']
    [cleanModel '/A_1/Cylindrical6']
    [cleanModel '/A_1/Planar2']
    };

removedJoints = strings(0, 1);
for index = 1:numel(redundantJoints)
    blockPath = redundantJoints{index};
    if getSimulinkBlockHandle(blockPath) ~= -1
        delete_block(blockPath);
        removedJoints(end + 1, 1) = string(blockPath); %#ok<AGROW>
    end
end

set_param(cleanModel, 'SimulationCommand', 'update');
save_system(cleanModel);

simulationEndTime = NaN;
if runShortSimulation
    simulationOutput = sim(cleanModel, 'StopTime', '0.05');
    simulationEndTime = simulationOutput.tout(end);
end

close_system(cleanModel, 0);

result = struct();
result.xmlPath = xmlPath;
result.rawModelPath = fullfile(importDir, [rawModel '.slx']);
result.cleanModelPath = fullfile(importDir, [cleanModel '.slx']);
result.dataFileName = dataFileName;
result.removedJoints = removedJoints;
result.modelUpdatePassed = true;
result.shortSimulationPassed = runShortSimulation;
result.simulationEndTime = simulationEndTime;
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
