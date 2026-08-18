function outputPath = save_figure_6dof(figureHandle, fileName)
%SAVE_FIGURE_6DOF Save a tutorial figure to the project results folder.

arguments
    figureHandle (1, 1) matlab.ui.Figure
    fileName (1, 1) string
end

tutorialDir = fileparts(mfilename('fullpath'));
resultDir = fullfile(tutorialDir, 'results');
if ~isfolder(resultDir)
    mkdir(resultDir);
end

outputPath = fullfile(resultDir, fileName + ".png");
exportgraphics(figureHandle, outputPath, 'Resolution', 160);
fprintf('曲线图已保存: %s\n', outputPath);
end
