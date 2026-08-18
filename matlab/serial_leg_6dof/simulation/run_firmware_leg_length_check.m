function result = run_firmware_leg_length_check(makeFigure)
%RUN_FIRMWARE_LEG_LENGTH_CHECK Compare firmware L0 limits with CAD kinematics.

arguments
    makeFigure (1, 1) logical = true
end

firmware = team_firmware_parameters();
cad = cad_serial_leg_parameters();
qReference = cad.referenceOutputAngles;
poseReference = cad_serial_leg_forward_kinematics(qReference, cad);

offsetDegrees = -30:1:30;
reachable = false(numel(offsetDegrees));
legLengthM = nan(size(reachable));
conditionNumber = nan(size(reachable));
for row = 1:numel(offsetDegrees)
    for column = 1:numel(offsetDegrees)
        q = qReference + deg2rad([offsetDegrees(row); offsetDegrees(column)]);
        try
            pose = cad_serial_leg_forward_kinematics(q, cad);
            [~, diagnostics] = cad_serial_leg_task_jacobian(q, cad);
            reachable(row, column) = true;
            legLengthM(row, column) = pose.legLength;
            conditionNumber(row, column) = diagnostics.conditionNumber;
        catch
            % Unreachable closed-chain branches remain outside the mask.
        end
    end
end

withinFirmwareRange = reachable & ...
    legLengthM >= firmware.legLengthRange(1) & ...
    legLengthM <= firmware.legLengthRange(2);
wellConditioned = reachable & conditionNumber <= 100;

fprintf('固件确认腿长范围: %.3f ~ %.3f m\n', firmware.legLengthRange);
fprintf('固件默认腿长目标: %.3f m\n', firmware.legLengthDefault);
fprintf('CAD参考姿态L0: %.9f m\n', poseReference.legLength);
fprintf('±30°主动输出扫描在固件腿长范围内: %.1f %%\n', ...
    100 * nnz(withinFirmwareRange) / nnz(reachable));
fprintf('固件范围内且cond(J)<=100: %.1f %%\n', ...
    100 * nnz(withinFirmwareRange & wellConditioned) / nnz(withinFirmwareRange));

if makeFigure
    figureHandle = figure('Name', 'Firmware leg length range', ...
        'Color', 'white');
    imagesc(offsetDegrees, offsetDegrees, legLengthM.');
    axis xy equal tight;
    colorbar;
    hold on;
    contour(offsetDegrees, offsetDegrees, withinFirmwareRange.', [0.5 0.5], ...
        'k', 'LineWidth', 1.2);
    xlabel('q_1相对参考角 (deg)');
    ylabel('q_2相对参考角 (deg)');
    title('固件确认L0范围在CAD主动输出空间中的区域');
    saveas(figureHandle, fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        'results', '18_firmware_leg_length_range.png'));
end

result.firmware = firmware;
result.cadParameters = cad;
result.referencePose = poseReference;
result.offsetDegrees = offsetDegrees;
result.reachable = reachable;
result.legLengthM = legLengthM;
result.conditionNumber = conditionNumber;
result.withinFirmwareRange = withinFirmwareRange;
result.wellConditioned = wellConditioned;
result.warning = "固件腿长范围是运行边界证据，不等于实体止挡和碰撞边界";
end
