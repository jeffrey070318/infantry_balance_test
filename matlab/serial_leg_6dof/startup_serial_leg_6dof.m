function rootDir = startup_serial_leg_6dof()
%STARTUP_SERIAL_LEG_6DOF Add this six-state project to the MATLAB path.

rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(rootDir));
fprintf('六维串联腿工程已加入路径: %s\n', rootDir);
end
