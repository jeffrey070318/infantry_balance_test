function geometry = placeholder_serial_leg_geometry()
%PLACEHOLDER_SERIAL_LEG_GEOMETRY Test geometry, not a CAD transcription.

geometry.origin = [0; 0];
geometry.activeLink1 = 0.133;
geometry.passiveLink1 = 0.250;
geometry.passiveLink2 = 0.250;
geometry.activeLink2 = 0.133;
geometry.branch = -1;
geometry.taskAngleZero = pi / 2;
geometry.source = "仅用于闭环算法测试的对称占位几何";
geometry.warning = "不得当作交龙最终尺寸或本队SolidWorks尺寸";
end
