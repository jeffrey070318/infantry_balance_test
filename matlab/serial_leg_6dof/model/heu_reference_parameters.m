function parameters = heu_reference_parameters(legLength)
%HEU_REFERENCE_PARAMETERS Parameters transcribed from HEU get_k_length.m.

arguments
    legLength (1, 1) double {mustBeFinite, mustBePositive} = 0.18
end

parameters.legLength = legLength;
parameters.wheelRadius = 0.10;
parameters.lowerComDistance = legLength / 2;
parameters.upperComDistance = legLength / 2;
parameters.bodyComOffset = 0.066;
parameters.wheelMass = 0.95;
parameters.legMass = 0.15;
parameters.bodyMass = 9.65;
parameters.wheelInertia = parameters.wheelMass * parameters.wheelRadius^2;
parameters.legInertia = parameters.legMass * (legLength^2 + 0.05^2) / 12;
parameters.bodyInertia = parameters.bodyMass * (0.32^2 + 0.12^2) / 12;
parameters.gravity = 9.8;
parameters.source = "广西大学收集的哈工程 get_k_length.m 参数";
parameters.warning = "参考开源参数，不是本队串联腿实机参数";
end
