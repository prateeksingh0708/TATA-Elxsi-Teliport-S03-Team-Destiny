classdef ObstacleAvoidance < handle
    properties
        obstacles
        safeRadius
        repulseStrength
    end
    methods
        function obj = ObstacleAvoidance()
            obj.obstacles = [];
            obj.safeRadius = 2.0;
            obj.repulseStrength = 3.0;
        end
        function addObstacle(obj, pos, radius)
            obj.obstacles = [obj.obstacles; pos(:)', radius];
        end
        function [avoidVel, detected, closestDist] = computeAvoidance(obj, dronePos)
            avoidVel = [0; 0; 0]; detected = false; closestDist = inf;
            if isempty(obj.obstacles), return; end
            for i = 1:size(obj.obstacles, 1)
                obsPos = obj.obstacles(i, 1:3)';
                obsRad = obj.obstacles(i, 4);
                diff = dronePos(:) - obsPos;
                dist = norm(diff);
                closestDist = min(closestDist, dist - obsRad);
                threshold = obsRad + obj.safeRadius;
                if dist < threshold && dist > 0.01
                    detected = true;
                    strength = obj.repulseStrength * (threshold - dist) / threshold;
                    avoidVel = avoidVel + strength * (diff / dist);
                end
            end
        end
    end
end
