classdef TrajectoryPlanner < handle
    % TrajectoryPlanner - Multi-phase trajectory generation for landing
    % Implements state machine: approach → sync → descend → landed
    
    properties
        phase              % Current phase string
        platformModel      % Reference to MovingPlatform object
        
        % Phase thresholds
        approachHeight     % Height for approach phase (m, negative NED)
        syncHeight         % Height for synchronization (m, negative NED)
        descentRate        % Descent velocity (m/s, positive = down)
        
        % Transition thresholds
        approachRadius     % Horizontal distance threshold (m)
        syncVelThreshold   % Velocity matching threshold (m/s)
        syncPosThreshold   % Position alignment threshold (m)
        landingThreshold   % Final landing height (m)
        
        % Logging
        phaseHistory       % Phase transition log
        phaseTimeStamps    % Time of each phase transition
    end
    
    methods
        function obj = TrajectoryPlanner(platform)
            % Constructor
            %
            % Input:
            %   platform: MovingPlatform object
            %
            % Usage:
            %   planner = TrajectoryPlanner(platformObject);
            
            obj.platformModel = platform;
            obj.phase = 'approach';
            
            % Phase heights (NED: negative = up)
            obj.approachHeight = -5;    % 5m above platform
            obj.syncHeight = -3;        % 3m above platform
            obj.descentRate = 0.3;      % 30cm/s descent
            
            % Transition criteria
            obj.approachRadius = 2.0;      % 2m horizontal radius
            obj.syncVelThreshold = 0.5;    % 0.5m/s velocity error
            obj.syncPosThreshold = 1.0;    % 1m position error
            obj.landingThreshold = 0.3;    % 30cm above platform
            
            % Initialize logging
            obj.phaseHistory = {'approach'};
            obj.phaseTimeStamps = [0];
        end
        
        function [targetPos, targetVel, phase] = getTarget(obj, dronePos, droneVel, t)
            % Compute target position and velocity based on current phase
            %
            % Inputs:
            %   dronePos: [x; y; z] current drone position (NED)
            %   droneVel: [vx; vy; vz] current drone velocity
            %   t: current simulation time (seconds)
            %
            % Outputs:
            %   targetPos: [x; y; z] target position
            %   targetVel: [vx; vy; vz] target velocity
            %   phase: current phase string
            
            % Get platform state
            platformPos = obj.platformModel.getPosition(t);
            % FIXED:
platformVel = obj.platformModel.getVelocity(t);
            % Calculate relative state
            relativePos = dronePos - platformPos;
            horizontalDist = norm(relativePos(1:2));
            heightAbovePlatform = -(dronePos(3) - platformPos(3));  % Positive = above
            
            % Velocity error
            velError = norm(droneVel - platformVel);
            
            % State machine logic
            switch obj.phase
                case 'approach'
                    % PHASE 1: Approach position above platform
                    targetPos = platformPos;
                    targetPos(3) = platformPos(3) + obj.approachHeight;
                    targetVel = platformVel;  % Match platform velocity
                    
                    % Transition condition
                    if horizontalDist < obj.approachRadius
                        obj.transitionPhase('sync', t);
                    end
                    
                case 'sync'
                    % PHASE 2: Synchronize velocity with platform
                    targetPos = platformPos;
                    targetPos(3) = platformPos(3) + obj.syncHeight;
                    targetVel = platformVel;
                    
                    % Transition condition (both velocity and position aligned)
                    if velError < obj.syncVelThreshold && ...
                       horizontalDist < obj.syncPosThreshold
                        obj.transitionPhase('descend', t);
                    end
                    
                case 'descend'
                    % PHASE 3: Controlled descent onto platform
                    targetPos = platformPos;
                    
                    % Gradual descent
                    if heightAbovePlatform > obj.landingThreshold
                        targetPos(3) = dronePos(3) + obj.descentRate * 0.1;
                    else
                        targetPos(3) = platformPos(3);  % Touch down
                    end
                    
                    targetVel = platformVel;  % Maintain sync
                    
                    % Transition condition
                    if heightAbovePlatform < obj.landingThreshold
                        obj.transitionPhase('landed', t);
                    end
                    
                case 'landed'
                    % PHASE 4: Stay on platform
                    targetPos = platformPos;
                    targetVel = platformVel;
            end
            
            phase = obj.phase;
        end
        
        function transitionPhase(obj, newPhase, t)
            % Internal method to handle phase transitions
            
            fprintf('[Planner] [t=%.1fs] Phase transition: %s → %s\n', ...
                    t, obj.phase, newPhase);
            
            obj.phase = newPhase;
            obj.phaseHistory{end+1} = newPhase;
            obj.phaseTimeStamps(end+1) = t;
        end
        
        function isLanded = checkLanded(obj)
            % Check if landing is complete
            %
            % Output:
            %   isLanded: true if in 'landed' phase
            
            isLanded = strcmp(obj.phase, 'landed');
        end
        
        function resetPlanner(obj)
            % Reset planner to initial state
            
            obj.phase = 'approach';
            obj.phaseHistory = {'approach'};
            obj.phaseTimeStamps = [0];
            
            fprintf('[Planner] Reset to approach phase\n');
        end
        
        function setHeights(obj, approachHeight, syncHeight)
            % Update phase heights
            %
            % Usage: planner.setHeights(-6, -4); % 6m and 4m above platform
            
            obj.approachHeight = approachHeight;
            obj.syncHeight = syncHeight;
            
            fprintf('[Planner] Heights updated: Approach=%.1fm, Sync=%.1fm\n', ...
                    -approachHeight, -syncHeight);
        end
        
        function setThresholds(obj, approachRad, syncVel, syncPos, landing)
            % Update transition thresholds
            
            obj.approachRadius = approachRad;
            obj.syncVelThreshold = syncVel;
            obj.syncPosThreshold = syncPos;
            obj.landingThreshold = landing;
            
            fprintf('[Planner] Thresholds updated\n');
        end
    end
end