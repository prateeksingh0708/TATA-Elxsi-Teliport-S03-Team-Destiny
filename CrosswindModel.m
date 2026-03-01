classdef CrosswindModel < handle
    % CrosswindModel - Realistic wind disturbance generator
    % Combines steady wind, gusts, and turbulence
    
    properties
        steadyWind         % [vx, vy, vz] constant wind component (m/s)
        gustAmplitude      % Peak gust strength (m/s)
        gustFrequency      % Gust oscillation frequency (Hz)
        turbulenceLevel    % Turbulence intensity (0-1 scale)
        
        randomSeed         % Random seed for reproducibility
        noiseGenerator     % Random number generator object
        
        % Logging
        windHistory        % History of wind velocities
        timeHistory        % Corresponding time points
    end
    
    methods
        function obj = CrosswindModel(steadyWind, gustAmplitude, turbulence)
            % Constructor
            %
            % Inputs:
            %   steadyWind: [vx; vy; vz] constant wind vector (m/s)
            %   gustAmplitude: maximum gust strength (m/s)
            %   turbulence: turbulence intensity 0-1
            %
            % Usage:
            %   wind = CrosswindModel([1.5; 1.0; 0], 2.0, 0.3);
            
            obj.steadyWind = steadyWind(:);
            obj.gustAmplitude = gustAmplitude;
            obj.gustFrequency = 0.2;  % Default: 5-second period gusts
            obj.turbulenceLevel = turbulence;
            
            % Initialize random number generator
            obj.randomSeed = 42;
            obj.noiseGenerator = RandStream('mt19937ar', 'Seed', obj.randomSeed);
            
            % Initialize logging
            obj.windHistory = [];
            obj.timeHistory = [];
        end
        
        function windVel = getWind(obj, t)
            % Calculate wind velocity at time t
            %
            % Input:
            %   t: current time (seconds)
            %
            % Output:
            %   windVel: [vx; vy; vz] wind velocity vector (m/s)
            
            % Component 1: Steady wind (constant)
            steady = obj.steadyWind;
            
            % Component 2: Sinusoidal gusts
            gustPhase = 2 * pi * obj.gustFrequency * t;
            gustMagnitude = obj.gustAmplitude * sin(gustPhase);
            
            % Gust direction (rotating in horizontal plane)
            gustAngle = gustPhase;
            gustDirection = [
                cos(gustAngle);
                sin(gustAngle);
                0.1 * sin(2 * gustPhase)  % Small vertical component
            ];
            
            % Normalize direction
            if norm(gustDirection) > 0
                gustDirection = gustDirection / norm(gustDirection);
            end
            
            gust = gustMagnitude * gustDirection;
            
            % Component 3: Random turbulence (high-frequency noise)
            turbulence = obj.turbulenceLevel * obj.gustAmplitude * [
                obj.noiseGenerator.randn();
                obj.noiseGenerator.randn();
                0.3 * obj.noiseGenerator.randn()  % Less vertical turbulence
            ];
            
            % Combined wind velocity
            windVel = steady + gust + turbulence;
            
            % Log data
            obj.windHistory = [obj.windHistory, windVel];
            obj.timeHistory = [obj.timeHistory, t];
        end
        
        function force = getWindForce(obj, t, dragCoeff, droneVel)
            % Calculate wind force on drone (simplified aerodynamics)
            %
            % Inputs:
            %   t: current time (seconds)
            %   dragCoeff: aerodynamic drag coefficient (default: 0.1)
            %   droneVel: current drone velocity (for relative wind)
            %
            % Output:
            %   force: [Fx; Fy; Fz] wind force vector
            
            if nargin < 3
                dragCoeff = 0.1;
            end
            if nargin < 4
                droneVel = [0; 0; 0];
            end
            
            % Relative wind velocity (wind - drone)
            windVel = obj.getWind(t);
            relativeWind = windVel - droneVel(:);
            
            % Simple drag force: F = k * v
            force = dragCoeff * relativeWind;
        end
        
        function setGustParameters(obj, amplitude, frequency)
            % Update gust characteristics
            %
            % Usage: wind.setGustParameters(3.0, 0.3);
            
            obj.gustAmplitude = amplitude;
            obj.gustFrequency = frequency;
            
            fprintf('[Wind] Gust parameters updated: Amplitude=%.1fm/s, Freq=%.2fHz\n', ...
                    amplitude, frequency);
        end
        
        function [windHist, timeHist] = getHistory(obj)
            % Retrieve logged wind history
            %
            % Outputs:
            %   windHist: 3xN matrix of wind velocities
            %   timeHist: 1xN vector of time points
            
            windHist = obj.windHistory;
            timeHist = obj.timeHistory;
        end
    end
end