classdef AirSimClient < handle
    % AirSimClient - Complete MATLAB interface for AirSim API
    % Handles all communication with AirSim simulator
    
    properties
        baseURL         % Base URL for AirSim API
        vehicleName     % Name of the drone vehicle
        timeout         % Request timeout in seconds
        isConnected     % Connection status flag
    end
    
    methods
        function obj = AirSimClient(vehicleName)
            % Constructor - Initialize AirSim client
            % 
            % Usage: client = AirSimClient('Drone1');
            
            if nargin < 1
                obj.vehicleName = 'Drone1';
            else
                obj.vehicleName = vehicleName;
            end
            
            obj.baseURL = 'http://localhost:41451';
            obj.timeout = 10;
            obj.isConnected = false;
            
            % Test connection
            obj.isConnected = obj.ping();
            
            if obj.isConnected
                fprintf('[AirSim] Connected to vehicle: %s\n', obj.vehicleName);
            else
                warning('[AirSim] Failed to connect. Ensure AirSim is running.');
            end
        end
        
        function result = ping(obj)
            % Test connection to AirSim
            % Returns: true if connected, false otherwise
            
            try
                url = sprintf('%s/ping', obj.baseURL);
                options = weboptions('Timeout', 5);
                response = webread(url, options);
                result = true;
            catch
                result = false;
            end
        end
        
        function confirmConnection(obj)
            % Verify connection and throw error if not connected
            if ~obj.isConnected
                error('[AirSim] Not connected to AirSim. Start AirSim and reconnect.');
            end
        end
        
        function armDisarm(obj, arm)
            % Arm or disarm the drone
            % 
            % Usage: client.armDisarm(true);  % Arm
            %        client.armDisarm(false); % Disarm
            
            obj.confirmConnection();
            
            url = sprintf('%s/vehicles/%s/arm', obj.baseURL, obj.vehicleName);
            body = struct('arm', arm);
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            try
                webwrite(url, body, options);
                if arm
                    fprintf('[AirSim] Drone armed\n');
                else
                    fprintf('[AirSim] Drone disarmed\n');
                end
            catch ME
                warning('[AirSim] Arm/Disarm failed: %s', ME.message);
            end
        end
        
        function takeoff(obj, timeout_sec)
            % Command drone to takeoff
            %
            % Usage: client.takeoff(5); % Takeoff with 5 second timeout
            
            obj.confirmConnection();
            
            if nargin < 2
                timeout_sec = 5;
            end
            
            url = sprintf('%s/vehicles/%s/takeoff', obj.baseURL, obj.vehicleName);
            body = struct('timeout_sec', timeout_sec);
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            
            try
                webwrite(url, body, options);
                fprintf('[AirSim] Takeoff commanded (waiting %ds)\n', timeout_sec);
            catch ME
                warning('[AirSim] Takeoff failed: %s', ME.message);
            end
        end
        
        function state = getState(obj)
            % Get current drone state (position, velocity, orientation)
            % 
            % Returns struct with fields:
            %   - position: [x; y; z] in NED frame (meters)
            %   - velocity: [vx; vy; vz] in NED frame (m/s)
            %   - orientation: [w; x; y; z] quaternion
            %   - timestamp: simulation time
            
            obj.confirmConnection();
            
            url = sprintf('%s/vehicles/%s/state', obj.baseURL, obj.vehicleName);
            
            try
                options = weboptions('Timeout', obj.timeout);
                rawState = webread(url, options);
                
                % Parse position (NED frame)
                state.position = [
                    rawState.kinematics_estimated.position.x_val;
                    rawState.kinematics_estimated.position.y_val;
                    rawState.kinematics_estimated.position.z_val
                ];
                
                % Parse velocity (NED frame)
                state.velocity = [
                    rawState.kinematics_estimated.linear_velocity.x_val;
                    rawState.kinematics_estimated.linear_velocity.y_val;
                    rawState.kinematics_estimated.linear_velocity.z_val
                ];
                
                % Parse orientation (quaternion: w, x, y, z)
                state.orientation = [
                    rawState.kinematics_estimated.orientation.w_val;
                    rawState.kinematics_estimated.orientation.x_val;
                    rawState.kinematics_estimated.orientation.y_val;
                    rawState.kinematics_estimated.orientation.z_val
                ];
                
                % Timestamp
                state.timestamp = rawState.timestamp;
                
            catch ME
                warning('[AirSim] Failed to get state: %s', ME.message);
                % Return zero state on failure
                state = struct('position', [0;0;0], ...
                              'velocity', [0;0;0], ...
                              'orientation', [1;0;0;0], ...
                              'timestamp', 0);
            end
        end
        
        function moveByVelocity(obj, vx, vy, vz, duration)
            % Send velocity command to drone
            %
            % Inputs:
            %   vx: velocity in North direction (m/s)
            %   vy: velocity in East direction (m/s)
            %   vz: velocity in Down direction (m/s) - negative = up
            %   duration: command duration (seconds)
            %
            % Usage: client.moveByVelocity(2, 0, -1, 1); % Move North + Up
            
            obj.confirmConnection();
            
            url = sprintf('%s/vehicles/%s/move_by_velocity', ...
                         obj.baseURL, obj.vehicleName);
            
            % Prepare request body
            body = struct(...
                'vx', vx, ...
                'vy', vy, ...
                'vz', vz, ...
                'duration', duration, ...
                'drivetrain', 0, ...
                'yaw_mode', struct('is_rate', false, 'yaw_or_rate', 0) ...
            );
            
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            
            try
                webwrite(url, body, options);
            catch ME
                warning('[AirSim] Move command failed: %s', ME.message);
            end
        end
        
        function moveToPosition(obj, x, y, z, velocity, timeout_sec)
            % Move drone to specific position
            %
            % Inputs:
            %   x, y, z: target position in NED frame (meters)
            %   velocity: movement speed (m/s)
            %   timeout_sec: maximum time allowed (seconds)
            
            obj.confirmConnection();
            
            if nargin < 5
                velocity = 1.0;
            end
            if nargin < 6
                timeout_sec = 10;
            end
            
            url = sprintf('%s/vehicles/%s/move_to_position', ...
                         obj.baseURL, obj.vehicleName);
            
            body = struct(...
                'x', x, 'y', y, 'z', z, ...
                'velocity', velocity, ...
                'timeout_sec', timeout_sec, ...
                'drivetrain', 0, ...
                'yaw_mode', struct('is_rate', false, 'yaw_or_rate', 0) ...
            );
            
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            
            try
                webwrite(url, body, options);
                fprintf('[AirSim] Moving to [%.1f, %.1f, %.1f]\n', x, y, z);
            catch ME
                warning('[AirSim] Move to position failed: %s', ME.message);
            end
        end
        
        function hover(obj)
            % Command drone to hover at current position
            
            obj.confirmConnection();
            
            url = sprintf('%s/vehicles/%s/hover', obj.baseURL, obj.vehicleName);
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            
            try
                webwrite(url, struct(), options);
                fprintf('[AirSim] Hover engaged\n');
            catch ME
                warning('[AirSim] Hover failed: %s', ME.message);
            end
        end
        
        function land(obj, timeout_sec)
            % Command drone to land
            
            obj.confirmConnection();
            
            if nargin < 2
                timeout_sec = 10;
            end
            
            url = sprintf('%s/vehicles/%s/land', obj.baseURL, obj.vehicleName);
            body = struct('timeout_sec', timeout_sec);
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            
            try
                webwrite(url, body, options);
                fprintf('[AirSim] Landing commanded\n');
            catch ME
                warning('[AirSim] Land failed: %s', ME.message);
            end
        end
        
        function reset(obj)
            % Reset simulation to initial state
            
            url = sprintf('%s/reset', obj.baseURL);
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            
            try
                webwrite(url, struct(), options);
                fprintf('[AirSim] Simulation reset\n');
                pause(1); % Wait for reset to complete
            catch ME
                warning('[AirSim] Reset failed: %s', ME.message);
            end
        end
        
        function enableApiControl(obj, enable)
            % Enable or disable API control
            
            obj.confirmConnection();
            
            url = sprintf('%s/vehicles/%s/enable_api_control', ...
                         obj.baseURL, obj.vehicleName);
            body = struct('enable', enable);
            options = weboptions('RequestMethod', 'post', ...
                                 'MediaType', 'application/json', ...
                                 'Timeout', obj.timeout);
            
            try
                webwrite(url, body, options);
                if enable
                    fprintf('[AirSim] API control enabled\n');
                else
                    fprintf('[AirSim] API control disabled\n');
                end
            catch ME
                warning('[AirSim] Enable API control failed: %s', ME.message);
            end
        end
    end
end