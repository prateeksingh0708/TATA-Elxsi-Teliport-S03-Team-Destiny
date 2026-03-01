classdef PID_Controller < handle
    % PID_Controller - Three-axis PID controller for quadcopter
    % Implements proportional-integral-derivative control with anti-windup
    
    properties
        Kp                % Proportional gains [x; y; z]
        Ki                % Integral gains [x; y; z]
        Kd                % Derivative gains [x; y; z]
        
        errorIntegral     % Accumulated error for integral term
        previousError     % Previous error for derivative calculation
        dt                % Control loop time step (seconds)
        
        integralLimit     % Anti-windup saturation limit
        outputLimit       % Output saturation limit (m/s)
        
        % Logging
        errorHistory      % History of errors
        outputHistory     % History of control outputs
    end
    
    methods
        function obj = PID_Controller(Kp, Ki, Kd, dt)
            % Constructor
            %
            % Inputs:
            %   Kp: [Kp_x; Kp_y; Kp_z] proportional gains
            %   Ki: [Ki_x; Ki_y; Ki_z] integral gains
            %   Kd: [Kd_x; Kd_y; Kd_z] derivative gains
            %   dt: time step (seconds)
            %
            % Usage:
            %   pid = PID_Controller([1.5; 1.5; 2.0], [0.1; 0.1; 0.2], ...
            %                        [0.8; 0.8; 1.0], 0.1);
            
            obj.Kp = Kp(:);
            obj.Ki = Ki(:);
            obj.Kd = Kd(:);
            obj.dt = dt;
            
            % Initialize states
            obj.errorIntegral = [0; 0; 0];
            obj.previousError = [0; 0; 0];
            
            % Set limits
            obj.integralLimit = 10;   % Prevent integral windup
            obj.outputLimit = 8;      % Maximum output velocity (m/s)
            
            % Initialize logging
            obj.errorHistory = [];
            obj.outputHistory = [];
        end
        
        function output = compute(obj, error)
            % Compute PID control output
            %
            % Input:
            %   error: [e_x; e_y; e_z] position error vector
            %
            % Output:
            %   output: [u_x; u_y; u_z] control velocity command
            
            error = error(:);
            
            % Proportional term: P = Kp * e
            P = obj.Kp .* error;
            
            % Integral term with anti-windup: I = Ki * ∫e dt
            obj.errorIntegral = obj.errorIntegral + error * obj.dt;
            
            % Apply anti-windup saturation
            obj.errorIntegral = max(min(obj.errorIntegral, obj.integralLimit), ...
                                   -obj.integralLimit);
            
            I = obj.Ki .* obj.errorIntegral;
            
            % Derivative term: D = Kd * de/dt
            errorDerivative = (error - obj.previousError) / obj.dt;
            D = obj.Kd .* errorDerivative;
            
            % Combined PID output
            output = P + I + D;
            
            % Apply output saturation
            outputMag = norm(output);
            if outputMag > obj.outputLimit
                output = obj.outputLimit * output / outputMag;
            end
            
            % Update state
            obj.previousError = error;
            
            % Log data
            obj.errorHistory = [obj.errorHistory, error];
            obj.outputHistory = [obj.outputHistory, output];
        end
        
        function reset(obj)
            % Reset controller state (useful between runs)
            
            obj.errorIntegral = [0; 0; 0];
            obj.previousError = [0; 0; 0];
            obj.errorHistory = [];
            obj.outputHistory = [];
            
            fprintf('[PID] Controller reset\n');
        end
        
        function setGains(obj, Kp, Ki, Kd)
            % Update controller gains during runtime
            %
            % Usage: pid.setGains([2.0; 2.0; 2.5], [0.2; 0.2; 0.3], ...
            %                     [1.0; 1.0; 1.2]);
            
            obj.Kp = Kp(:);
            obj.Ki = Ki(:);
            obj.Kd = Kd(:);
            
            fprintf('[PID] Gains updated\n');
        end
        
        function setLimits(obj, integralLimit, outputLimit)
            % Update saturation limits
            
            obj.integralLimit = integralLimit;
            obj.outputLimit = outputLimit;
            
            fprintf('[PID] Limits updated: Integral=%.1f, Output=%.1f\n', ...
                    integralLimit, outputLimit);
        end
    end
end