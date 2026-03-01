classdef DroneSimulator < handle
    properties
        position
        velocity
        dt
        cmdVelocity
    end
    methods
        function obj = DroneSimulator(initPos, dt)
            obj.position    = initPos(:);
            obj.velocity    = [0; 0; 0];
            obj.dt          = dt;
            obj.cmdVelocity = [0; 0; 0];
        end
        function state = getState(obj)
            state.position = obj.position;
            state.velocity = obj.velocity;
        end
        function moveByVelocity(obj, vx, vy, vz, ~)
            obj.cmdVelocity = [vx; vy; vz];
        end
        function hover(obj)
            obj.cmdVelocity = [0; 0; 0];
        end
        function update(obj, windForce)
            tau = 0.3;  % response time constant (seconds)
            obj.velocity = obj.velocity + (obj.cmdVelocity - obj.velocity) * (obj.dt / tau);
            obj.velocity = obj.velocity + windForce * obj.dt;
            obj.position = obj.position + obj.velocity * obj.dt;
        end
    end
end