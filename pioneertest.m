function pioneertest()
    disp('Matlab interfacing CoppeliaSim, with the legacy Remote Api');
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % close all open connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    if clientID > -1
        disp('Connected to remote API server');
        [res, motorL] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
        [res, motorR] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
        
        usensor = zeros(1, 16);
        for i=1:16
            [res, usensor(i)] = sim.simxGetObjectHandle(clientID, ['Pioneer_p3dx_ultrasonicSensor' num2str(i)], sim.simx_opmode_blocking);
            [res, state, point, detectedObj, detectedSurfNormVec] = sim.simxReadProximitySensor(clientID, usensor(i), sim.simx_opmode_streaming);
        end
        
        t = clock;
        startSimT = sim.simxGetLastCmdTime(clientID);
        startT = t(6);
        
        while t(6) - startT < 5
            res = sim.simxSetJointTargetVelocity(clientID, motorL, 1.0, sim.simx_opmode_streaming);
            res = sim.simxSetJointTargetVelocity(clientID, motorR, 0.0, sim.simx_opmode_streaming);
            for i=1:16
                [res, state, point, detectedObj, detectedSurfNormVec] = sim.simxReadProximitySensor(clientID, usensor(i), sim.simx_opmode_buffer);
            end
            t = clock;
            simT = sim.simxGetLastCmdTime(clientID);
            fprintf('System time  %f\nSimTime    %f\n', [t(6)-startT 1e-3*(simT-startSimT)]);
            %disp(t(6)-startT);
        end
        
        t = clock;
        startT = t(6);
        
        while t(6) - startT < 1
            res = sim.simxSetJointTargetVelocity(clientID, motorL, 0.0, sim.simx_opmode_streaming);
            res = sim.simxSetJointTargetVelocity(clientID, motorR, 0.0, sim.simx_opmode_streaming);
            t = clock;
            disp(t(6)-startT);
        end
        sim.simxFinish(clientID);
        
    else
        disp('Failed to connect.')
    end
    sim.delete()
end