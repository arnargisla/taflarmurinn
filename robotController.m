% Package needed for this to work
% http://se.mathworks.com/hardware-support/arduino-matlab.html
classdef robotController < handle
    properties (Constant)
        plottingEnabled = true;
        ledPin = 'D13';
        serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
        serialCommunicationPort2 = '/dev/ttyS102';
        serialCommunicationPort3 = '/dev/ttyS103';
        serialCommunicationPort4 = '/dev/ttyS104';
        
        boardType = 'uno'; %'Mega2560';
        gripperPotPin = 'D2';
        gripperServoPin = 'D3';
        
        stepperDirPin = 'D4';
        stepperStepPin = 'D5';
        theta2AServoPin = 'D6';
        theta2BServoPin = 'D7';

        theta3ServoPin = 'D8';
        theta4ServoPin = 'D9';
        theta5AServoPin = 'D10';
        theta5BServoPin = 'D11';

        theta6ServoPin = 'D12';
        controllerOnPin = 'D13';
        
        controlPin1 = 'A0';
        controlPin2 = 'A1';
        controlPin3 = 'A2';
        controlPin4 = 'A3';
        controlPin5 = 'A4';
        controlPin6 = 'A5';
        
    end
    properties (GetAccess=private)
        arduinoConnected = true;
        arduino;
        robot;
        link1, link2, link3, link4, link5, link6;
        jointPositions;
        robotFigure;
        
        theta2AServo;
        theta2BServo;
        theta3Servo;
        theta4Servo;
        
        theta5AServo;
        theta5BServo;
        theta6Servo;
        
        gripperServo;
        
        robotControlTimer;
        motorStepSize = 7;
        timerPeriod = 0.2;
        
        gripperState = 1;
        lastGripperState = 1;
        
        servosOn = true;
        remoteEnabled = true;
        
        stepperPosition = 100;
    end
    methods
        function self=robotController(arduinoConnected)
            self.arduinoConnected = arduinoConnected;
            
            self.jointPositions = [90 90 90 90 90 90];

            %+---+-----------+-----------+-----------+-----------+-----------+             
            %| j |     theta |         d |         a |     alpha |    offset |             
            %+---+-----------+-----------+-----------+-----------+-----------+             
            %|  1|         q1|          0|          0|      1.571|          0|             
            %|  2|         q2|          0|     0.4318|          0|          0|             
            %|  3|         q3|       0.15|     0.0203|     -1.571|          0|             
            %|  4|         q4|     0.4318|          0|      1.571|          0|             
            %|  5|         q5|          0|          0|     -1.571|          0|             
            %|  6|         q6|          0|          0|          0|          0|             
            %+---+-----------+-----------+-----------+-----------+-----------+             


            % Define the links using Denavit-Hartenberg (D-H) parameters
            self.link1 = Link('d',  0.3, 'a',      0.0, 'alpha', pi/2, 'offset', -pi/2);
            self.link2 = Link('d',    0, 'a',      0.2, 'alpha',    0, 'offset',     0);
            self.link3 = Link('d',    0, 'a',      0.2, 'alpha',    0, 'offset', -pi/2);
            self.link4 = Link('d',    0, 'a',      0.2, 'alpha',    0, 'offset', -pi/2);
            self.link5 = Link('d',    0, 'a',      0.2, 'alpha', pi/2, 'offset', -pi/2);
            self.link6 = Link('d',    0, 'a',      0.1, 'alpha',    0, 'offset', -pi/2);

            % Set joint limits
            %L(1) does not have any limits
            self.link2.qlim = pi/180*[-90 90];
            self.link3.qlim = pi/180*[-90 90];
            self.link4.qlim = pi/180*[-90 90];
            self.link5.qlim = pi/180*[-90 90];
            self.link6.qlim = pi/180*[-90 90];

            links = [self.link1, self.link2, self.link3, self.link4, self.link5, self.link6];

            % Define the robot, composed of the links
            self.robot = SerialLink(links);
            self.robot.name = 'Taflarmur';
            self.robotFigure = figure;
            figure(self.robotFigure);
            self.robot.plot(self.jointPositions, 'nojaxes', 'nojvec', 'joints');
            self.moveRobot();
            
            if(self.arduinoConnected)
                disp 'initializing arduino ...'
                try
                    self.arduino = arduino(self.serialCommunicationPort, self.boardType);
                catch exception
                    disp 'port 1 failed'
                    disp(exception.message)
                    try
                        self.arduino = arduino(self.serialCommunicationPort2, self.boardType);
                    catch exception
                        disp 'port 2 failed'
                        disp(exception.message)
                        try
                            self.arduino = arduino(self.serialCommunicationPort3, self.boardType);
                        catch exception
                            disp 'port 3 failed'
                            disp(exception.message)
                            try
                                self.arduino = arduino(self.serialCommunicationPort4, self.boardType);
                            catch exception
                                disp 'port 4 failed'
                                disp(exception.message)
                            end
                        end
                    end
                end
                
                
                disp 'done ... initializing arduino'
                
                % stup po
                
                configurePin(self.arduino, self.controlPin1, 'AnalogInput');
                configurePin(self.arduino, self.controlPin2, 'AnalogInput');
                configurePin(self.arduino, self.controlPin3, 'AnalogInput');
                configurePin(self.arduino, self.controlPin4, 'AnalogInput');
                configurePin(self.arduino, self.controlPin5, 'AnalogInput');
                configurePin(self.arduino, self.controlPin6, 'AnalogInput');
                
                % setup stepper
                configurePin(self.arduino, self.stepperDirPin, 'DigitalOutput');
                configurePin(self.arduino, self.stepperStepPin, 'DigitalOutput');
                
                configurePin(self.arduino, self.controllerOnPin, 'DigitalInput');
                
                self.setUpServos();
            end
            
            
            %% start timer
            
            
            disp 'starting timer ...'
            self.robotControlTimer = timer;
            self.robotControlTimer.TimerFcn = @(~,thisEvent)self.update();
            self.robotControlTimer.Period = self.timerPeriod;
            self.robotControlTimer.TasksToExecute = 100000000;
            self.robotControlTimer.ExecutionMode = 'fixedRate';

            start(self.robotControlTimer);
            
            disp 'done ... starting timer'
            
            %%
            if(self.gripperState)
                self.gripperOpen();
            else
                self.gripperClose();
            end
            
        end
        function stopTimer(self)
            %stop(self.robotControlTimer);
        end
        function startTimer(self)
            %start(self.robotControlTimer);
        end
        function update(self)
            disp 'update'
            
            if(self.arduinoConnected)
                % self.updateJoint1();
                if(self.servosOn)
                    self.updateJoint2();
                    self.updateJoint3();
                    self.updateJoint4();
                    self.updateJoint5();
                    self.updateJoint6();
                    if(self.lastGripperState ~= self.gripperState)
                        if(self.gripperState)
                            self.gripperOpen();
                        else
                            self.gripperClose();
                        end
                    end
                end
            end
            
            controllerOn = readDigitalPin(self.arduino, self.controllerOnPin);
            if(self.remoteEnabled && controllerOn)
                self.gripperState = readDigitalPin(self.arduino, self.gripperPotPin);
                control1Reading = readVoltage(self.arduino, self.controlPin1);
                control2Reading = readVoltage(self.arduino, self.controlPin2);
                control3Reading = readVoltage(self.arduino, self.controlPin3);
                control4Reading = readVoltage(self.arduino, self.controlPin4);
                control5Reading = readVoltage(self.arduino, self.controlPin5);
                control6Reading = readVoltage(self.arduino, self.controlPin6);
                disp([num2str(control1Reading), ' ', num2str(control2Reading), ' ', num2str(control3Reading), ' ', num2str(control4Reading), ' ', num2str(control5Reading), ' ', num2str(control6Reading)])
                
                d = [0, 0, 0, 0, 0, 0];
                inc = 5;
                
                if(control1Reading > 4)
                    d(1) = inc;
                end
                if(control1Reading < 1)
                    d(1) = -inc;
                end
                
                if(control2Reading > 4)
                    d(2) = inc;
                end
                if(control2Reading < 1)
                    d(2) = -inc;
                end
                
                if(control3Reading > 4)
                    d(3) = inc;
                end
                if(control3Reading < 1)
                    d(3) = -inc;
                end
                
                if(control4Reading > 4)
                    d(4) = inc;
                end
                if(control4Reading < 1)
                    d(4) = -inc;
                end
                
                if(control5Reading > 4)
                    d(5) = inc;
                end
                if(control5Reading < 1)
                    d(5) = -inc;
                end
                
                if(control6Reading > 4)
                    d(6) = inc;
                end
                if(control6Reading < 1)
                    d(6) = -inc;
                end
                
                res = d + self.jointPositions;
                self.setJointPositions(res);
            end
        end
        function setUpServos(self)
            if(self.servosOn)
                disp 'setting up servos ...'
                self.setupTheta2AServo();
                self.setupTheta2BServo();
                self.setupTheta3Servo();
                self.setupTheta4Servo();
                self.setupTheta5AServo();
                self.setupTheta5BServo();
                self.setupTheta6Servo();

                self.setupGripperServo();
                disp 'done ... setting up servos'
            end
        end
        function setupTheta2AServo(self)
            self.theta2AServo = servo(...
                self.arduino, self.theta2AServoPin,...
                'MinPulseDuration', 700*10^-6, ...
                'MaxPulseDuration', 2520*10^-6);
        end
        function setupTheta2BServo(self)
            self.theta2BServo = servo(...
                self.arduino, self.theta2BServoPin,...
                'MinPulseDuration', 700*10^-6, ...
                'MaxPulseDuration', 2400*10^-6);
        end
        function setupTheta3Servo(self)
            self.theta3Servo = servo(...
                self.arduino, self.theta3ServoPin,...
                'MinPulseDuration', 700*10^-6, ...
                'MaxPulseDuration', 2350*10^-6);
        end
        function setupTheta4Servo(self)
            self.theta4Servo = servo(...
                self.arduino, self.theta4ServoPin,...
                'MinPulseDuration', 700*10^-6, ...
                'MaxPulseDuration', 2600*10^-6);
        end
        function setupTheta5AServo(self)
            self.theta5AServo = servo(...
                self.arduino, self.theta5AServoPin,...
                'MinPulseDuration', 850*10^-6, ...
                'MaxPulseDuration', 3500*10^-6);
        end
        function setupTheta5BServo(self)
            self.theta5BServo = servo(...
                self.arduino, self.theta5BServoPin,...
                'MinPulseDuration', 850*10^-6, ...
                'MaxPulseDuration', 3500*10^-6);
        end
        function setupTheta6Servo(self)
            self.theta6Servo = servo(...
                self.arduino, self.theta6ServoPin,...
                'MinPulseDuration', 850*10^-6, ...
                'MaxPulseDuration', 3500*10^-6);
        end
        function setupGripperServo(self)
            self.gripperServo = servo(...
                self.arduino, self.gripperServoPin,...
                'MinPulseDuration', 850*10^-6, ...
                'MaxPulseDuration', 3500*10^-6);
        end
        function gripperState = getGripperState(self)
            gripperState = self.gripperState;
        end
        function writePositionServo(self, servo, pos)
            self.stopTimer();
            writePosition(servo, pos);
            self.startTimer();
        end
        function gripperClose(self)
            self.writePositionServo(self.gripperServo, 0/180);
            pause(0.5);
            self.writePositionServo(self.gripperServo, 130/180);
            self.lastGripperState = self.gripperState;
            self.gripperState = 0;
        end
        function gripperOpen(self)
            self.writePositionServo(self.gripperServo, 180/180);
            pause(0.5);
            self.writePositionServo(self.gripperServo, 175/180);
            self.lastGripperState = self.gripperState;
            self.gripperState = 1;
        end
        function setJoint1Position(self, angle)
            self.jointPositions(1) = angle;
        end
        function setJoint2Position(self, angle)
            self.jointPositions(2) = angle;
        end
        function moveStepper(self, numSteps, dir)
            self.stopTimer();
            writeDigitalPin(self.arduino, self.stepperDirPin, dir);
            self.startTimer();
            for i = 1:numSteps
                self.stopTimer();
                writeDigitalPin(self.arduino, self.stepperStepPin, 1);
                self.startTimer();
                pause(0.01);
                self.stopTimer();
                writeDigitalPin(self.arduino, self.stepperStepPin, 0);
                self.startTimer();
            end
        end
        function updateJoint1(self)
            
            destination = 200 * self.jointPositions(1)/360;
            position = self.stepperPosition;
            difference = abs(destination - position);
            
            
            direction = 1;
            
            numSteps = difference;
            %disp([' numSteps ', num2str(numSteps), ...
            %      ' difference ', num2str(difference)]);
            maxNumSteps = 10;
            if(numSteps > maxNumSteps)
                numSteps = maxNumSteps;
            end
            if(difference < 1)
                numSteps = 0;
            end
            %disp([' direction ', num2str(direction), ...
            %      ' numSteps ', num2str(numSteps), ...
            %      ' position ', num2str(position), ...
            %      ' destination ', num2str(destination), ...
            %      ' position ', num2str(numSteps)]);
            if(direction == 1)
                self.stepperPosition = mod(self.stepperPosition + numSteps, 200);
            else
                self.stepperPosition = mod(self.stepperPosition - numSteps, 200);
            end
            
            writeDigitalPin(self.arduino, self.stepperDirPin, direction);
            
            for i = 1:numSteps
                writeDigitalPin(self.arduino, self.stepperStepPin, 1);
                writeDigitalPin(self.arduino, self.stepperStepPin, 0);
            end
            
        end
        function updateJoint2(self)
            self.updateJoint(self.theta2AServo, 2);
            self.updateJoint(self.theta2BServo, 2, true);
        end
        function updateJoint3(self)
            self.updateJoint(self.theta3Servo, 3);
        end
        function updateJoint4(self)
            self.updateJoint(self.theta4Servo, 4);
        end
        function updateJoint5(self)
            self.updateJoint(self.theta5BServo, 5);
            self.updateJoint(self.theta5AServo, 5, true);
        end
        function updateJoint6(self)
            self.updateJoint(self.theta6Servo, 6);
        end
        function updateJoint(self, servo, jointIndex, invert)
            if nargin < 4
                invert = false;
            end
            
            position = 180 * readPosition(servo);            
            destination = self.jointPositions(jointIndex);
            if(invert)
                destination = 180 - destination;
            end
            difference = abs(destination - position);
            if(position < destination)
                nextPosition = position + self.motorStepSize;
            else
                nextPosition = position - self.motorStepSize;
            end
            
            if(difference < self.motorStepSize)
                nextPosition = destination;
            end
            if(difference < 0.5)
                nextPosition = destination;
            end
%             disp(['position ', num2str(position),...
%                  ' destination ', num2str(destination),...
%                  ' difference ', num2str(difference),...
%                  ' nextPosition ', num2str(nextPosition),...
%                  ' invert ', num2str(invert)]);
            self.moveServoToInstant(servo, nextPosition);
        end
        function moveServoToInstant(self, servo, destinationAngleInDegrees)
            servoDestination = destinationAngleInDegrees/180;
            
            self.writePositionServo(servo, servoDestination);
        end
        function setJoint3Position(self, angle)
            self.jointPositions(3) = angle;
        end
        function setJoint4Position(self, angle)
            self.jointPositions(4) = angle;
        end
        function setJoint5Position(self, angle)
            self.jointPositions(5) = angle;
        end
        function setJoint6Position(self, angle)
            self.jointPositions(6) = angle;
        end
        function positions = getJointPositions(self)
            positions = self.jointPositions;
        end
        function setJointPositions(self, positions)
            joint1pos = mod(positions(1), 360);
            positions = min(positions, 180);
            positions = max(positions, 0);
            setJoint1Position(self, joint1pos);
            setJoint2Position(self, positions(2));
            setJoint3Position(self, positions(3));
            setJoint4Position(self, positions(4));
            setJoint5Position(self, positions(5));
            setJoint6Position(self, positions(6));
        end
        function moveRobot(self)

            % Generate a time vector of 4 seconds, in 0.1 sec steps, total 81 steps.
            % t = 0:0.1:4;

            % Define the starting positions of the six joints
            % qi = self.jointPositions;

            % Define the final positions of the six joints
            % qf = [pi/4 0 pi/4 0 0 0];
            
            % Compute the joint coordinate trajectory.
            % The function jtraj() creates a trajectory for each joint so that
            % the initial and final velocity and acceleration of each joint are zero.
            % q are the joint positions, qd are joint velocities and qdd accelerations.
            % [q, ~, ~] = jtraj(qi, qf, t);

            
            % make sure we plot on the correct figure
            % figure(self.robotFigure);
            
            % Animate end effector trajectory in 3D space
            %plot(self.robot, q);%, 'workspace', [-22 38 -22 38 -22 38]);
            
            % self.robot.plot(self.jointPositions, 'nojaxes', 'nojvec', 'joints');
            %plot(self.robot, self.jointPositions);
            self.robot.animate(pi/180 * self.jointPositions);
            
        end
        function delete(obj)
            stop(obj.robotControlTimer);
        end
    end
end

