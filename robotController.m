% Package needed for this to work
% http://se.mathworks.com/hardware-support/arduino-matlab.html
classdef robotController < handle
    properties (Constant)
        plottingEnabled = true;
        ledPin = 'D13';
        serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
        serialCommunicationPort2 = '/dev/ttyS102'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
        serialCommunicationPort3 = '/dev/ttyS103'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
        
        stepperDirPin = 'D4';
        stepperStepPin = 'D5';
        theta2AServoPin = 'D6';
        theta2BServoPin = 'D7';

        theta3ServoPin = 'D8';
        theta4ServoPin = 'D9';
        theta5AServoPin = 'D10';
        theta5BServoPin = 'D11';

        theta6ServoPin = 'D12';
        
    end
    properties (GetAccess=private)
        arduinoConnected = true;
        arduino;
        robot;
        link1, link2, link3, link4, link5, link6;
        jointPositions;
        robotFigure;
        
        theta2AServo;
        theta3Servo;
        theta4Servo;
        theta5AServo;
        theta5BServo;
        theta6Servo;
        
        robotControlTimer;
        motorStepSize = 7;
        timerPeriod = 0.5;
        
        
        stepperPosition = 0;
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
                try
                    self.arduino = arduino(self.serialCommunicationPort, 'uno');
                catch exception
                    disp 'port 1 failed'
                    disp(exception)
                    try
                        self.arduino = arduino(self.serialCommunicationPort2, 'uno');
                    catch exception
                        disp 'port 2 failed'
                        disp(exception)
                        try
                            self.arduino = arduino(self.serialCommunicationPort3, 'uno');
                        catch exception
                            disp 'port 3 failed'
                            disp(exception)
                        end
                    end
                end
                
                configurePin(self.arduino, self.ledPin, 'DigitalOutput');
                
                self.setUpServos();
            end
            
            
            %% start timer
            
            
            self.robotControlTimer = timer;
            self.robotControlTimer.TimerFcn = @(~,thisEvent)self.update();
            self.robotControlTimer.Period = self.timerPeriod;
            self.robotControlTimer.TasksToExecute = 100000000;
            self.robotControlTimer.ExecutionMode = 'fixedRate';

            
            start(self.robotControlTimer);
        end
        function update(self)
            if(self.arduinoConnected)
                self.updateJoint1();
                %self.updateJoint2();
                %self.updateJoint3();
                %self.updateJoint4();
                %self.updateJoint5();
                %self.updateJoint6();
            end
        end
        function setUpServos(self)
            self.setupTheta2AServo();
            self.setupTheta3Servo();
            self.setupTheta4Servo();
            self.setupTheta5AServo();
            self.setupTheta5BServo();
            self.setupTheta6Servo();
        end
        function setupTheta2AServo(self)
            self.theta2AServo = servo(...
                self.arduino, self.theta2AServoPin,...
                'MinPulseDuration', 700*10^-6, ...
                'MaxPulseDuration', 2520*10^-6);
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
                'MaxPulseDuration', 3600*10^-6);
        end
        function setupTheta5BServo(self)
            self.theta5BServo = servo(...
                self.arduino, self.theta5BServoPin,...
                'MinPulseDuration', 850*10^-6, ...
                'MaxPulseDuration', 3600*10^-6);
        end
        function setupTheta6Servo(self)
            self.theta6Servo = servo(...
                self.arduino, self.theta6ServoPin,...
                'MinPulseDuration', 850*10^-6, ...
                'MaxPulseDuration', 3400*10^-6);
        end
        function testTheta2AServo(self)
            servo = self.theta2AServo;
            self.testServo(servo);
        end
        function testServo(~, servo)
             disp 'testing servo';
             moveServoTo(servo, 90, 2.0);
             moveServoTo(servo, 110, 2.0);
             moveServoTo(servo, 70, 2.0);
             moveServoTo(servo, 90, 2.0);
        end
        function turnLedOn(self) 
            if(self.arduinoConnected)
                writeDigitalPin(self.arduino, self.ledPin, true);
            end
        end
        function turnLedOff(self)
            if(self.arduinoConnected)
                writeDigitalPin(self.arduino, self.ledPin, false);
            end
        end
        function setJoint1Position(self, angle)
            self.jointPositions(1) = angle;
        end
        function setJoint2Position(self, angle)
            self.jointPositions(2) = angle;
        end
        function updateJoint1(self)
            
            destination = 200 * self.jointPositions(1)/360;
            position = self.stepperPosition;
            difference = abs(destination - position);
            
            
            direction = 1;
            
            numSteps = difference;
            disp([' numSteps ', num2str(numSteps), ...
                  ' difference ', num2str(difference)]);
            maxNumSteps = 10;
            if(numSteps > maxNumSteps)
                numSteps = maxNumSteps;
            end
            if(difference < 1)
                numSteps = 0;
            end
            disp([' direction ', num2str(direction), ...
                  ' numSteps ', num2str(numSteps), ...
                  ' position ', num2str(position), ...
                  ' destination ', num2str(destination), ...
                  ' position ', num2str(numSteps)]);
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
        end
        function updateJoint3(self)
            self.updateJoint(self.theta3Servo, 3);
        end
        function updateJoint4(self)
            self.updateJoint(self.theta4Servo, 4);
        end
        function updateJoint5(self)
            self.updateJoint(self.theta5AServo, 5);
            self.updateJoint(self.theta5BServo, 5, true);
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
            moveServoToInstant(servo, nextPosition);
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
            setJoint1Position(self, positions(1));
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

