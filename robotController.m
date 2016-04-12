% Package needed for this to work
% http://se.mathworks.com/hardware-support/arduino-matlab.html
classdef robotController < handle
    properties (Constant)
        arduinoConnected = true;
        plottingEnabled = true;
        ledPin = 'D13';
        serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
    end
    properties (GetAccess=private)
        arduino;
        robot;
        link1, link2, link3, link4, link5, link6;
        jointPositions;
        robotFigure;
    end
    methods
        function self=robotController
            
        self.jointPositions = [0 0 0 0 0 0];
            
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
        self.link1 = Link('alpha', pi/2, 'a',    0, 'd', 0.3);
        self.link2 = Link('alpha',   0, 'a',  0.5, 'd', 0.0);
        self.link3 = Link('alpha',    0, 'a',  0.5, 'd', 0.0);
        self.link4 = Link('alpha',    0, 'a',  0.5, 'd', 0.0);
        self.link5 = Link('alpha', pi/2, 'a',  0.5, 'd', 0.0);
        self.link6 = Link('alpha',    0, 'a',  0.5, 'd', 0.5);
        
        
        self.link1 = Link('d',    0.2, 'a',      0, 'alpha',  pi/2);
        self.link2 = Link('d',      0, 'a',    0.0, 'alpha',     0);
        self.link3 = Link('d',      0, 'a', 0.4318, 'alpha',     0);
        self.link4 = Link('d',    0.0, 'a', 0.0203, 'alpha', -pi/2);
        self.link5 = Link('d',    0.0, 'a',      0, 'alpha',  pi/2);
        self.link6 = Link('d',      0, 'a',      0, 'alpha', -pi/2);

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
        self.moveRobot();
            
            if(self.arduinoConnected)
                self.arduino = arduino(self.serialCommunicationPort, 'uno');
                configurePin(self.arduino, self.ledPin, 'DigitalOutput');
            end
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
            t = 0:0.1:4;

            % Define the starting positions of the six joints
            qi = self.jointPositions;

            % Define the final positions of the six joints
            qf = [pi/4 0 pi/4 0 0 0];
            
            % Compute the joint coordinate trajectory.
            % The function jtraj() creates a trajectory for each joint so that
            % the initial and final velocity and acceleration of each joint are zero.
            % q are the joint positions, qd are joint velocities and qdd accelerations.
            [q, ~, ~] = jtraj(qi, qf, t);

            
            % make sure we plot on the correct figure
            figure(self.robotFigure);
            
            % Animate end effector trajectory in 3D space
            %plot(self.robot, q);%, 'workspace', [-22 38 -22 38 -22 38]);
            
            self.robot.plot(self.jointPositions, 'nojaxes', 'nojvec', 'joints');
            %plot(self.robot, self.jointPositions);
            
        end
    end
end

