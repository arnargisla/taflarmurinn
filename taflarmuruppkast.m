figure

% Define the links using Denavit-Hartenberg (D-H) parameters
L(1) = Link('alpha', pi/2, 'a',    0, 'd', 0.10);
L(2) = Link('alpha',    0, 'a',  0.25, 'd',   0);
L(3) = Link('alpha',    0, 'a',  0.20, 'd',   0);
L(4) = Link('alpha', -pi/2, 'a', 0, 'd',    0);
L(5) = Link('alpha',  pi/2, 'a', 0, 'd',   0.3);
L(6) = Link('alpha', -pi/2, 'a', 0.3, 'd',   0.0);


% Define the links using Denavit-Hartenberg (D-H) parameters
L(1) = Link('alpha', pi/2, 'a',    0, 'd', 0.10);
L(2) = Link('alpha',    0, 'a',  0.25, 'd',   0);
L(3) = Link('alpha',    0, 'a',  0.20, 'd',   0);
L(4) = Link('alpha', -pi/2, 'a', 0.5, 'd',    0);
L(5) = Link('alpha',  pi/2, 'a', 0, 'd',   0.3);
L(6) = Link('alpha', -pi/2, 'a', 0.3, 'd',   0.0);

% Set joint limits
%L(1) does not have any limits
%L(2).qlim = pi/180*[-90 90];
%L(3).qlim = pi/180*[-90 90];
%L(4).qlim = pi/180*[-90 90];
%L(5).qlim = pi/180*[-90 90];
%L(6).qlim = pi/180*[-90 90];

% Define the robot, composed of the links
MyRobot = SerialLink(L);
MyRobot.name = 'Taflarmur';

%% Prepare and display motion of the robot

% Generate a time vector of 4 seconds, in 0.1 sec steps, total 81 steps.
t = [0:0.1:4];

% Define the starting positions of the six joints
qi = [0 0 0 0 0 0];

% Define the final positions of the six joints
qf = [pi/4 0 pi/4 0 0 0];


% Compute the joint coordinate trajectory.
% The function jtraj() creates a trajectory for each joint so that
% the initial and final velocity and acceleration of each joint are zero.
% q are the joint positions, qd are joint velocities and qdd accelerations.
[q, qd, qdd] = jtraj(qi, qf, t);

% Animate end effector trajectory in 3D space
%MyRobot.plot(q);%, 'workspace', [-22 38 -22 38 -22 38]);
%plot(MyRobot, q);
%hold;


%qi = qf;
%qf = [pi/4 pi/2 pi/4 pi/4 pi/4 pi/4];
%[q, qd, qdd] = jtraj(qi, qf, t);
%MyRobot.plot(q);

%qi = qf;
%qf = [pi/4 pi/4 pi/4 pi/4 pi/4 pi/4];
%[q, qd, qdd] = jtraj(qi, qf, t);
%plot = MyRobot.plot(q);





qf = [0 0 0 0 0 0];
MyRobot.teach();
