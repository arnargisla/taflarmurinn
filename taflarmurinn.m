% Package needed for this to work
%http://se.mathworks.com/hardware-support/arduino-matlab.html
clear a ledPin ledState blinkIntervalInSeconds serialCommunicationPort;

%% Configuration
endRotatorServoPin = 'D12';
s2Pin = 'D5';

stepperDirPin = 'D4';
stepperStepPin = 'D5';
theta2AServoPin = 'D6';
theta2BServoPin = 'D7';

theta3ServoPin = 'D8';
theta4ServoPin = 'D9';
theta5FirstServoPin = 'D10';
theta5SecondServoPin = 'D11';

theta6ServoPin = 'D12';
ledPin = 'D13';


serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
serialCommunicationPort2 = '/dev/ttyS102';
blinkIntervalInSeconds = 1;



%% Setup arduino
try
    a = arduino(serialCommunicationPort, 'uno');
catch exception
    disp 'port 1 failed'
    try
        a = arduino(serialCommunicationPort2, 'uno');
    catch exception
        disp 'port 2 failed'
    end
end
%% 

configurePin(a, ledPin, 'DigitalOutput');
configurePin(a, stepperDirPin, 'DigitalOutput');
configurePin(a, stepperStepPin, 'DigitalOutput');

%% Setup rest

if 0
    %% Set up servos
    clear theta2Servo theta3Servo theta4Servo servo;

    % s2 = servo(a, s2Pin, 'MinPulseDuration', 720*10^-6, 'MaxPulseDuration', 2400*10^-6); %% servoinn frá oddi

    theta2AServo = servo(a, theta2AServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2520*10^-6);
    theta3Servo = servo(a, theta3ServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2350*10^-6);
    theta4Servo = servo(a, theta4ServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2600*10^-6);
    
    % vantar t5A og t5B
    theta6Servo = servo(a, theta6ServoPin, 'MinPulseDuration', 850*10^-6, 'MaxPulseDuration', 3850*10^-6);


    s = endRotateServo;

    %%
    readPosition(s)
    %%
    %%
    clear servo
    servo = bottomServo;
    writePosition(servo, 90/180);
    pause(0.7)
    writePosition(servo, 180/180);
    pause(2)
    writePosition(servo, 0/180);
    pause(2)
    %%


    writePosition(servo, 0/180);
    pause(2)
    writePosition(servo, 90/180);
    pause(2)
    writePosition(servo, 180/180);


    %%
    servo = bottomServo
    %%
    servo = theta2Servo
    %%

    writePosition(servo, 90/180);
    %%
    writePosition(servo, 90/180);
    %%
    writePosition(servo, 180/180);

    %% Run stepper

    writeDigitalPin(a, stepperDirPin, 1);
    for i = 1:10
        writeDigitalPin(a, stepperStepPin, 1);
        writeDigitalPin(a, stepperStepPin, 0);
    end

%%
end
