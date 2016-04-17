%%
clear;
theta2AServoPin = 'D6';
theta2BServoPin = 'D7';
theta5AServoPin = 'D10';
stepperDirPin = 'D4';
stepperStepPin = 'D5';

gripperServoPin = 'D3';

serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                         ...something like 'COM1' or 'COM23'
serialCommunicationPort2 = '/dev/ttyS102'; 
serialCommunicationPort3 = '/dev/ttyS103'; 
serialCommunicationPort4 = '/dev/ttyS104'; 

try
    arduino = arduino(serialCommunicationPort, 'uno');
catch exception
    disp 'port 1 failed'
        disp(exception)
    try
        arduino = arduino(serialCommunicationPort2, 'uno');
    catch exception
        disp 'port 2 failed'
        disp(exception)
        try
            arduino = arduino(serialCommunicationPort3, 'uno');
        catch exception
            disp 'port 3 failed'
            disp(exception)
            
            try
                arduino = arduino(serialCommunicationPort4, 'uno');
            catch exception
                disp 'port 4 failed'
                disp(exception)
            end
        end
    end
end

%%
clear servo theta2AServo theta2BServo theta5AServo
% 700 2520
%theta5AServo = servo(arduino, theta5AServoPin, 'MinPulseDuration', 850*10^-6, 'MaxPulseDuration', 3500*10^-6);

%gripperServo = servo(arduino, gripperServoPin, 'MinPulseDuration', 850*10^-6, 'MaxPulseDuration', 3500*10^-6);

%theta2AServo = servo(arduino, theta2AServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2550*10^-6);
%theta2BServo = servo(arduino, theta2BServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2400*10^-6);
servo = gripperServo;

%%
moveServoTo(servo, 30, 0.2);
moveServoTo(servo, 180, 0.2);
moveServoTo(servo, 90, 0.2);

%%

writePosition(servo, 90/180);
%%

writePosition(servo, 0/180);
pause(0.5);
writePosition(servo, 130/180);

%%
writePosition(servo, 180/180);
pause(0.5);
writePosition(servo, 175/180);

%%
pos = 60;
writePosition(theta2AServo, (pos)/180);
writePosition(theta2BServo, (180 - pos)/180);

%%
pos = 0;
writePosition(theta2AServo, (pos)/180);
writePosition(theta2BServo, (180 - pos)/180);

%%

configurePin(arduino,stepperDirPin, 'DigitalOutput');
configurePin(arduino,stepperStepPin, 'DigitalOutput');

%%
numSteps = 10;
writeDigitalPin(arduino, stepperDirPin, 1);
for i = 1:numSteps
    writeDigitalPin(arduino, stepperStepPin, 1);
    pause(0.1);
    writeDigitalPin(arduino, stepperStepPin, 0);
end
writeDigitalPin(arduino, stepperDirPin, 0);
for i = 1:numSteps
    writeDigitalPin(arduino, stepperStepPin, 1);
    pause(0.1);
    writeDigitalPin(arduino, stepperStepPin, 0);
end

%% 

writeDigitalPin(arduino, stepperDirPin, 0);
writeDigitalPin(arduino, stepperStepPin, 1);
writeDigitalPin(arduino, stepperStepPin, 0);

%%
writeDigitalPin(arduino, stepperDirPin, 1);
writeDigitalPin(arduino, stepperStepPin, 1);
writeDigitalPin(arduino, stepperStepPin, 0);

