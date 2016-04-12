% Package needed for this to work
%http://se.mathworks.com/hardware-support/arduino-matlab.html
clear a ledPin ledState blinkIntervalInSeconds serialCommunicationPort;

%% Configuration
ledPin = 'D13';
s1Pin = 'D4';
s2Pin = 'D5';
bottomServoPin = 'D6'; % Neðsti servoinn
theta2ServoPin = 'D7'; % n;st neðsti servoinn
theta3ServoPin = 'D8'; % n;st n;st neðsti servoinn
xPin = 'A1';
yPin = 'A0';
serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
serialCommunicationPort2 = '/dev/ttyS102';
blinkIntervalInSeconds = 1;

%% Setup arduino
try
    a = arduino(serialCommunicationPort, 'uno');
catch exception
    disp 'port 1 failed'
end
try
    a = arduino(serialCommunicationPort2, 'uno');
catch exception
    disp 'port 2 failed'
end
%% 

configurePin(a, ledPin, 'DigitalOutput');

%% Setup rest

clear s1 s2 bottomServo theta2Servo theta3Servo servo s;
endRotateServo = servo(a, s1Pin, 'MinPulseDuration', 850*10^-6, 'MaxPulseDuration', 3850*10^-6);
s2 = servo(a, s2Pin, 'MinPulseDuration', 720*10^-6, 'MaxPulseDuration', 2400*10^-6);
bottomServo = servo(a, bottomServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2520*10^-6);
theta2Servo = servo(a, theta2ServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2350*10^-6);
theta3Servo = servo(a, theta3ServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2600*10^-6);


s = endRotateServo;

%%
readPosition(s)
%%
if 0
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

%%


%%
    writePosition(s1, 90/180);
    writePosition(bottomServo, 90/180);
    writePosition(theta2Servo, 90/180);
    writePosition(theta3Servo, 90/180);
    pause(2);
    writePosition(s1, 0/180);
    writePosition(bottomServo, 60/180);
    writePosition(theta2Servo, 110/180);
    writePosition(theta3Servo, 70/180);
    pause(2);
    writePosition(s1, 90/180);
    writePosition(bottomServo, 90/180);
    writePosition(theta2Servo, 90/180);
    writePosition(theta3Servo, 90/180);
    pause(2);
    writePosition(s1, 180/180);
    writePosition(bottomServo, 110/180);
    writePosition(theta2Servo, 60/180);
    writePosition(theta3Servo, 110/180);
    pause(2);
    writePosition(s1, 90/180);
    writePosition(bottomServo, 90/180);
    writePosition(theta2Servo, 90/180);
    writePosition(theta3Servo, 90/180);

    %%
    
    writePosition(bottomServo, 90/180);
    writePosition(theta2Servo, 90/180);
    pause(2);
    writePosition(bottomServo, 120/180);
    writePosition(theta2Servo, 90/180);
    pause(2);
    writePosition(bottomServo, 90/180);
    writePosition(theta2Servo, 90/180);
    pause(2);
    writePosition(bottomServo, 50/180);
    writePosition(theta2Servo, 90/180);
    pause(2);
    writePosition(bottomServo, 90/180);
    writePosition(theta2Servo, 90/180);
end
