% Package needed for this to work
%http://se.mathworks.com/hardware-support/arduino-matlab.html
clear a ledPin ledState blinkIntervalInSeconds serialCommunicationPort;

%% Configuration
ledPin = 'D13';
endRotatorServoPin = 'D4';
s2Pin = 'D5';
servoPins = ['D6', 'D7', 'D8', 'D9', 'D10', 'D11'];
theta2ServoPin = 'D6'; 
theta3ServoPin = 'D7';
theta4ServoPin = 'D8';
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
endRotateServo = servo(a, endRotatorServoPin, 'MinPulseDuration', 850*10^-6, 'MaxPulseDuration', 3850*10^-6);

s2 = servo(a, s2Pin, 'MinPulseDuration', 720*10^-6, 'MaxPulseDuration', 2400*10^-6); %% servoinn frá oddi

servos(1) = servo(a, servoPins(1), 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2520*10^-6);
servos(2) = servo(a, servoPins(2), 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2350*10^-6);
servos(3) = servo(a, servoPins(3), 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2600*10^-6);


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

end
