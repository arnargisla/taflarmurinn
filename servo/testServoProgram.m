%%
clear;
theta2AServoPin = 'D6';

serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                         ...something like 'COM1' or 'COM23'
serialCommunicationPort2 = '/dev/ttyS102'; % NOTE in Windows this is 
                         ...something like 'COM1' or 'COM23'

try
    arduino = arduino(serialCommunicationPort, 'uno');
catch exception
    disp 'port 1 failed'
    try
        arduino = arduino(serialCommunicationPort2, 'uno');
    catch exception
        disp 'port 2 failed'
    end
end

%%
clear servo theta2AServo
% 700 2520
theta2AServo = servo(arduino, theta2AServoPin, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2500*10^-6);
servo = theta2AServo;

%%
moveServoTo(servo, 70, 1.0);
moveServoTo(servo, 130, 1.0);
moveServoTo(servo, 90, 1.0);

%%
moveServoTo(servo, 30, 1.0);
moveServoTo(servo, 0, 2.0);

%%
moveServoTo(servo, 90, 1.0);

