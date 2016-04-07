% Package needed for this to work
%http://se.mathworks.com/hardware-support/arduino-matlab.html
clear a ledPin ledState blinkIntervalInSeconds serialCommunicationPort;

%Configuration
ledPin = 'D13';
servoPin = 'D4';
xPin = 'A1';
yPin = 'A0';
serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'     
blinkIntervalInSeconds = 1;

%Setup
a = arduino(serialCommunicationPort, 'uno');
configurePin(a, ledPin, 'DigitalOutput');
configurePin(a, xPin, 'AnalogInput');
configurePin(a, yPin, 'AnalogInput');

%Program
ledState = true;
writeDigitalPin(a, ledPin, ledState);

clear s;
s = servo(a, 'D4', 'MinPulseDuration', 850*10^-6, 'MaxPulseDuration', 3850*10^-6);

tic
while(true)
    writePosition(s, readVoltage(a, xPin)/5);
    if(toc > 0.2)
        tic
        readVoltage(a, xPin)
    end
end

writePosition(s, 0/180);
pause(1);


