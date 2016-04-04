% Package needed for this to work
%http://se.mathworks.com/hardware-support/arduino-matlab.html
clear a ledPin ledState blinkIntervalInSeconds serialCommunicationPort;

%Configuration
ledPin = 'D13';
serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
blinkIntervalInSeconds = 1;

%Setup
a = arduino(serialCommunicationPort, 'uno');
configurePin(a, ledPin, 'DigitalOutput');

%Program
ledState = true;
writeDigitalPin(a, ledPin, ledState);
tic
while (true)
    if(toc > blinkIntervalInSeconds)
        tic
        ledState = not (ledState);
        writeDigitalPin(a, ledPin, ledState);
        disp 'blink'
    end
end