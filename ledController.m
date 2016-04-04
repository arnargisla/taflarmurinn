% Package needed for this to work
% http://se.mathworks.com/hardware-support/arduino-matlab.html
classdef ledController
    properties (Constant)
        ledPin = 'D13';
        serialCommunicationPort = '/dev/ttyS101'; % NOTE in Windows this is 
                                 ...something like 'COM1' or 'COM23'
    end
    properties (GetAccess=private)
        a;
    end
    methods
        function obj=ledController
            obj.a = arduino(obj.serialCommunicationPort, 'uno');
            configurePin(obj.a, obj.ledPin, 'DigitalOutput');

            obj.greet;
            %start gui
            ledcontrolgui(obj, {});
        end
        function turnLedOn(self) 
            writeDigitalPin(self.a, self.ledPin, true);
        end
        function turnLedOff(self)
            writeDigitalPin(self.a, self.ledPin, false);
        end
        function greet(self)
            disp 'hello';
        end
    end
end

