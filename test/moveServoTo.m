function [] = moveServoTo( servo, destinationAngleInDegrees, movementTimeInSec)

    %MOVESERVOTOANGLE Summary of this function goes here
    %   Detailed explanation goes here
    
    if nargin < 3
        movementTimeInSec = 2;
    end
    stepSize = 1/180;
    servoPosition = readPosition(servo);
    servoDestination = destinationAngleInDegrees/180;
    numberOfSteps = abs(servoPosition * 180 - destinationAngleInDegrees);
    
    if(numberOfSteps < 2)
        numberOfSteps = 2;
    end
    if(numberOfSteps > 180)
        numberOfSteps = 180;
    end
    
    timePerStep = movementTimeInSec/numberOfSteps;
    
    for i = linspace(servoPosition, servoDestination, numberOfSteps)
        writePosition(servo, i);
        pause(timePerStep);
    end
end

