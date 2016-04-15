function [] = moveServoToInstant( servo, destinationAngleInDegrees)

    %MOVESERVOTOANGLE Summary of this function goes here
    %   Detailed explanation goes here
    
    servoDestination = destinationAngleInDegrees/180;
    writePosition(servo, servoDestination);
end

