clear;
addpath '/home/arnar/taflarmurinn'
% Create a new controller
controller = robotController(true);

%%
try
controller.stopTimer();
% Create a view for the led controller
robotControlPanel('controller', controller);
controller.startTimer();
catch e
    disp(e)
end

%%
if(0)
%%
    clear;
    stop(timerfind);
    controller.stopTimer();
    try
        delete(controller);
    catch
        
    end
    stop(timerfind);
    clear controller;
    
    if(0)
        %%
        controller.gripperOpen()
        %%
        controller.gripperClose()
        %%
    end
%%
end


%%

controller.gripperOpen();
controller.setJointPositions([90, 75, 135, 135, 105, 120]);
pause(3);
controller.setJointPositions([90, 90, 150, 170, 130, 45]);
pause(1);
controller.setJointPositions([90, 90, 165, 170, 115, 45]);
pause(1);
controller.setJointPositions([90, 90, 175, 170, 120, 45]);
pause(1);
controller.gripperClose();
pause(1);
controller.setJointPositions([90, 90, 175, 170, 120, 45]);
pause(1);
controller.setJointPositions([90, 90, 165, 170, 115, 45]);
pause(1);
controller.setJointPositions([90, 90, 150, 170, 130, 45]);
pause(1);
controller.setJointPositions([90, 75, 135, 135, 180, 120]);
pause(2);
controller.setJointPositions([90, 90, 90, 90, 90, 90]);
pause(7);

controller.setJointPositions([90, 75, 135, 135, 105, 120]);
pause(3);
controller.setJointPositions([90, 90, 150, 170, 130, 45]);
pause(1);
controller.setJointPositions([90, 90, 165, 170, 115, 45]);
pause(1);
controller.setJointPositions([90, 90, 175, 170, 120, 45]);
pause(1);
controller.gripperOpen();
pause(1);
controller.setJointPositions([90, 90, 175, 170, 120, 45]);
pause(1);
controller.setJointPositions([90, 90, 165, 170, 115, 45]);
pause(1);
controller.setJointPositions([90, 90, 150, 170, 130, 45]);
pause(1);
controller.setJointPositions([90, 75, 135, 135, 180, 120]);
pause(2);
controller.setJointPositions([90, 90, 90, 90, 90, 90]);
pause(3);

%%

controller.setJointPositions([90, 90, 90, 90, 90, 90]);
%%

controller.gripperClose();
%%

controller.gripperOpen();
%%

controller.setJointPositions([90, 90, 169, 165, 107, 45]);
pause(1);
controller.gripperClose();
pause(1);
controller.setJointPositions([90, 45, 150, 75, 105, 90]);
pause(3);
controller.setJointPositions([90, 90, 90, 90, 90, 90]);


%%
controller.moveStepper(10, 0);