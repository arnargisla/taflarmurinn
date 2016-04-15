clear;

addpath('servo');

% Create a new ledController
controller = robotController(true);

% Create a view for the led controller
robotControlPanel('controller', controller);

%%
if(0)
%%
    delete(controller);
    stop(timerfind);
    clear controller;
%%
end
