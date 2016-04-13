classdef timerTestClass < handle
    properties (GetAccess=private)
        myTimer;
    end
    methods
        function self=timerTestClass()
            self.myTimer = timer;
            self.myTimer.Period = 3;
            self.myTimer.TasksToExecute = 3;
            self.myTimer.TimerFcn = @self.timerFunction;
            start(self.myTimer)
            disp('hello');
            
        end
        function timerFunction(self)
            disp('intimer');
        end

    end
end
