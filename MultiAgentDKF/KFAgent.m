classdef KFAgent < BasicAgent
    properties
        H = zeros(2,2);
        z = 0;          % 
        ix = 0;         % internal state
    end
    
    methods
        
      % constructor
      function obj = KFAgent(pos,vel)
          %init KF related variable
          
          obj = obj@BasicAgent(pos,vel);
      end
      
      function obj = observe(obj, percepted_agent)
          
      end
      
    end
end