classdef DQ_VrepInterface_hexapod < DQ_VrepInterface
    methods
        function v = get_body_velocity_from_vrep(obj)
           v = obj.simGetObjectVelocity('/hexapod');
        end 
        
        
        
    end
end