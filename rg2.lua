sim=require'sim'

function sysCall_init()
    motorHandle=sim.getObject('../openCloseJoint')
    centerJoint=sim.getObject('../centerJoint')
    motorVelocity=0.5 -- m/s
    motorForce=20 -- N
    
    connector=sim.getObject('../attachPoint')
    objectSensor=sim.getObject('../attachProxSensor')
    
    lock = true
    
    attachFlag = 1
end

function sysCall_actuation()
    local v=-motorVelocity
    --local data=sim.getIntProperty(sim.handle_scene, 'signal.RG2_open', {noError = true})
    local data=sim.getInt32Signal('RG2_open')
    
    --if data and data~=0 then
    --    v=motorVelocity
    --end
    
    if data == 1 then
        v = motorVelocity
        loose(connector,objectSensor)
    elseif data == 2 then
        v= motorVelocity
        loose(connector,objectSensor,true)
    else
        tighten(connector,objectSensor)
    end
    
    sim.setJointTargetForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
    
end

function tighten(father,fatherSensor)
    index = 0
    while true do 
        shape = sim.getObjects(index,sim.object_shape_type)
        if(shape == -1) then
            break
        end
        
        res,dis = sim.checkProximitySensor(fatherSensor,shape)
        if res~=nil and dis~=nil then
            _,para1 = sim.getObjectInt32Param(shape,sim.shapeintparam_respondable)
            _,para2 = sim.getObjectInt32Param(shape,sim.shapeintparam_static)
            if para1 == 1 and para2 == 0 then
                if attachFlag >dis+0.3 then
                    attachFlag = dis
                else
                    attachedShape = shape
                    sim.setObjectParent(attachedShape,father,true)
                    lock = false
                    break
                end
            end
        end
        index = index + 1
    end
end

function loose(father,fatherSensor,make_static)
    child = sim.getObjectChild(father,0)
    if child ~= -1 then
        sim.setObjectParent(child,-1,true)
        attachFlag = 1
        if make_static == true then
            sim.setModelProperty(child,sim.modelproperty_not_dynamic)
        end
    end
end



--    function sysCall_joint(inData)
--        if inData.handle==centerJoint then
--            local error=(-sim.getJointPosition(motorHandle)/2)-inData.currentPos 
--            local ctrl=error*20 
--            local velocityToApply=ctrl 
--            if (velocityToApply>inData.maxVel) then 
--                velocityToApply=inData.maxVel 
--            end 
--            if (velocityToApply<-inData.maxVel) then 
--                velocityToApply=-inData.maxVel 
--            end 
--            local forceOrTorqueToApply=inData.maxForce 
--            local outData={vel=velocityToApply,force=forceOrTorqueToApply} 
--            return outData 
--        end
--    end 

    -- You have basically 2 alternatives to grasp an object:
    --
    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
    --    coefficients, object masses and inertias)
    --
    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
    --    much easier and offers very stable results.
    --
    -- Alternative 2 is explained hereafter:
    --
    --
    -- a) In the initialization phase, retrieve some handles:
    -- 
    -- connector=sim.getObject('../attachPoint')
    -- objectSensor=sim.getObject('../attachProxSensor')
    
    -- b) Before closing the gripper, check which dynamically non-static and respondable object is
    --    in-between the fingers. Then attach the object to the gripper:
    --
    -- index=0
    -- while true do
    --     shape=sim.getObjects(index,sim.sceneobject_shape)
    --     if (shape==-1) then
    --         break
    --     end
    --     if sim.getBoolProperty(shape, 'dynamic') and sim.getBoolProperty(shape, 'respondable') and (sim.checkProximitySensor(objectSensor,shape)==1) then
    --         -- Ok, we found a non-static respondable shape that was detected
    --         attachedShape=shape
    --         -- Do the connection:
    --         sim.setObjectParent(attachedShape,connector,true)
    --         break
    --     end
    --     index=index+1
    -- end
    
    -- c) And just before opening the gripper again, detach the previously attached shape:
    --
    -- sim.setObjectParent(attachedShape,-1,true)
