sim=require'sim'

function sysCall_init() 
    s=sim.getObject('../Sensor')
    l=sim.getObject('../LoopClosureDummy1')
    l2=sim.getObject('../LoopClosureDummy2')
    b=sim.getObject('..')
    suctionPadLink=sim.getObject('../Link')


    sim.setLinkDummy(l,-1)
    sim.setObjectParent(l,b,true)
    m=sim.getObjectMatrix(l2)
    sim.setObjectMatrix(l,m)
end

function sysCall_cleanup() 
    sim.setLinkDummy(l,-1)
    sim.setObjectParent(l,b,true)
    m=sim.getObjectMatrix(l2)
    sim.setObjectMatrix(l,m)
end 

function sysCall_sensing() 
    parent=sim.getObjectParent(l)
    local data=sim.getInt32Signal('RG2_open')
    if data == 1 then
        if (parent~=b) then
            sim.setLinkDummy(l,-1)
            sim.setObjectParent(l,b,true)
            m=sim.getObjectMatrix(l2)
            sim.setObjectMatrix(l,m)
        end
    elseif data == 0 then
        if (parent==b) then
            -- Here we want to detect a respondable shape, and then connect to it with a force sensor (via a loop closure dummy dummy link)
            -- However most respondable shapes are set to "non-detectable", so "sim.readProximitySensor" or similar will not work.
            -- But "sim.checkProximitySensor" or similar will work (they don't check the "detectable" flags), but we have to go through all shape objects!
            index=0
            while true do
                shape=sim.getObjects(index,sim.sceneobject_shape)
                if (shape==-1) then
                    break
                end
                if (shape~=b) and sim.getBoolProperty(shape, 'respondable') and (sim.checkProximitySensor(s,shape)==1) then
                    -- Ok, we found a respondable shape that was detected
                    -- We connect to that shape:
                    -- Make sure the two dummies are initially coincident:
                    sim.setObjectParent(l,b,true)
                    m=sim.getObjectMatrix(l2)
                    sim.setObjectMatrix(l,m)
                    -- Do the connection:
                    sim.setObjectParent(l,shape,true)
                    sim.setLinkDummy(l,l2)
                    break
                end
                index=index+1
            end
        end
    end
end 
