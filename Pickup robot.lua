function sysCall_init()
    simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObjectHandle('IRB4600_joint'..i..'#0')
    end
    simTip=sim.getObjectHandle('IRB4600_IkTip#0')
    simTarget=sim.getObjectHandle('IRB4600_IkTarget#0')
    modelBase=sim.getObjectHandle(sim.handle_self)
    
    ikEnv=simIK.createEnvironment()

    -- Prepare the ik group, using the convenience function 'simIK.addElementFromScene':
    ikGroup=simIK.createGroup(ikEnv)
    simIK.addElementFromScene(ikEnv,ikGroup,modelBase,simTip,simTarget,simIK.constraint_pose)

    -- IK movement data:
    
    ikMaxVel={32,32,32,144}
    ikMaxAccel={8,8,8,9}  
    ikMaxJerk= {6,6,6,8}
    ------------------------------
    
    data={}
    data.ikEnv=ikEnv
    data.ikGroup=ikGroup
    data.tip=simTip
    data.target=simTarget
    data.joints=simJoints
    
    -- Put your main loop here, e.g.:
    basketPos = {1.1,0.9,0.09}
    
    ------------------------------Basket handles 
    yellow_baskt = sim.getObjectHandle('Yellow')
    red_baskt = sim.getObjectHandle('Red')
    green_baskt = sim.getObjectHandle('Green')
    orange_baskt = sim.getObjectHandle('Orange')
    -------------------------------------------
    Cam = sim.getObjectHandle('Vision_sensor#0')
    Cam_script = sim.getScriptHandle('Vision_sensor#0')
    flag_script = sim.getScriptHandle('IRB4600_Flag')
    suxHdl = sim.getObjectHandle('suctionPad#0')
    bskIndex_g = 0
    bskIndex_r = 0
    bskIndex_y = 0
    bskIndex_o = 0
    start = 42
    number = 467
    picked_up_objects = {}
    b_color = 0
    
    --for turning
    top_axis = 3
    
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function sysCall_cleanup()
    -- do some clean-up here
end

function alignCube(rotHdl,cubeHdl) --corrects the cube alignment, pass the motor and cube handles
    local pre_ori = sim.getObjectOrientation(cubeHdl,-1)
    rotateAng(rotHdl,5)
    local now_ori = sim.getObjectOrientation(cubeHdl,-1)
    local change = now_ori[top_axis] - pre_ori[top_axis]
    
    local tar = (now_ori[top_axis]/math.pi*180)
    if(change < 0)then
        tar = 90 - tar
    end
    
    rotateAng(rotHdl,-tar)
    

    --[[
    local targetAng = sim.getJointPosition(rotHdl) + 95/180*math.pi
    sim.setJointTargetPosition(rotHdl,targetAng)
    local aligned = false
    
    while(not aligned) do
        ori = sim.getObjectOrientation(cubeHdl,-1)
        print('ori top',ori[top_axis],math.floor(ori[top_axis]/math.pi*180*100))
        if(math.floor(ori[top_axis]/math.pi*180*100) == 0 or (math.floor(ori[top_axis]/math.pi*180*100)%90 == 0)) then
            aligned = true
        end
    end
    --]]
    
end

function getTopAxis(cuboid_handle) --returns the top axis of the cube, pass the hand;e of the current cube
    local rotHandle=sim.getObjectHandle('IRB4600_joint6#0')
    local topAxis = 3

    local prev_ori = sim.getObjectOrientation(cuboid_handle,-1)
    rotateAng(rotHandle,5)
    local new_ori = sim.getObjectOrientation(cuboid_handle,-1)
    --rotateAng(rotHandle,-3)
    
    ori_difference = {}
    for i = 1,3,1 do
        ori_difference[i] = new_ori[i] - prev_ori[i]
        ori_difference[i] = math.abs(math.floor(ori_difference[i]/math.pi*180))
        if(ori_difference[i] > 357) then
            ori_difference[i] = 0
        end
    end
    local maxDiff = math.max(unpack(ori_difference))
    if(ori_difference[1] == maxDiff) then
        topAxis = 1
    elseif(ori_difference[2] == maxDiff) then
        topAxis = 2
    end
    
    return topAxis
end

function resetJoint(hdl) --resets the given motor joint position to zero, pass the motor handle
    local current_rot = sim.getJointPosition(hdl)/math.pi*180
   
    rotateAng(hdl,-current_rot)
    current_rot = sim.getJointPosition(hdl)/math.pi*180
    
end

function setMotorRot(hdl,rot)
    rot = rot*math.pi/180
    local targetRot = -rot*sim.getJointPosition(hdl)/math.abs(sim.getJointPosition(hdl))
    sim.setJointTargetPosition(hdl,targetRot)
    while(math.floor(sim.getJointPosition(hdl)*1000) ~= math.floor(targetRot*1000))do
        math.floor(sim.getJointPosition(hdl)*1000)
        math.floor(targetRot*1000)
    end
end

function rotateAng(hdl,angle) --rotates the given motor by the given angle(in degrees)
    angle = angle/180*math.pi
    local targetAng = sim.getJointPosition(hdl) + angle
    sim.setJointTargetPosition(hdl,targetAng)
    while(math.floor(sim.getJointPosition(hdl)*1000) ~= math.floor(targetAng*1000)) do
        math.floor(sim.getJointPosition(hdl)*1000)
        math.floor(targetAng*1000)
    end
end

function sPick(enable)-----Pick up funtion
    if enable then
        sim.writeCustomDataBlock(suxHdl,'activity','on')
    else
        sim.writeCustomDataBlock(suxHdl,'activity','off')
    end
end

function update_variables(color) 
    if color == 1 then
        bskIndex_y = bskIndex_y -1
    elseif color == 2 then
        bskIndex_g = bskIndex_g -1
    elseif color == 3 then
        bskIndex_o = bskIndex_o -1
    else
        bskIndex_r = bskIndex_r -1
    end
end
function moveToPoseCallback(q,velocity,accel,auxData)
    sim.setObjectPose(auxData.target,-1,q)
    simIK.applyIkEnvironmentToScene(auxData.ikEnv,auxData.ikGroup)
end

function moveToPose_viaIK(maxVelocity,maxAcceleration,maxJerk,targetQ,auxData)
    local currentQ=sim.getObjectPose(auxData.tip,-1)
    return sim.moveToPose(-1,currentQ,maxVelocity,maxAcceleration,maxJerk,targetQ,moveToPoseCallback,auxData,nil)
end

function robot_reset_pose_1()---Reset pose 1
    local pose = {0,1,2.50,0.0,1.0,0.0,0.0}
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end

function robot_reset_pose_2()---Reset pose 2
    local pose = {0,3,1.6,0.0,1.0,0.0,0.0}
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end

function robot_reset_pose_3()---Reset pose 2
    local pose = {0,1,0.6,0.0,1.0,0.0,0.0}
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end

function robot_pickup(get_pose)-----Box pick up
    local pose = get_pose
    pose[3] = pose[3]+0.06
    pose[4]=0.0
    pose[5]=1.0
    pose[6]=0.0
    pose[7]=0.0
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    
    --resetting the joint to zero
    --rotHandle=sim.getObjectHandle('IRB4600_joint6#0')
    --print('joint resetting')
    --resetJoint(rotHandle)
    
    pose[3] = pose[3]- 0.06
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    sim.wait(0.02)
    dat = sim.callScriptFunction('sysCall_sensing@Vision_sensor#0',Cam_script,1)------Getting color when contacted with cube 
    sPick(true)
    
    sim.wait(0.02)
    sim.callScriptFunction('getready@IRB4600_Flag',flag_script,dat)
    pose[3] = pose[3]+ 0.1
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    
    return dat ----Returing color 
end

function dropToBasket(color,next_cuboid_handle)
    local bskHdl=sim.getObjectHandle('Green')
    local bskIndex=bskIndex_g
    flag_gripper = sim.getObjectHandle('suctionPad')
    pickup_gripper = sim.getObjectHandle('suctionPad#0')
    send_color = 2
    if(color == 'orange') then
        bskHdl=sim.getObjectHandle('Orange')
        bskIndex=bskIndex_o
        send_color = 3
    elseif(color == 'red') then
        bskHdl=sim.getObjectHandle('Red')
        bskIndex=bskIndex_r
        send_color = 4
    elseif(color == 'yellow') then
        bskHdl=sim.getObjectHandle('Yellow')
        bskIndex=bskIndex_y
        send_color = 1
    end
    --setting to safe spot above basket
    local pose=sim.getObjectPose(bskHdl,-1)
    pose = {pose[1],pose[2],pose[3]+0.3,0,1,0,0}
    pose[2] = pose[2]+0.27 -- going close to baskets
    
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    pickup_g_loc = sim.getObjectPose(pickup_gripper,-1)
    flag_g_loc = sim.getObjectPose(flag_gripper,-1)
    g_dis = math.abs(pickup_g_loc[1]-flag_g_loc[1] ) -- waiting
    while (flag_g_loc[2]>-0.33 and g_dis<=0.25) do
        pickup_g_loc = sim.getObjectPose(pickup_gripper,-1)
        flag_g_loc = sim.getObjectPose(flag_gripper,-1)
        g_dis = math.abs(pickup_g_loc[1]-flag_g_loc[1] )
        sim.wait(0.001)
    end
    
    pose[2] = pose[2]-0.27
    local x_rel_pos = (bskIndex - 2)*0.05
    local y_rel_pos = (bskIndex//5 - 2)*0.05
    local dropPos = {pose[1]+x_rel_pos,pose[2]+y_rel_pos,0.04,0,1,0,0}
    
    dropPos[3] = dropPos[3]+0.1
    
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,dropPos,data)
    
    dropPos[3] = 0.05
    
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,dropPos,data)
    
    --correct cube alignment
    rotHandle=sim.getObjectHandle('IRB4600_joint6#0')
    alignCube(rotHandle,next_cuboid_handle)
    
    
    sPick(false)
    sim.wait(0.02)
    
    --incrementing basket index
    if(color == 'orange') then
        bskIndex_o = bskIndex_o + 1
        
    elseif(color == 'red') then
        bskIndex_r = bskIndex_r + 1
    elseif(color == 'yellow') then
        bskIndex_y = bskIndex_y + 1
    else
        bskIndex_g = bskIndex_g + 1
    end
    dropPos[3] = 0.1
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,dropPos,data)
    
    sim.callScriptFunction('update_cuboids@IRB4600_Flag',flag_script,{next_cuboid_handle,send_color})

end

local function has_value (tab, val)----Function to check if a element exists inside a given array
    for index, value in ipairs(tab) do
        if value == val then
            return true
        end
    end

    return false
end

function get_next_object()
    base_pose = {0.0,2.0}
    rotHandle=sim.getObjectHandle('IRB4600_joint6#0')
    my_pose = sim.getObjectPosition(rotHandle,-1)
    cuboids = {}---- saving all cuboids yet to be picked up
    cuboid_handle = 0
    cuboid_handle_2 = 0
    min_dis = 0
    min_dis_2 = 0
    final = (start+number)
    local out_box_count = 0
    for i = start,final,1 
    do 
        handler = sim.getObjects(i,sim.object_shape_type)
        if handler == -1 then
            return -1
        end
        if not has_value(picked_up_objects, handler) then
            position = sim.getObjectPosition(handler,-1)
            dis =  ((position[1]- my_pose[1])^2 + (position[2]- my_pose[2])^2 )^0.5
            dis_base =  ((position[1]- base_pose[1])^2 + (position[2]- base_pose[2])^2 )^0.5
            if dis_base>=0.7 and dis_base<=2.3 then
                if min_dis==0 or dis<=min_dis then
                    cuboid_handle = handler
                    min_dis=dis
                end
                if min_dis_2==0 or dis_base<=min_dis_2 then
                    cuboid_handle_2 = handler
                    min_dis_2=dis_base
                end
            else
                out_box_count = out_box_count + 1
                print("Out of range boxes ",position[1],position[2])
            end
        end
    end
    
    if cuboid_handle== 0 then
        if cuboid_handle_2 == 0 then
            return -1
        else
            return cuboid_handle_2
        end
    end
    return cuboid_handle------Returning selected cuboid handle
end

function coroutineMain()
    while true do
        
        next_cuboid_handle = get_next_object() --- next cuboid to pick
        table.insert(picked_up_objects,next_cuboid_handle) 
        if  next_cuboid_handle == -1 then --- Checking if all cuboids have been placed
            robot_reset_pose_3()
            break
        end
        local pose=sim.getObjectPose(next_cuboid_handle,-1)----Gettinf current cuboid location
        
        b_color = robot_pickup(pose)
        top_axis = getTopAxis(next_cuboid_handle)
        
        dropToBasket(b_color,next_cuboid_handle)

       
       
    end
    print('pick up robot done')
end

-- See the user manual or the available code snippets for additional callback functions and details


