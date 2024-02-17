function sysCall_init()
    simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObjectHandle('IRB4600_joint'..i)
    end
    simTip=sim.getObjectHandle('IRB4600_IkTip')
    simTarget=sim.getObjectHandle('IRB4600_IkTarget')
    modelBase=sim.getObjectHandle(sim.handle_self)
    
    ikEnv=simIK.createEnvironment()
    
    -- Prepare the ik group, using the convenience function 'simIK.addElementFromScene':
    ikGroup=simIK.createGroup(ikEnv)
    simIK.addElementFromScene(ikEnv,ikGroup,modelBase,simTip,simTarget,simIK.constraint_pose)

    -- IK movement data:
    ikMaxVel= {32,32,32,144}
    ikMaxAccel= {8.8,8.8,8.8,9.9} 
    ikMaxJerk= {6.6,6.6,6.6,8.8} 
    
    data={}
    data.ikEnv=ikEnv
    data.ikGroup=ikGroup
    data.tip=simTip
    data.target=simTarget
    data.joints=simJoints
    --(1)-------------------------------------------------------------------------
    --List with order of colors of the squares
    --1 yellow
    --2 green
    --3 orange
    --4 red
    A ={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,1,1,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1,1,4,1,4,4,4,4,4,4,4,1,4,4,4,4,1,4,1,1,4,4,4,1,1,1,1,1,1,1,1,1,4,4,4,4,1,1,4,4,4,4,4,4,4,4,4,4,1,4,4,4,4,4,1,1,4,4,1,4,1,1,4,4,4,4,4,1,4,4,4,4,1,1,4,1,1,4,1,1,4,4,4,4,1,4,4,4,4,4,1,1,4,1,1,1,1,1,1,1,1,1,1,4,4,4,4,4,1,1,4,1,4,1,1,1,1,1,1,1,1,1,4,4,1,4,1,1,4,4,1,4,1,1,1,1,1,1,1,1,1,1,1,4,1,1,4,4,4,4,4,1,1,1,1,1,1,1,4,4,4,4,1,1,4,4,4,4,4,1,1,1,1,1,1,4,4,4,4,4,1,1,4,4,4,4,4,4,1,1,1,1,1,4,4,4,1,4,1,1,4,4,1,4,4,1,1,1,1,1,1,1,1,1,1,4,1,1,4,1,4,1,4,1,1,1,1,1,1,1,4,4,4,4,1,1,4,1,4,1,4,1,1,1,1,1,1,1,4,4,1,4,1,1,4,1,4,1,4,1,1,1,1,1,1,1,1,1,1,4,1,1,4,4,4,4,1,4,1,1,1,4,1,1,1,4,4,4,1,1,4,1,4,4,4,4,4,4,4,4,4,4,4,4,1,4,1,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    cuboids = {}
    coming_cuboid = {}
    ---------------------------------------------------------------------------
    suxHdl = sim.getObjectHandle('suctionPad')
    pickup_script = sim.getScriptHandle('IRB4600_pickup')
    yellow_baskt = sim.getObjectHandle('Yellow')
    red_baskt = sim.getObjectHandle('Red')
    green_baskt = sim.getObjectHandle('Green')
    orange_baskt = sim.getObjectHandle('Orange')
    b_color = 0
    rotatorHdl = sim.getObjectHandle('IRB4600_joint6')
    first_time = true
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

function resetJoint(hdl) --resets the given motor joint position to zero, pass the motor handle
    local current_rot = sim.getJointPosition(hdl)/math.pi*180
    
    rotateAng(hdl,-current_rot)
    current_rot = sim.getJointPosition(hdl)/math.pi*180
    
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

function moveToPoseCallback(q,velocity,accel,auxData)
    sim.setObjectPose(auxData.target,-1,q)
    simIK.applyIkEnvironmentToScene(auxData.ikEnv,auxData.ikGroup)
end

function moveToPose_viaIK(maxVelocity,maxAcceleration,maxJerk,targetQ,auxData)
    local currentQ=sim.getObjectPose(auxData.tip,-1)
    return sim.moveToPose(-1,currentQ,maxVelocity,maxAcceleration,maxJerk,targetQ,moveToPoseCallback,auxData,nil)
end

function sPick(enable)-----Pick up funtion
    if enable then
        sim.writeCustomDataBlock(suxHdl,'activity','on')
    else
        sim.writeCustomDataBlock(suxHdl,'activity','off')
    end
end

function update_variables(color) 
    sim.callScriptFunction('update_variables@IRB4600_pickup',pickup_script,color)
end

function robot_pickup(get_pose,color)-----Box pick up
    local pose = get_pose
    pickup_gripper = sim.getObjectHandle('suctionPad#0')
    flag_gripper = sim.getObjectHandle('suctionPad')
    flag_g_loc = sim.getObjectPose(pickup_gripper,-1)
    changed = false
    pose[4]=0.0
    pose[5]=1.0
    pose[6]=0.0
    pose[7]=0.0
    if flag_g_loc[2] <-0.36 then --- checking if gripper is near the basket area
        pose[2] = pose[2] - 0.27
        pose[3] = pose[3]+0.2 
        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
        changed = true
   
    end
    
    pickup_g_loc = sim.getObjectPose(pickup_gripper,-1)
    flag_g_loc = sim.getObjectPose(flag_gripper,-1)
    g_dis = math.abs(pickup_g_loc[1]-flag_g_loc[1] )
    waited = 0
    
    while (pickup_g_loc[2]<0.33 and g_dis<=0.25 ) do -- waiting
        waited = 1
        pickup_g_loc = sim.getObjectPose(pickup_gripper,-1)
        flag_g_loc = sim.getObjectPose(flag_gripper,-1)
        g_dis = math.abs(pickup_g_loc[1]-flag_g_loc[1] )
        sim.wait(0.001)
    end
    if waited == 1 then -- if the robot has waited going back to check for a new box
        return waited
    end
    if changed == true then 
        pose[2] = pose[2] + 0.28
    end
    pose[3] = pose[3]+ 0.2
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    pose[3] = pose[3]- 0.2
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    sPick(true)
    sim.wait(0.1)
    pose[3] = pose[3]+0.2 
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    update_variables(color)
    return 0
end

function robot_drop(color,handle)
    local place =0
    for i=1,468 do
        if A[i]==color then
            place=i
            A[i]=0--Mark the found cood as filled
            break
        end
    end
    column=place//18
    row=place%18
    if row==0 then
        row=18
        column=column-1
    end
    -----------------------------------------------------------------------------------------
    pose = {}
    pose[1]=0.65-column*0.05--x cord to place
    pose[2]=-1.3+row*0.05--y cord to place
    pose[3]=0.2
    pose[4]=0.0
    pose[5]=1.0
    pose[6]=0.0
    pose[7]=0.0
        -------------------------------------------------------------------------------------
        
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    pose[3]=0.08--put it down
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    pose[3]=0.021
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    sPick(false)
    sim.wait(0.1)
    --sim.setObjectPose(handle,-1,pose)
    pose[3]=0.2--connector comes up straight
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end
function robot_reset_pose_2()---Reset pose 2
    local pose = {0,-3,1.6,0.0,1.0,0.0,0.0}
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end
function robot_reset_pose_3()---Reset pose 3
    local pose = {0,-0.5,0.2,0.0,1.0,0.0,0.0}
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end
function update_cuboids(dat)
    table.insert(cuboids,dat)
end
function getready(dat)
    table.insert(coming_cuboid,dat)
end
function coroutineMain()
    while true do

        if table.getn(cuboids)>=1 then -- checking for last dropped cuboid
            coming_cuboid = {}
            num = table.getn(cuboids)
            local handle = cuboids[num][1]
            local pose=sim.getObjectPose(handle,-1)
            b_color = cuboids[num][2]
            
            if(math.abs(sim.getJointPosition(rotatorHdl)/math.pi*180) > 180)then
                
                resetJoint(rotatorHdl)
            end
            
            sit = robot_pickup(pose,b_color)
            
            
            
            if sit == 0 then
                robot_drop(b_color,handle)
            
                table.remove(cuboids,num)
            end
        else
            if(math.abs(sim.getJointPosition(rotatorHdl)/math.pi*180) > 180)then
                
                resetJoint(rotatorHdl)
            end
            if table.getn(coming_cuboid)>=1 then -- getting ready 
                color = coming_cuboid[1]
                target_basket = red_baskt
                if color == 'yellow' then
                target_basket = yellow_baskt
                elseif color == 'orange' then
                target_basket = orange_baskt
                elseif color == 'green' then
                target_basket = green_baskt
                end
                local pose=sim.getObjectPose(target_basket,-1)
                pose[2] = -0.35
                pose[3] = 0.2
                pose[4]=0.0
                pose[5]=1.0
                pose[6]=0.0
                pose[7]=0.0
                moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
                first_time = false
                
            else
                if first_time == false then
                    robot_reset_pose_3()
                
                    
                    
                end
                
            end
        end
    end
end


