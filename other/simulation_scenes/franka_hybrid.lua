function __setObjectPosition__(a,b,c)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.setObjectPosition(a,b,c)
end
function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end
function __setObjectQuaternion__(a,b,c)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.setObjectQuaternion(a,b,c)
end
function __getObjectQuaternion__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectQuaternion(a,b)
end
function __setObjectOrientation__(a,b,c)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.setObjectOrientation(a,b,c)
end

function getTransformStamped(objHandle,name,relTo,relToName)
    -- This function retrieves the stamped transform for a specific object
    t=sim.getSystemTime()
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[2],y=o[3],z=o[4],w=o[1]}
        }
    }
end

function getDesiredHybrid(msg)
    -- Pose
    desired_x = msg.pose.position.x
    desired_y = msg.pose.position.y
    desired_z = msg.pose.position.z
    desired_qx = msg.pose.orientation.x
    desired_qy = msg.pose.orientation.y
    desired_qz = msg.pose.orientation.z
    desired_qw = msg.pose.orientation.w
    
    -- Wrench
    desired_fx = msg.wrench.force.x
    desired_fy = msg.wrench.force.y
    desired_fz = msg.wrench.force.z
    -- TODO: add torque
    
    -- Selection Vector and Constraint Frame
    selection[1] = msg.sel_vector[1] -- 1-indexed?
    selection[2] = msg.sel_vector[2]
    selection[3] = msg.sel_vector[3]
    -- TODO: add torque
    
    cqx = msg.constraint_frame.x
    cqy = msg.constraint_frame.y
    cqz = msg.constraint_frame.z
    cqw = msg.constraint_frame.w
end

function vectorIntoConstraintFrame(qx,qy,qz,qw,x,y,z)
    -- make sure it is normalized
    mag = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    qx = qx / mag
    qy = qy/ mag
    qz = qz / mag
    qw = qw / mag
    
    r11=1.0-2*qy*qy-2*qz*qz
    r12=2*qx*qy-2*qz*qw
    r13=2*qx*qz+2*qy*qw
    r21=2*qx*qy+2*qz*qw
    r22=1.0-2*qx*qx-2*qz*qz
    r23=2*qy*qz-2*qx*qw
    r31=2*qx*qz-2*qy*qw
    r32=2*qy*qz+2*qx*qw
    r33=1.0-2*qx*qx-2*qy*qy
    
    -- In this case, it is the transpose of the rotation
    x_rot = r11*x+r21*y+r31*z
    y_rot = r12*x+r22*y+r32*z
    z_rot = r13*x+r23*y+r33*z
    return x_rot, y_rot, z_rot
end

function vectorOutOfConstraintFrame(qx,qy,qz,qw,x,y,z)
    -- make sure it is normalized
    mag = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    qx = qx / mag
    qy = qy/ mag
    qz = qz / mag
    qw = qw / mag
    
    r11=1.0-2*qy*qy-2*qz*qz
    r12=2*qx*qy-2*qz*qw
    r13=2*qx*qz+2*qy*qw
    r21=2*qx*qy+2*qz*qw
    r22=1.0-2*qx*qx-2*qz*qz
    r23=2*qy*qz-2*qx*qw
    r31=2*qx*qz-2*qy*qw
    r32=2*qy*qz+2*qx*qw
    r33=1.0-2*qx*qx-2*qy*qy

    x_rot = r11*x+r12*y+r13*z
    y_rot = r21*x+r22*y+r23*z
    z_rot = r31*x+r32*y+r33*z
    return x_rot, y_rot, z_rot
end

function sysCall_init()
    -- Set joint angles to same homing as the real panda
    j1 = sim.getObjectHandle("Franka_joint1")
    sim.setJointPosition(j1,0.0754606)
    j2 = sim.getObjectHandle("Franka_joint2")
    sim.setJointPosition(j2,-0.337453)
    j3 = sim.getObjectHandle("Franka_joint3")
    sim.setJointPosition(j3,0.150729)
    j4 = sim.getObjectHandle("Franka_joint4")
    sim.setJointPosition(j4,-2.46194)
    j5 = sim.getObjectHandle("Franka_joint5")
    sim.setJointPosition(j5,0.0587094)
    j6 = sim.getObjectHandle("Franka_joint6")
    sim.setJointPosition(j6,2.12597)
    j7 = sim.getObjectHandle("Franka_joint7")
    sim.setJointPosition(j7,0.972193)
    
    
    -- Starting pose for end-effector
    desired_x = 0.4
    desired_y = 0.099
    desired_z = 0.25

    desired_qx = 0.0
    desired_qy = 0.0
    desired_qz = 0.0
    desired_qw = 1.0
    
    desired_fx = 0.0
    desired_fy = 0.0
    desired_fz = 0.0
    
    force_gain = 0.01
    
    -- Starting selection matrix
    selection = {1, 1, 1}
    
    -- Constraint frame rotation (quaternion)
    cqx = 0.0
    cqy = 0.0
    cqz = 0.0
    cqw = 1.0
    
    -- EE position dummy used for inverse kinematics
    targetDummy = sim.getObjectHandle("Target")
    baseLink = sim.getObjectHandle("Franka_joint1")
    tipDummy = sim.getObjectHandle("Tip")
    ftreader = sim.getObjectHandle("Franka_connection")
    ft_publisher=simROS.advertise('/panda/wrench','geometry_msgs/Wrench')
    
    -- The first time there is a non-zero force, it
    -- will be used as the bias
    bias_set = 0
    bias_x = 0
    bias_y = 0
    bias_z = 0
    
    rosInterfacePresent=simROS
    
    if rosInterfacePresent then
        subscriberHybrid = simROS.subscribe('/panda/hybrid_pose','panda_ros_msgs/HybridPose','getDesiredHybrid')
    end
    
end

function sysCall_actuation()
    -- ------------------------------------
    -- Send force information back to input
    ---------------------------------------
    result,f,t = sim.readForceSensor(ftreader)
    curr_vel = sim.getObjectVelocity(tipDummy)
    curr_pos = sim.getObjectPosition(targetDummy,-1)
    
    -- Bias measurement is taken once the panda has a sufficiently low velocity
    if f~=nil and bias_set==0 and math.abs(curr_vel[1])<0.0007 and math.abs(curr_vel[2])<0.0007 and math.abs(curr_vel[3])<0.0007 and sim.getSimulationTime()>0.5 then
        bias_x = f[1]
        bias_y = f[2]
        bias_z = f[3] -- weight of tool
        bias_set = 1
    end
    
    if f==nil then
        f = {0.0, 0.0, 0.0}
    end
    
    ft =
    {
        force=
        {
            x = (f[1]-bias_x),
            y = (f[2]-bias_y),
            z = (f[3]-bias_z)
        }
    }
    
    -- Coppeliasim gives applied vs reaction force.
    ft_reaction =
    {
        force=
        {
            x = -ft.force.x,
            y = -ft.force.y,
            z = -ft.force.z
        }
    }
    
    simROS.publish(ft_publisher,ft_reaction)
    
    
    --print("F:",math.sqrt(f[1]^2+f[2]^2+f[3]^2))
    
    -- --------------------------------------------------------------------
    -- calculate the desired inverse kinematics for the position controller
    -----------------------------------------------------------------------
    -- send actual ee position via TF2
    simROS.sendTransform(getTransformStamped(tipDummy,'panda_ee',baseLink,'panda_link0'))
    
    delta_T = 0.01 -- This runs at 100 Hz -> Update the position accordingly when in Hybrid mode
    
    -- rotate curr_pos and curr_force into the constraint frame
    curr_pos_x,curr_pos_y,curr_pos_z = vectorIntoConstraintFrame(cqx,cqy,cqz,cqw,curr_pos[1],curr_pos[2],curr_pos[3])
    rot_fx,rot_fy,rot_fz = vectorIntoConstraintFrame(cqx,cqy,cqz,cqw,ft.force.x,ft.force.y,ft.force.z)
    
    -- Compute the control law based on the selection matrix
    -- Deal with the Panda translation issues
    offset_panda_x,offset_panda_y,offset_panda_z = vectorIntoConstraintFrame(cqx,cqy,cqz,cqw,0.0413,0.0,0.746)
    -- Forces are already reaction forces with the virtual force torque sensor
    desired_x_total = selection[1]*(desired_x+offset_panda_x)+(1-selection[1])*(force_gain*delta_T*(desired_fx-rot_fx)+curr_pos_x)
    desired_y_total = selection[2]*(desired_y+offset_panda_y)+(1-selection[2])*(force_gain*delta_T*(desired_fy-rot_fy)+curr_pos_y)
    desired_z_total = selection[3]*(desired_z+offset_panda_z)+(1-selection[3])*(force_gain*delta_T*(desired_fz-rot_fz)+curr_pos_z)
    
    -- rotate back into the global robot frame
    desired_x_total,desired_y_total,desired_z_total = vectorOutOfConstraintFrame(cqx,cqy,cqz,cqw,desired_x_total,desired_y_total,desired_z_total)
    
    -- The orientation is local (in the constraint frame). Rotate out of the constraint frame using quaternion multiplication
    -- https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    -- q_global = q_CF * q_local
    final_qw = desired_qw*cqw-desired_qx*cqx-desired_qy*cqy-desired_qz*cqz
    final_qx = desired_qw*cqx+desired_qx*cqw-desired_qy*cqz+desired_qz*cqy
    final_qy = desired_qw*cqy+desired_qx*cqz+desired_qy*cqw-desired_qz*cqx
    final_qz = desired_qw*cqz-desired_qx*cqy+desired_qy*cqx+desired_qz*cqw
    
    sim.setObjectPosition(targetDummy,-1,{desired_x_total, desired_y_total, desired_z_total})
    sim.setObjectQuaternion(targetDummy,-1,{final_qx, final_qy, final_qz, final_qw})
    
end
