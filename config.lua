activate = "activate"

function sysCall_init()

modelBase=sim.getObjectHandle(sim.handle_self)
modelName=sim.getObjectName(modelBase)
-- Retrive the handle of all joints
joint_1_s_handle=sim.getObjectHandle('joint_1_s')
joint_2_l_handle=sim.getObjectHandle('joint_2_l')
joint_3_u_handle=sim.getObjectHandle('joint_3_u')
-- joint_4_r_handle=sim.getObjectHandle('joint_4_r')
-- joint_5_b_handle=sim.getObjectHandle('joint_5_b')
-- joint_6_t_handle=sim.getObjectHandle('joint_6_t')

-- Retrieve the handle of Vision Sensor
visionSensorHandle=sim.getObjectHandle('Vision_sensor')

worldVisual=sim.getObjectHandle('/world_visual')
worldVisualScript=sim.getScriptAssociatedWithObject(worldVisual)

suctionPad=sim.getObjectHandle('suctionPad#0')
suctionPadScript=sim.getScriptAssociatedWithObject(suctionPad)

if simROS2 then

    -- Enable the publishing of Vision Sensor Data
    pub=simROS2.createPublisher('/camera/image_raw', 'sensor_msgs/msg/Image')
    simROS2.publisherTreatUInt8ArrayAsString(pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)

    -- Enable joint publishing
    j1_state_pub = simROS2.createPublisher('/csim_simulation/robot_sim_env/joint_1_s/state', 'std_msgs/msg/Float32')
    j2_state_pub = simROS2.createPublisher('/csim_simulation/robot_sim_env/joint_2_l/state', 'std_msgs/msg/Float32')
    j3_state_pub = simROS2.createPublisher('/csim_simulation/robot_sim_env/joint_3_u/state', 'std_msgs/msg/Float32')
    -- j4_state_pub = simROS.advertise('/csim_simulation/robot_sim_env/joint_4_r/state', 'std_msgs/Float32')
    -- j5_state_pub = simROS.advertise('/csim_simulation/robot_sim_env/joint_5_b/state', 'std_msgs/Float32')
    -- j6_state_pub = simROS.advertise('/csim_simulation/robot_sim_env/joint_6_t/state', 'std_msgs/Float32')


    --enable joint subscriber
    -- j1_cmd_sub = simROS.subscribe('/csim_simulation/robot_sim_env/joint_1_s/cmd', 'std_msgs/Float32', 'j1Cmd_callback')
    -- j2_cmd_sub = simROS.subscribe('/csim_simulation/robot_sim_env/joint_2_l/cmd', 'std_msgs/Float32', 'j2Cmd_callback')
    -- j3_cmd_sub = simROS.subscribe('/csim_simulation/robot_sim_env/joint_3_u/cmd', 'std_msgs/Float32', 'j3Cmd_callback')
    -- j4_cmd_sub = simROS.subscribe('/csim_simulation/robot_sim_env/joint_4_r/cmd', 'std_msgs/Float32', 'j4Cmd_callback')
    -- j5_cmd_sub = simROS.subscribe('/csim_simulation/robot_sim_env/joint_5_b/cmd', 'std_msgs/Float32', 'j5Cmd_callback')
    -- j6_cmd_sub = simROS.subscribe('/csim_simulation/robot_sim_env/joint_6_t/cmd', 'std_msgs/Float32', 'j6Cmd_callback')

    active = simROS2.createSubscription('/csim_simulation/active', 'std_msgs/msg/String', 'activate_cb')
    joint_angles_sub = simROS2.createSubscription('/joints_to_aurdino', 'custom_message/msg/Floats', 'joint_angles_cb' )
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
        end
        end

        function activate_cb(activate_msg)
        activate = activate_msg['data']
        end

        activateSuctionPad=function(active)
        -- print (activate)
        if (activate == 'activate') then
            sim.setScriptSimulationParameter(suctionPadScript,'active','true')
            else
                sim.setScriptSimulationParameter(suctionPadScript,'active','false')
                sim.setScriptSimulationParameter(worldVisualScript, 'active','false')
                end
                end


                function joint_angles_cb(msg)
                joint_angles = msg['data']
                j1_position = (joint_angles[1] * 3.14 /180) -1.57
                j2_position = (joint_angles[2] * 3.14 /180) -1.57
                j3_position = (joint_angles[3] * 3.14 /180) -1.57

                -- print(j1_position)
                -- print (j2_position)
                -- print (j3_position)
                sim.setJointTargetVelocity(joint_1_s_handle, 15)
                sim.setJointTargetVelocity(joint_2_l_handle, 15)
                sim.setJointTargetVelocity(joint_3_u_handle, 15)
                sim.setJointTargetPosition( joint_1_s_handle, -j1_position)
                sim.setJointTargetPosition( joint_2_l_handle, j2_position)
                sim.setJointTargetPosition( joint_3_u_handle, j3_position)


                j1_pos = sim.getJointPosition ( joint_1_s_handle )
                d1={}
                d1['data'] = j1_pos
                simROS2.publish( j1_state_pub, d1 )

                j2_pos = sim.getJointPosition ( joint_2_l_handle )
                d2={}
                d2['data'] = j2_pos
                simROS2.publish( j2_state_pub, d2 )

                j3_pos = sim.getJointPosition ( joint_3_u_handle )
                d3={}
                d3['data'] = j3_pos
                simROS2.publish( j3_state_pub, d3 )

                end

                -- function j1Cmd_callback( msg )
                -- sm.setJointTargetPosition( joint_1_s_handle, msg['data'] )
                -- end

                -- function j2Cmd_callback( msg )
                -- sim.setJointTargetPosition( joint_2_l_handle, msg['data'] )
                -- end

                -- function j3Cmd_callback( msg )
                -- sim.setJointTargetPosition( joint_3_u_handle, msg['data'] )
                -- end

                -- function j4Cmd_callback( msg )
                -- sim.setJointTargetPosition( joint_4_r_handle, msg['data'] )
                -- end

                -- function j5Cmd_callback( msg )
                -- sim.setJointTargetPosition( joint_5_b_handle, msg['data'] )
                -- end

                -- function j6Cmd_callback( msg )
                -- sim.setJointTargetPosition( joint_6_t_handle, msg['data'] )
                -- end



                function sysCall_actuation()
                -- local jpos = sim.getJointPosition ( joint_1_s_handle )
                -- d={}
                -- d['data'] = jpos
                -- simROS.publish( j1_state_pub, d )

                -- jpos = sim.getJointPosition ( joint_2_l_handle )
                -- d={}
                -- d['data'] = jpos
                -- simROS.publish( j2_state_pub, d )

                -- jpos = sim.getJointPosition ( joint_3_u_handle )
                -- d={}
                -- d['data'] = jpos
                -- simROS.publish( j3_state_pub, d )

                -- jpos = sim.getJointPosition ( joint_4_r_handle )
                -- d={}
                -- d['data'] = jpos
                -- simROS.publish( j4_state_pub, d )

                -- jpos = sim.getJointPosition ( joint_5_b_handle )
                -- d={}
                -- d['data'] = jpos
                -- simROS.publish( j5_state_pub, d )

                -- jpos = sim.getJointPosition ( joint_6_t_handle )
                -- d={}
                -- d['data'] = jpos
                -- simROS.publish( j6_state_pub, d )

                if (active == false) then

                    activateSuctionPad(true)
                    else

                        activateSuctionPad(false)
                        end



                        end


                        function sysCall_sensing()
                        if simROS2 then
                            -- Publish the image of the active vision sensor:
                            local data,resolution=sim.getVisionSensorImg(visionSensorHandle)
                            d={}
                            d.header={stamp=simROS2.getTime(), frame_id='a'}
                            d.height=resolution[2]
                            d.width=resolution[1]
                            d.encoding='rgb8'
                            d.is_bigendian=1
                            d.step=resolution[1]*3
                            d.data=data
                            simROS2.publish(pub,d)
                            end
                            end



