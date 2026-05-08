//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2024, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <hishida3@jhu.edu>
    \author    Hisashi Ishida
    \date      09.18.2024
    
*/
//==============================================================================

#include "tf_plugin.h"

using namespace std;

void convertFloatTobtMatrix(double rotation[3][3], btMatrix3x3 btRotationMatrix){
    // Convert the rotation matrix to a btMatrix3x3
    btRotationMatrix.setValue(
        rotation[0][0], rotation[0][1], rotation[0][2],
        rotation[1][0], rotation[1][1], rotation[1][2],
        rotation[2][0], rotation[2][1], rotation[2][2]
    );
}

Transforms::Transforms(){   
}

void Transforms::convertPoseStampedMsgTocTransform(chai3d::cTransform &trans, AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg){
    trans.setLocalPos(cVector3d(msg->pose.position.x,
                                            msg->pose.position.y,
                                            msg->pose.position.z));
    cQuaternion rot(msg->pose.orientation.w,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z);
    cMatrix3d rotM;
    rot.toRotMat(rotM);
    trans.setLocalRot(rotM);
}

void Transforms::transformCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg){
    if (msg->header.stamp.sec ==0) {
        cerr << "Received PoseStamped with invalid (zero) timestamp" << endl;
        isMsgValid_ = false;
        return;
    }
    isMsgValid_ = true;

    if (!initialized_) {
        convertPoseStampedMsgTocTransform(filteredTransform_, msg);
        initialized_ = true;
        return;
    }

    if (isFiltered_){
        chai3d::cTransform currentTransform;
        convertPoseStampedMsgTocTransform(currentTransform, msg);

        // Blend positions
        chai3d::cVector3d blended_pos = alpha_ * currentTransform.getLocalPos() + (1 - alpha_) * filteredTransform_.getLocalPos();

        // Blend orientations using SLERP
        chai3d::cQuaternion q_prev, q_curr;
        q_prev.fromRotMat(filteredTransform_.getLocalRot());
        q_curr.fromRotMat(currentTransform.getLocalRot());
        chai3d::cQuaternion q_blend;
        q_blend.slerp(1.0-alpha_, q_prev, q_curr);

        // Final filtered transform
        chai3d::cMatrix3d blended_rot;
        q_blend.toRotMat(blended_rot);
        filteredTransform_.setLocalPos(blended_pos);
        filteredTransform_.setLocalRot(blended_rot);

        transformation_ = filteredTransform_;
    }

    else{
        convertPoseStampedMsgTocTransform(transformation_, msg);
    }
}

void Transforms::referenceTransformCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg){
    if (msg->header.stamp.sec ==0) {
        cerr << "Received PoseStamped with invalid (zero) timestamp" << endl;
        isMsgValid_ = false;
        return;
    }
    isMsgValid_ = true;

    if (!initialized_) {
        convertPoseStampedMsgTocTransform(filteredReferenceTransform_, msg);
        initialized_ = true;
        return;
    }

    if (isFiltered_){
        chai3d::cTransform currentReferenceTransform;
        convertPoseStampedMsgTocTransform(currentReferenceTransform, msg);

        // Blend positions
        chai3d::cVector3d blended_pos = alpha_ * currentReferenceTransform.getLocalPos() + (1 - alpha_) * filteredReferenceTransform_.getLocalPos();

        // Blend orientations using SLERP
        chai3d::cQuaternion q_prev, q_curr;
        q_prev.fromRotMat(filteredReferenceTransform_.getLocalRot());
        q_curr.fromRotMat(currentReferenceTransform.getLocalRot());
        chai3d::cQuaternion q_blend;
        q_blend.slerp(1.0 - alpha_, q_prev, q_curr);

        // Final filtered transform
        chai3d::cMatrix3d blended_rot;
        q_blend.toRotMat(blended_rot);
        filteredReferenceTransform_.setLocalPos(blended_pos);
        filteredReferenceTransform_.setLocalRot(blended_rot);

        reference_trans_ = filteredReferenceTransform_;
    }

    else{
        convertPoseStampedMsgTocTransform(reference_trans_, msg);
    }
}

afTFPlugin::afTFPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF TF Plugin" << endl;
    cout << "/*********************************************" << endl;
}

int afTFPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
    p_opt::options_description cmd_opts("AMBF_TF_Plugin Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("mute", p_opt::value<bool>()->default_value(true), "Mute")
            ("tf_list", p_opt::value<string>()->default_value(""), "Name of tf_list yaml file");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    // Loading options 
    m_tf_list_path = var_map["tf_list"].as<string>();
    bool mute = var_map["mute"].as<bool>();

    // Define path
    string file_path = __FILE__;
    m_current_filepath = file_path.substr(0, file_path.rfind("/"));

    // Get pointer to World
    m_worldPtr = a_afWorld;

    // Improve the constratint
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; // improve out of plane error of joints

    // Load audio
    string audioFilepath = m_current_filepath + "/../example/sounds/tone_440hz.wav";
    if (!mute){
        m_audioDevice = new cAudioDevice();
        m_mainCamera = m_worldPtr->getCamera("main_camera");
        m_mainCamera->getInternalCamera()->attachAudioDevice(m_audioDevice);
        
        m_audioBuffer = new cAudioBuffer();
        if (m_audioBuffer->loadFromFile(audioFilepath)){
            m_audioSource = new cAudioSource();
            m_audioSource->setAudioBuffer(m_audioBuffer);
            m_audioSource->setLoop(true);
            m_audioState = AudioState::STOPPED;
        }
        else{
            delete m_audioSource;
            delete m_audioBuffer;
            m_audioSource = nullptr;
            m_audioBuffer = nullptr;
            cerr << "FAILED TO LOAD Beep AUDIO FROM " << audioFilepath << endl;
        }
    }
    
    // When config file was defined
    if(!m_tf_list_path.empty()){
        int result = readTFListYaml(m_tf_list_path);

        // Check for the stored transformation list and if the type is INITIAL, move the corresponding object
        for (size_t i = 0; i < m_transformList.size(); i++){
            if (m_transformList[i]->transformType_ == TransformationType::INITIAL){
                btTransform transform = to_btTransform(m_transformList[i]->transformation_);
                moveRigidBody(m_transformList[i], transform, 0.001);
            }
        }
        return result;
    }

    // No config file specified
    else{
        cerr << "[ERROR] No TF list was specified!!" << endl;
        return -1;
    }
}

void afTFPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){ 
    // Reload a configuration file when 'ALT + R' key are pressed
    if (a_action == GLFW_MOD_ALT){
        if (a_key == GLFW_KEY_R){
            cerr << "> Reloading the user defined tf list..." << endl;
            readTFListYaml(m_tf_list_path);
        }
    }
}

void afTFPlugin::graphicsUpdate(){
}
// Function to print a btTransform (translation and rotation)
void printBtTransform(const btTransform &transform) {
    // Extract the translation (origin)
    btVector3 origin = transform.getOrigin();
    std::cout << "Translation: ["
              << origin.getX() << ", "
              << origin.getY() << ", "
              << origin.getZ() << "]" << std::endl;

    // Extract the rotation (quaternion)
    btQuaternion rotation = transform.getRotation();
    std::cout << "Rotation (quaternion): ["
              << rotation.getX() << ", "
              << rotation.getY() << ", "
              << rotation.getZ() << ", "
              << rotation.getW() << "]" << std::endl;

    // Optional: Convert quaternion to Euler angles if preferred
    btScalar roll, pitch, yaw;
    transform.getBasis().getEulerZYX(yaw, pitch, roll); // Bullet uses ZYX convention for Euler angles
    std::cout << "Rotation (Euler angles): ["
              << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
}

void afTFPlugin::moveRigidBody(const Transforms* transformINFO, const btTransform transform, double dt){
    btTransform command;
    if (transformINFO->parentRB_){
        btTransform parentTransform;
        transformINFO->parentRB_->m_bulletRigidBody->getMotionState()->getWorldTransform(parentTransform);
        command = parentTransform * transform;
    }
    else{
        command = transform;
    }

    // If the rigidbody is static
    if (transformINFO->childRB_->m_bulletRigidBody->isStaticOrKinematicObject()){
        // Apply transformation to the child body
        transformINFO->childRB_->m_bulletRigidBody->getMotionState()->setWorldTransform(command);
        transformINFO->childRB_->m_bulletRigidBody->setWorldTransform(command);
    }

    // If the rigid body is non-static
    else{
        // Get current location     
        btTransform curr_trans = transformINFO->childRB_->getCOMTransform();

        btVector3 pCommand, rCommand;
        // Use the internal Cartesian Position Controller to Compute Output
        pCommand = transformINFO->childRB_->m_controller.computeOutput<btVector3>(curr_trans.getOrigin(), command.getOrigin(), dt);
        // Use the internal Cartesian Rotation Controller to Compute Output
        rCommand = transformINFO->childRB_->m_controller.computeOutput<btVector3>(curr_trans.getBasis(), command.getBasis(), dt);
        
        // Set controller param here if needed
        if (transformINFO->childRB_->m_controller.m_positionOutputType == afControlType::FORCE){
            transformINFO->childRB_->m_bulletRigidBody->applyCentralForce(pCommand);
            transformINFO->childRB_->m_bulletRigidBody->applyTorque(rCommand);
        }

        else if (transformINFO->childRB_->m_controller.m_positionOutputType == afControlType::VELOCITY){
            transformINFO->childRB_->m_bulletRigidBody->setLinearVelocity(pCommand);
            transformINFO->childRB_->m_bulletRigidBody->setAngularVelocity(rCommand);
        }
    }
}


void afTFPlugin::physicsUpdate(double dt){   
    for (size_t i = 0; i < m_transformList.size(); i++){
        if (m_transformList[i]->transformType_ == TransformationType::FIXED ||
        m_transformList[i]->transformType_ == TransformationType::ROS){
            if (m_transformList[i]->rosNode_){
                ambf_ral::spin_some(m_transformList[i]->rosNode_);
            }
            chai3d::cTransform ref_inv;
            if(m_transformList[i]->isReference_){
                ref_inv = m_transformList[i]->reference_trans_;
                ref_inv.invert();
            }
            chai3d::cTransform ctrans = ref_inv * m_transformList[i]->transformation_;
            btTransform transform = to_btTransform(ctrans);

            moveRigidBody(m_transformList[i], transform, dt);
        }

        if (!m_transformList[i]->isMsgValid_  && m_audioSource){
            if (m_audioState == AudioState::STOPPED){
                m_audioSource->play();
                m_audioState = AudioState::PLAYING;
            }
        }
        else{
            if (m_audioState == AudioState::PLAYING){
                m_audioSource->stop();
                m_audioState = AudioState::STOPPED;
            }
        }
    }
}

int afTFPlugin::readTFListYaml(string file_path){
    cerr << "> Loading the user defined tf list..." << endl; 
    cerr << file_path << endl;

    //Load the user defined object here. 
    YAML::Node node = YAML::LoadFile(file_path);
    
    if (node["transformations"]){
        for (size_t i = 0; i < node["transformations"].size(); i++){
            Transforms* transformINFO = new Transforms();
            string transformName = node["transformations"][i].as<string>();
            
            if (node[transformName]){
                // Store transformation name
                transformINFO->name_ = transformName;
                
                // Store transformation type
                if (node[transformName]["type"].as<string>() == "FIXED")
                    transformINFO->transformType_ = TransformationType::FIXED;
                if (node[transformName]["type"].as<string>() =="INITIAL")
                    transformINFO->transformType_ = TransformationType::INITIAL;
                if (node[transformName]["type"].as<string>() == "ROS")
                    transformINFO->transformType_ = TransformationType::ROS;

                // Store parent information
                // If the parent is "World" then keep the parentRB_ as nullptr
                if (node[transformName]["parent"].as<string>() != "World"){
                    transformINFO->parentRB_ = m_worldPtr->getRigidBody(node[transformName]["parent"].as<string>());
                    if (!transformINFO->parentRB_){
                        cerr << "[ERROR!!] Parent rigid body " << node[transformName]["parent"].as<string>() << " not found in the world!" << endl;
                        delete transformINFO;
                        return -1;
                    }
                }
                
                // Store child information
                transformINFO->childRB_ = m_worldPtr->getRigidBody(node[transformName]["child"].as<string>());
                if (!transformINFO->childRB_){
                    cerr << "[ERROR!!] Child rigid body " << node[transformName]["child"].as<string>() << " not found in the world!" << endl;
                    delete transformINFO;
                    return -1;
                }

                // Store transformation 
                readTransformationFromYaml(transformINFO, node);
                
                // Push the pointer into the list
                m_transformList.push_back(transformINFO);
            }
            else{
                return -1;
            }
        }
        return 1;
    }
    else {
        return -1;
    }
}

void afTFPlugin::readTransformationFromYaml(Transforms* transformINFO, YAML::Node& node){
    if (transformINFO->transformType_ == TransformationType::FIXED || transformINFO->transformType_ == TransformationType::INITIAL){
        
        // Transformation written in position: {x: 0.0, y:0.0, z:0.0}, orientation: {r: 0.0, p: 0.0, y:0.0}
        if (node[transformINFO->name_]["transformation"]["position"] && node[transformINFO->name_]["transformation"]["orientation"]){
            
            YAML::Node transformPos = node[transformINFO->name_]["transformation"]["position"];
            YAML::Node transformOri = node[transformINFO->name_]["transformation"]["orientation"];

            cVector3d trans = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&transformPos));
            cMatrix3d rot = to_cMatrix3d(adf_loader_1_0::ADFUtils::rotationFromNode(&transformOri));

            transformINFO->transformation_.setLocalPos(trans);
            transformINFO->transformation_.setLocalRot(rot); 
        }

        // Transformation written in 4x4 matrix
        else{
            vector<vector<double>> mat = node[transformINFO->name_]["transformation"].as<vector<vector<double>>>();
            double rotMat[3][3];
            // Retrieve the matrix from YAML and copy it to the array
            for (size_t i = 0; i < 3; ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    rotMat[i][j] = mat[i][j];
                }
            }
            transformINFO->transformation_.setLocalPos(cVector3d(mat[0][3], mat[1][3], mat[2][3]));
            
            cMatrix3d rot;
            rot.set(rotMat);
            transformINFO->transformation_.setLocalRot(rot); 
        }
    }

    else if (transformINFO->transformType_ == TransformationType::ROS){
        // Set up the subscriber
        transformINFO->rosNode_ = afROSNode::getNodeAndRegister("AMBF_TF_Plugin_Node");
        string topicName = node[transformINFO->name_]["rostopic name"].as<string>();
        ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, PoseStamped), Transforms>
            (transformINFO->transformSub_, transformINFO->rosNode_, topicName, 1, &Transforms::transformCallback, transformINFO);

        if (node[transformINFO->name_]["reference rostopic name"]){
            transformINFO->isReference_ = true;
            topicName = node[transformINFO->name_]["reference rostopic name"].as<string>();
            ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, PoseStamped), Transforms>
                (transformINFO->referenceSub_, transformINFO->rosNode_, topicName, 1, &Transforms::referenceTransformCallback, transformINFO);
        }

        if (node[transformINFO->name_]["filter"]){
                transformINFO->isFiltered_ = true;
                if (node[transformINFO->name_]["filter"]["alpha"]){
                transformINFO->alpha_ = node[transformINFO->name_]["filter"]["alpha"].as<double>();
            }
        }
    }
}

void afTFPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afTFPlugin::close(){
    return -1;
}
