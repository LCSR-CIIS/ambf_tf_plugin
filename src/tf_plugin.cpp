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

// Apply a diagonal frame conversion M = diag(sx, sy, sz) to a btTransform.
// Position: component-wise scale. Rotation: M * R * M (each entry scaled by sx_i * sx_j).
// All supported conversions are self-inverse (M^2 = I), so the same function handles both directions.
static btTransform applyDiagFrameConv(const btTransform& t, btScalar sx, btScalar sy, btScalar sz){
    btVector3 o = t.getOrigin();
    o.setValue(sx * o.x(), sy * o.y(), sz * o.z());
    const btMatrix3x3& r = t.getBasis();
    btScalar s[3] = {sx, sy, sz};
    btMatrix3x3 converted(
        s[0]*r[0][0]*s[0], s[0]*r[0][1]*s[1], s[0]*r[0][2]*s[2],
        s[1]*r[1][0]*s[0], s[1]*r[1][1]*s[1], s[1]*r[1][2]*s[2],
        s[2]*r[2][0]*s[0], s[2]*r[2][1]*s[1], s[2]*r[2][2]*s[2]
    );
    return btTransform(converted, o);
}

static btTransform applyFrameConversion(FrameConversion conv, const btTransform& t){
    switch(conv){
        case FrameConversion::LPS_TO_RPS:
        case FrameConversion::RPS_TO_LPS:
            return applyDiagFrameConv(t, -1, 1, 1);
        case FrameConversion::OPENGL_TO_OPENCV:
        case FrameConversion::OPENCV_TO_OPENGL:
            return applyDiagFrameConv(t, 1, -1, -1);
        default:
            return t;
    }
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
        // Apply the first message immediately so a single latched publication
        // takes effect. Without this, transformation_ would stay identity until
        // a *second* message arrives, which never happens for latch=True topics.
        transformation_ = filteredTransform_;
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
        q_blend.slerp(alpha_, q_prev, q_curr);

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
        // Apply immediately so a single latched publication takes effect.
        reference_trans_ = filteredReferenceTransform_;
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
        q_blend.slerp(alpha_, q_prev, q_curr);

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
        cerr << "> TF plugin initialized successfully!" << endl;
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

// Compose the child's local (parent-relative) transform into a world-frame
// command, exactly as moveRigidBody applies it. Shared so the apply-once settle
// check compares against the same target the controller is driving toward.
static btTransform computeWorldCommand(const Transforms* transformINFO, const btTransform& transform){
    if (transformINFO->parentRB_){
        btTransform parentTransform;
        transformINFO->parentRB_->m_bulletRigidBody->getMotionState()->getWorldTransform(parentTransform);
        return parentTransform * transform;
    }
    return transform;
}

// True when 'curr' is within (posTol metres, rotTol radians) of 'target'.
static bool isTransformSettled(const btTransform& curr, const btTransform& target,
                               double posTol, double rotTol){
    btScalar posErr = (curr.getOrigin() - target.getOrigin()).length();
    btScalar d = btFabs(curr.getRotation().dot(target.getRotation()));
    if (d > btScalar(1.0)) d = btScalar(1.0);
    btScalar angErr = btScalar(2.0) * btAcos(d);   // geodesic angle between orientations
    return posErr <= posTol && angErr <= rotTol;
}

void afTFPlugin::moveRigidBody(const Transforms* transformINFO, const btTransform transform, double dt){
    btTransform command = computeWorldCommand(transformINFO, transform);

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


void afTFPlugin::applyWorldTransform(const Transforms* transformINFO, const btTransform& worldCommand, double dt){
    if (transformINFO->childRB_->m_bulletRigidBody->isStaticOrKinematicObject()){
        transformINFO->childRB_->m_bulletRigidBody->getMotionState()->setWorldTransform(worldCommand);
        transformINFO->childRB_->m_bulletRigidBody->setWorldTransform(worldCommand);
    }
    else{
        btTransform curr_trans = transformINFO->childRB_->getCOMTransform();
        btVector3 pCommand = transformINFO->childRB_->m_controller.computeOutput<btVector3>(curr_trans.getOrigin(), worldCommand.getOrigin(), dt);
        btVector3 rCommand = transformINFO->childRB_->m_controller.computeOutput<btVector3>(curr_trans.getBasis(), worldCommand.getBasis(), dt);

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
        // Apply-once: once the subscribed transform has been applied a single
        // time, stop re-asserting it so the body can move freely afterward.
        if (m_transformList[i]->applyOnce_ && m_transformList[i]->applied_)
            continue;

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
            if (m_transformList[i]->frameConversion_ != FrameConversion::NONE)
                transform = applyFrameConversion(m_transformList[i]->frameConversion_, transform);
            if (m_transformList[i]->invertSubscribed_)
                transform = transform.inverse();
            if (m_transformList[i]->hasPreTransform_)
                transform = m_transformList[i]->preTransform_ * transform;

            moveRigidBody(m_transformList[i], transform, dt);

            // Apply-once latch. For ROS, wait for a real message (initialized_);
            // FIXED has no message so it is eligible immediately. A static/
            // kinematic child reaches the pose exactly in one tick; a dynamic
            // child is driven by the controller, so latch only once it has
            // settled within tolerance of the commanded world pose.
            if (m_transformList[i]->applyOnce_ &&
                (m_transformList[i]->transformType_ == TransformationType::FIXED ||
                 m_transformList[i]->initialized_)){
                afRigidBodyPtr child = m_transformList[i]->childRB_;
                if (child->m_bulletRigidBody->isStaticOrKinematicObject()){
                    m_transformList[i]->applied_ = true;
                }
                else{
                    btTransform target = computeWorldCommand(m_transformList[i], transform);
                    if (isTransformSettled(child->getCOMTransform(), target,
                                           m_transformList[i]->settlePosTol_,
                                           m_transformList[i]->settleRotTol_))
                        m_transformList[i]->applied_ = true;
                }
            }
        }

        else if (m_transformList[i]->transformType_ == TransformationType::ROS_RELATIVE){
            ambf_ral::spin_some(m_transformList[i]->rosNode_);

            btTransform subscribedDelta = to_btTransform(m_transformList[i]->transformation_);
            if (m_transformList[i]->frameConversion_ != FrameConversion::NONE)
                subscribedDelta = applyFrameConversion(m_transformList[i]->frameConversion_, subscribedDelta);
            if (m_transformList[i]->invertSubscribed_)
                subscribedDelta = subscribedDelta.inverse();

            const btTransform& T0 = m_transformList[i]->initialChildTransform_;
            btVector3   deltaPos = subscribedDelta.getOrigin();
            btMatrix3x3 deltaRot = subscribedDelta.getBasis();

            if (m_transformList[i]->hasPreTransform_){
                // Treat pre_transform as a change-of-basis: similarity transform for rotation,
                // plain rotation for position (avoids contamination by T0's rotation).
                const btMatrix3x3 R_P = m_transformList[i]->preTransform_.getBasis();
                deltaPos = R_P * deltaPos;
                deltaRot = R_P * deltaRot * R_P.transpose();
            }

            // Position: add world-frame delta directly (no T0 rotation applied to delta).
            // Rotation: apply delta in world frame on top of the child's initial rotation.
            btTransform worldTarget(deltaRot * T0.getBasis(), T0.getOrigin() + deltaPos);

            applyWorldTransform(m_transformList[i], worldTarget, dt);

            if (m_transformList[i]->applyOnce_ && m_transformList[i]->initialized_){
                afRigidBodyPtr child = m_transformList[i]->childRB_;
                if (child->m_bulletRigidBody->isStaticOrKinematicObject()){
                    m_transformList[i]->applied_ = true;
                }
                else if (isTransformSettled(child->getCOMTransform(), worldTarget,
                                            m_transformList[i]->settlePosTol_,
                                            m_transformList[i]->settleRotTol_)){
                    m_transformList[i]->applied_ = true;
                }
            }
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
                else if (node[transformName]["type"].as<string>() =="INITIAL")
                    transformINFO->transformType_ = TransformationType::INITIAL;
                else if (node[transformName]["type"].as<string>() == "ROS")
                    transformINFO->transformType_ = TransformationType::ROS;
                else if (node[transformName]["type"].as<string>() == "ROS_RELATIVE")
                    transformINFO->transformType_ = TransformationType::ROS_RELATIVE;

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
                cerr << "[ERROR!!] Transformation " << transformName << " not found in the yaml file!" << endl;
                return -1;
            }
        }
        return 1;
    }
    else {
        cerr << "[ERROR!!] No transformation list found in the yaml file!" << endl;
        return -1;
    }
}

static void parseFrameConversion(Transforms* transformINFO, YAML::Node& node){
    if (!node[transformINFO->name_]["convert frame"]) return;
    string conv = node[transformINFO->name_]["convert frame"].as<string>();
    if      (conv == "LPS_to_RPS")       transformINFO->frameConversion_ = FrameConversion::LPS_TO_RPS;
    else if (conv == "RPS_to_LPS")       transformINFO->frameConversion_ = FrameConversion::RPS_TO_LPS;
    else if (conv == "OpenGL_to_OpenCV") transformINFO->frameConversion_ = FrameConversion::OPENGL_TO_OPENCV;
    else if (conv == "OpenCV_to_OpenGL") transformINFO->frameConversion_ = FrameConversion::OPENCV_TO_OPENGL;
    else cerr << "[WARNING] Unknown convert frame value: " << conv << endl;
}

static void parseSubscribedModifiers(Transforms* transformINFO, YAML::Node& node){
    const string& name = transformINFO->name_;

    if (node[name]["invert"] && node[name]["invert"].as<bool>())
        transformINFO->invertSubscribed_ = true;

    if (node[name]["apply once"] && node[name]["apply once"].as<bool>())
        transformINFO->applyOnce_ = true;

    // Optional settle tolerances for apply-once on a dynamic child.
    if (node[name]["settle position tolerance"])
        transformINFO->settlePosTol_ = node[name]["settle position tolerance"].as<double>();
    if (node[name]["settle orientation tolerance"])
        transformINFO->settleRotTol_ = node[name]["settle orientation tolerance"].as<double>();

    if (node[name]["pre transform"]){
        YAML::Node pt = node[name]["pre transform"];
        chai3d::cTransform cPre;
        if (pt["position"] && pt["orientation"]){
            YAML::Node ptPos = pt["position"];
            YAML::Node ptOri = pt["orientation"];
            cVector3d trans = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&ptPos));
            cMatrix3d rot   = to_cMatrix3d(adf_loader_1_0::ADFUtils::rotationFromNode(&ptOri));
            cPre.setLocalPos(trans);
            cPre.setLocalRot(rot);
        }
        transformINFO->preTransform_ = to_btTransform(cPre);
        transformINFO->hasPreTransform_ = true;
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

        parseFrameConversion(transformINFO, node);
        parseSubscribedModifiers(transformINFO, node);
    }

    else if (transformINFO->transformType_ == TransformationType::ROS_RELATIVE){
        // Same subscriber setup as ROS
        transformINFO->rosNode_ = afROSNode::getNodeAndRegister("AMBF_TF_Plugin_Node");
        string topicName = node[transformINFO->name_]["rostopic name"].as<string>();
        ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, PoseStamped), Transforms>
            (transformINFO->transformSub_, transformINFO->rosNode_, topicName, 1, &Transforms::transformCallback, transformINFO);

        if (node[transformINFO->name_]["filter"]){
            transformINFO->isFiltered_ = true;
            if (node[transformINFO->name_]["filter"]["alpha"]){
                transformINFO->alpha_ = node[transformINFO->name_]["filter"]["alpha"].as<double>();
            }
        }

        parseFrameConversion(transformINFO, node);
        parseSubscribedModifiers(transformINFO, node);

        // Capture the child's initial world transform as the reference origin
        transformINFO->childRB_->m_bulletRigidBody->getMotionState()->getWorldTransform(transformINFO->initialChildTransform_);
        transformINFO->isInitialCaptured_ = true;
    }
}

void afTFPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afTFPlugin::close(){
    return -1;
}
