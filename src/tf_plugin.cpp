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

// Convert chai3d::cTransform to btTransform
void convertChaiToBulletTransform(chai3d::cTransform& cTrans, btTransform& btTrans){
    
    // Set translation
    btVector3 translation;
    translation.setValue(cTrans.getLocalPos().x(), cTrans.getLocalPos().y(), cTrans.getLocalPos().z());

    // Set rotation
    btMatrix3x3 btRotationMatrix;
    btRotationMatrix.setValue(
        cTrans.getLocalRot().getRow(0).x(), cTrans.getLocalRot().getRow(0).y(), cTrans.getLocalRot().getRow(0).z(),
        cTrans.getLocalRot().getRow(1).x(), cTrans.getLocalRot().getRow(1).y(), cTrans.getLocalRot().getRow(1).z(),
        cTrans.getLocalRot().getRow(2).x(), cTrans.getLocalRot().getRow(2).y(), cTrans.getLocalRot().getRow(2).z()
    );

    btTrans.setOrigin(translation);
    btTrans.setBasis(btRotationMatrix);
}

void Transforms::transformCallback(geometry_msgs::PoseStampedConstPtr msg){
    transformation_.setLocalPos(cVector3d(msg->pose.position.x,
                                        msg->pose.position.y,
                                        msg->pose.position.z));
    cQuaternion rot(msg->pose.orientation.w,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z);
    cMatrix3d rotM;
    rot.toRotMat(rotM);
    transformation_.setLocalRot(rotM);
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
            ("tf_list", p_opt::value<string>()->default_value(""), "Name of tf_list yaml file");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    // Loading options 
    string tf_list_path = var_map["tf_list"].as<string>();

    // Define path
    string file_path = __FILE__;
    m_current_filepath = file_path.substr(0, file_path.rfind("/"));

    // Get pointer to World
    m_worldPtr = a_afWorld;

    // Improve the constratint
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; // improve out of plane error of joints

    // When config file was defined
    if(!tf_list_path.empty()){
        int result = readTFListYaml(tf_list_path);

        // Check for the stored transformation list and if the type is INITIAL, move the corresponding object
        for (size_t i = 0; i < m_transformList.size(); i++){
            if (m_transformList[i]->transformType_ == TransformationType::INITIAL){
                btTransform transform;
                convertChaiToBulletTransform(m_transformList[i]->transformation_, transform);
                moveRigidBody(m_transformList[i], transform);
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
}

void afTFPlugin::graphicsUpdate(){
}

void afTFPlugin::moveRigidBody(const Transforms* transformINFO, const btTransform transform){
    btTransform command;
    if (transformINFO->parentRB_){
        btTransform parentTransform;
        transformINFO->parentRB_->m_bulletRigidBody->getMotionState()->getWorldTransform(parentTransform);
        command = parentTransform * transform;
    }
    else{
        command = transform;
    }

    // Apply transformation to the child body
    transformINFO->childRB_->m_bulletRigidBody->getMotionState()->setWorldTransform(command);
    transformINFO->childRB_->m_bulletRigidBody->setWorldTransform(command);
}


void afTFPlugin::physicsUpdate(double dt){   
    for (size_t i = 0; i < m_transformList.size(); i++){
        if (m_transformList[i]->transformType_ == TransformationType::FIXED ||
        m_transformList[i]->transformType_ == TransformationType::ROS){
            btTransform transform;
            convertChaiToBulletTransform(m_transformList[i]->transformation_, transform);
            moveRigidBody(m_transformList[i], transform);
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
                }
                
                // Store child information
                transformINFO->childRB_ = m_worldPtr->getRigidBody(node[transformName]["child"].as<string>());

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
        transformINFO->rosNode_ = afROSNode::getNode();
        string topicName = node[transformINFO->name_]["rostopic name"].as<string>();
        transformINFO->transformSub_ = transformINFO->rosNode_->subscribe(topicName, 1, &Transforms::transformCallback, transformINFO);
    }
}

void afTFPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afTFPlugin::close(){
    return -1;
}
