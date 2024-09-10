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
    
*/
//==============================================================================

#include "tf_plugin.h"

using namespace std;

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
        return readTFListYaml(tf_list_path);   
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

void afTFPlugin::physicsUpdate(double dt)
{   
    for (size_t i = 0; i < m_transformList.size(); i++){
    
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
            if (transformINFO->transformType_ == TransformationType::FIXED){
                vector<vector<double>> mat =node[transformName]["transformation"].as<vector<vector<double>>>();
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
                m_transformList.push_back(transformINFO);
            }

            else if (transformINFO->transformType_ == TransformationType::INITIAL){
                // Fixed-size array for the 3x3 rotation matrix
                double rotation[3][3];
                btVector3 translation;

                // Check if the transformation node exists
                if (node[transformName]["transformation"]) {
                    // Extract the 3x3 rotation matrix from the top-left corner
                    for (size_t i = 0; i < 3; ++i) {
                        for (size_t j = 0; j < 3; ++j) {
                            rotation[i][j] = node[transformName]["transformation"][i][j].as<double>();
                        }
                    }

                    // Extract the translation vector from the 4th column
                    translation.setValue(
                        node[transformName]["transformation"][0][3].as<double>(),
                        node[transformName]["transformation"][1][3].as<double>(),
                        node[transformName]["transformation"][2][3].as<double>()
                    );
                }

                // Convert the rotation matrix to a btMatrix3x3
                btMatrix3x3 btRotationMatrix(
                    rotation[0][0], rotation[0][1], rotation[0][2],
                    rotation[1][0], rotation[1][1], rotation[1][2],
                    rotation[2][0], rotation[2][1], rotation[2][2]
                );

                // Create the btTransform using the rotation matrix and translation vector
                btTransform transform, command;
                transform.setBasis(btRotationMatrix);
                transform.setOrigin(translation);

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

            else if (transformINFO->transformType_ == TransformationType::ROS){
                // Set up the subscriber
                m_transformList.push_back(transformINFO);
            }
        }
        return 1;
    }

    else {
        return -1;
    }
}

void afTFPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afTFPlugin::close(){
    return -1;
}
