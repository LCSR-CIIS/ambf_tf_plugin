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
// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <ambf_server/ambf_ral_config.h>
#include <ambf_server/RosComBase.h>
#include <version_1_0/adf_loader_1_0.h>
#include <afConversions.h>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <chrono>

#if AMBF_ROS1
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#elif AMBF_ROS2
#include <ambf_server/ambf_ral.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#endif

enum class AudioState{
    STOPPED = 0,
    PLAYING = 1
};

namespace boost{
    namespace program_options{
        class variables_map;
    }
}

namespace p_opt = boost::program_options;

using namespace std;
using namespace ambf;

enum class TransformationType{
    FIXED=0, INITIAL=1, ROS=2, ROS_RELATIVE=3
};

// Aggregated tracking status consumed by the multiview_panels plugin to gray
// out the navigation views. NOTE: this single topic assumes only ONE tracked
// tool (the drill / pointer marker). If several transforms enable
// "tracking status" their states are AND-ed together, so tracking more tools
// independently will require per-tool topics and changes in multiview_panels.
static const string TRACKING_VALID_TOPIC = "/ambf/tf_plugin/tracking_valid";

typedef std::chrono::steady_clock TrackingClock;

enum class FrameConversion{
    NONE=0,
    LPS_TO_RPS=1,
    RPS_TO_LPS=2,
    OPENGL_TO_OPENCV=3,
    OPENCV_TO_OPENGL=4
};

class Transforms{
    public:
        Transforms();
        string name_;
        TransformationType transformType_;
        afRigidBodyPtr parentRB_ = nullptr;
        afRigidBodyPtr childRB_ = nullptr;
        chai3d::cTransform transformation_;
        chai3d::cTransform reference_trans_;

        bool isReference_ = false;

        // Initial world transform for ROS_RELATIVE type
        btTransform initialChildTransform_;
        bool isInitialCaptured_ = false;

        // Optional frame conversion applied to subscribed pose before use
        FrameConversion frameConversion_ = FrameConversion::NONE;
        bool invertSubscribed_ = false;

        // Apply the (first) subscribed transform a single time, then stop
        // re-asserting it so the body can move freely afterward.
        bool applyOnce_ = false;
        bool applied_ = false;
        // For a dynamic (non-static) child, applyOnce_ latches only once the
        // controller has driven the body to within these tolerances of the
        // commanded pose. Static/kinematic children latch in one tick (exact).
        double settlePosTol_ = 1e-3;   // metres
        double settleRotTol_ = 1e-2;   // radians (~0.57 deg)
        bool hasPreTransform_ = false;
        btTransform preTransform_;

        // Flag if need to be filtered
        bool isFiltered_ = false;
        bool initialized_ = false;
        chai3d::cTransform filteredTransform_;
        chai3d::cTransform filteredReferenceTransform_;
        double alpha_ = 0.1;

        // ROS related
        ambf_ral::node_ptr_t rosNode_;
        #if AMBF_ROS1
        std::shared_ptr<ros::Subscriber> transformSub_;
        std::shared_ptr<ros::Subscriber> referenceSub_;

        #elif AMBF_ROS2
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr transformSub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr referenceSub_;
        #endif

        // Tracking status (optional, "tracking status" block in the tf config).
        // VALID -> INVALID when no valid sample arrives for trackingTimeout_.
        // INVALID -> VALID only after valid samples keep arriving without a
        // gap for trackingRecover_ (avoids flickering at the visibility edge).
        bool trackingStatusEnabled_ = false;
        double trackingTimeout_ = 0.05;  // seconds
        double trackingRecover_ = 0.2;   // seconds
        bool trackingValid_ = false;
        bool hasValidSample_ = false;
        TrackingClock::time_point lastValidRx_;
        TrackingClock::time_point freshSince_;
        void registerTrackingSample(bool valid);
        bool updateTrackingState();

        // Audio related
        bool isMsgValid_ = true;
        void convertPoseStampedMsgTocTransform(chai3d::cTransform &trans, AMBF_RAL_MSG_PTR(geometry_msgs,PoseStamped) msg);
        void transformCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg);
        void referenceTransformCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg);
};

// Convertion function
void convertFloatTobtMatrix(double rotation[3][3], btMatrix3x3 btRotationMatrix);

class afTFPlugin: public afSimulatorPlugin{
    public:
        afTFPlugin();
        virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
        virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
        virtual void graphicsUpdate() override;
        virtual void physicsUpdate(double dt) override;
        virtual void reset() override;
        virtual bool close() override;

    protected:
        void moveRigidBody(const Transforms*, const btTransform transform, double dt);
        void applyWorldTransform(const Transforms* transformINFO, const btTransform& worldCommand, double dt);

        int readTFListYaml(string file_path);
        void readTransformationFromYaml(Transforms* transformINFO, YAML::Node& node);
        void readTrackingStatusFromYaml(Transforms* transformINFO, YAML::Node& node);
        void updateTrackingStatus();


    // private:
        // Pointer to the world
        afWorldPtr m_worldPtr;

        // Path
        string m_current_filepath;
        string m_tf_list_path;
        
        // Controllable object
        vector<Transforms*> m_transformList;

        afCameraPtr m_mainCamera; 
        string m_audioFilepath;
        chai3d::cAudioSource* m_audioSource = nullptr;
        chai3d::cAudioBuffer* m_audioBuffer = nullptr;
        chai3d::cAudioDevice* m_audioDevice = nullptr;
        AudioState m_audioState = AudioState::STOPPED;

        // Tracking status publisher (see TRACKING_VALID_TOPIC)
        ambf_ral::node_ptr_t m_trackingRosNode;
        #if AMBF_ROS1
        std::shared_ptr<ros::Publisher> m_trackingValidPub;
        #elif AMBF_ROS2
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_trackingValidPub;
        #endif
        bool m_trackingValidPublished = false;
        bool m_hasPublishedTracking = false;
        TrackingClock::time_point m_lastTrackingPublish;
        const double m_trackingHeartbeatPeriod = 0.05; // seconds (20 Hz)

};


AF_REGISTER_SIMULATOR_PLUGIN(afTFPlugin)
