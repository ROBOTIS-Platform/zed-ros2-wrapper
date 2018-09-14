﻿#ifndef ZED_COMPONENT_HPP
#define ZED_COMPONENT_HPP

#include "visibility_control.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "sl/Camera.hpp"

namespace stereolabs {

    // >>>>> Typedefs to simplify declarations
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> imagePub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>> camInfoPub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<stereo_msgs::msg::DisparityImage>> disparityPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TransformStamped>> transformPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> posePub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odomPub;

    typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
    typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;
    // <<<<< Typedefs to simplify declarations

    /// ZedCameraComponent inheriting from rclcpp_lifecycle::LifecycleNode
    /**
    * The ZedCameraComponent does not like the regular "talker" node
    * inherit from node, but rather from lifecyclenode. This brings
    * in a set of callbacks which are getting invoked depending on
    * the current state of the node.
    * Every lifecycle node has a set of services attached to it
    * which make it controllable from the outside and invoke state
    * changes.
    * Available Services as for Beta1:
    * - <node_name>__get_state
    * - <node_name>__change_state
    * - <node_name>__get_available_states
    * - <node_name>__get_available_transitions
    * Additionally, a publisher for state change notifications is
    * created:
    * - <node_name>__transition_event
    */
    class ZedCameraComponent : public rclcpp_lifecycle::LifecycleNode {
      public:
        ZED_PUBLIC
        /// ZedCameraComponent constructor
        /**
        * The ZedCameraComponent/lifecyclenode constructor has the same
        * arguments a regular node.
        */
        explicit ZedCameraComponent(const std::string& node_name = "zed_node",
                                    const std::string& ros_namespace = "zed",
                                    bool intra_process_comms = false);

        virtual ~ZedCameraComponent();

        /// Transition callback for state error
        /**
        * on_error callback is being called when the lifecycle node
        * enters the "error" state.
        */
        rcl_lifecycle_transition_key_t on_error(const rclcpp_lifecycle::State& previous_state);

        /// Transition callback for state shutting down
        /**
        * on_shutdown callback is being called when the lifecycle node
        * enters the "shutting down" state.
        */
        rcl_lifecycle_transition_key_t on_shutdown(const rclcpp_lifecycle::State& previous_state);

        /// Transition callback for state configuring
        /**
        * on_configure callback is being called when the lifecycle node
        * enters the "configuring" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "unconfigured".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_configure(const rclcpp_lifecycle::State&);

        /// Transition callback for state activating
        /**
        * on_activate callback is being called when the lifecycle node
        * enters the "activating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "active" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "active"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_activate(const rclcpp_lifecycle::State&);

        /// Transition callback for state deactivating
        /**
        * on_deactivate callback is being called when the lifecycle node
        * enters the "deactivating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "active".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "active"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_deactivate(const rclcpp_lifecycle::State&);

        /// Transition callback for state cleaningup
        /**
        * on_cleanup callback is being called when the lifecycle node
        * enters the "cleaningup" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "unconfigured" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rcl_lifecycle_transition_key_t on_cleanup(const rclcpp_lifecycle::State&);

      protected:
        void zedGrabThreadFunc();
        void pointcloudThreadFunc();

        void initPublishers();

        void publishImages(rclcpp::Time timeStamp);
        void publishDepthData(rclcpp::Time timeStamp);

        /** \brief Get the information of the ZED cameras and store them in an
         * information message
         * \param zed : the sl::zed::Camera* pointer to an instance
         * \param left_cam_info_msg : the information message to fill with the left
         * camera informations
         * \param right_cam_info_msg : the information message to fill with the right
         * camera informations
         * \param left_frame_id : the id of the reference frame of the left camera
         * \param right_frame_id : the id of the reference frame of the right camera
         */
        void fillCamInfo(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                         std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
                         std::string leftFrameId, std::string rightFrameId,
                         bool rawParam = false);

        /** \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param timeStamp : the ros::Time to stamp the message
         */
        void publishCamInfo(camInfoMsgPtr camInfoMsg, camInfoPub pubCamInfo, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use (different image publishers
         * exist)
         * \param img_frame_id : the id of the reference frame of the image (different
         * image frames exist)
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishImage(sl::Mat img, imagePub pubImg, std::string imgFrameId, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat depth image with a ros Publisher
         * \param depth : the depth image to publish
         * \param timeStamp : the ros::Time to stamp the depth image
         */
        void publishDepth(sl::Mat depth, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat disparity image with a ros Publisher
         * \param disparity : the disparity image to publish
         * \param timestamp : the ros::Time to stamp the depth image
         */
        void publishDisparity(sl::Mat disparity, rclcpp::Time timestamp);

        /** \brief Publish a pointCloud with a ros Publisher
         */
        void publishPointCloud();

        /* \brief Publish the pose of the camera in "Map" frame as a transformation
         * \param baseTransform : Transformation representing the camera pose from
         * odom frame to map frame
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishPoseFrame(tf2::Transform baseTransform, rclcpp::Time timeStamp);

        /* \brief Publish the odometry of the camera in "Odom" frame as a
         * transformation
         * \param odomTransf : Transformation representing the camera pose from
         * base frame to odom frame
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishOdomFrame(tf2::Transform odomTransf, rclcpp::Time timeStamp);

        /* \brief Publish the pose of the imu in "Odom" frame as a transformation
         * \param imuTransform : Transformation representing the imu pose from base
         * frame to odom framevoid
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishImuFrame(tf2::Transform imuTransform, rclcpp::Time timeStamp);

        /* \brief Publish the pose of the camera in "Map" frame with a ros Publisher
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishPose(rclcpp::Time timeStamp);

        /* \brief Publish the pose of the camera in "Odom" frame with a ros Publisher
         * \param base2odomTransf : Transformation representing the camera pose
         * from base frame to odom frame
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishOdom(tf2::Transform base2odomTransf, rclcpp::Time timeStamp);

        /* \brief Utility to initialize the pose variables
         */
        void set_pose(float xt, float yt, float zt, float rr, float pr, float yr);

        /* \bried Start tracking loading the parameters from param server
         */
        void start_tracking();

      private:
        // Status variables
        rcl_lifecycle_transition_key_t mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CREATE;

        // Timestamps
        rclcpp::Time mLastGrabTimestamp;
        rclcpp::Time mPointCloudTime;

        // Grab thread
        std::thread mGrabThread;
        bool mThreadStop = false;
        int mCamTimeoutMsec = 5000; // Error generated if camera is not available after timeout

        // Pointcloud thread
        std::thread mPcThread; // Point Cloud thread

        // Flags
        bool mPoseSmoothing = false;
        bool mSpatialMemory = false;
        bool mInitOdomWithPose = true;

        // ZED SDK
        sl::Camera mZed;

        // Params
        sl::InitParameters mZedParams;
        int mZedId = 0;
        unsigned int mZedSerialNumber = 0;
        int mZedUserCamModel = 1;   // Camera model set by ROS Param
        sl::MODEL mZedRealCamModel; // Camera model requested to SDK
        int mCamFrameRate = 30;
        std::string mSvoFilepath = "";
        std::string mOdometryDb = "";
        bool mSvoMode = false;
        bool mVerbose = true;
        int mGpuId = -1;
        int mZedResol = 2; // Default resolution: RESOLUTION_HD720
        int mZedQuality = 1; // Default quality: DEPTH_MODE_PERFORMANCE
        int mDepthStabilization = 1;
        bool mCameraFlip = false;
        int mCamSensingMode = 0; // Default Sensing mode: SENSING_MODE_STANDARD
        bool mOpenniDepthMode =
            false; // 16 bit UC data in mm else 32F in m, for more info -> http://www.ros.org/reps/rep-0118.html
        bool mPublishTf = true;
        bool mPublishMapTf = true;

        // ZED dynamic params (TODO when available in ROS2)
        double mZedMatResizeFactor = 1.0; // Dynamic...
        int mCamConfidence = 80; // Dynamic...
        double mCamMaxDepth = 10.0; // Dynamic...

        // Publishers
        imagePub mPubRgb;
        imagePub mPubRawRgb;
        imagePub mPubLeft;
        imagePub mPubRawLeft;
        imagePub mPubRight;
        imagePub mPubRawRight;
        imagePub mPubDepth;
        imagePub mPubConfImg;
        imagePub mPubConfMap;

        camInfoPub mPubRgbCamInfo;
        camInfoPub mPubRgbCamInfoRaw;
        camInfoPub mPubLeftCamInfo;
        camInfoPub mPubLeftCamInfoRaw;
        camInfoPub mPubRightCamInfo;
        camInfoPub mPubRightCamInfoRaw;
        camInfoPub mPubDepthCamInfo;
        camInfoPub mPubConfidenceCamInfo;

        disparityPub mPubDisparity;

        pointcloudPub mPubPointcloud;

        transformPub mPubPoseTransf;
        transformPub mPubOdomTransf;
        transformPub mPubImuTransf;

        posePub mPubPose;
        odomPub mPubOdom;

        // Topics
        std::string mLeftTopic;
        std::string mLeftRawTopic;
        std::string mLeftCamInfoTopic;
        std::string mLeftCamInfoRawTopic;
        std::string mRightTopic;
        std::string mRightRawTopic;
        std::string mRightCamInfoTopic;
        std::string mRightCamInfoRawTopic;
        std::string mRgbTopic;
        std::string mRgbRawTopic;
        std::string mRgbCamInfoTopic;
        std::string mRgbCamInfoRawTopic;
        std::string mDepthTopic;
        std::string mDepthCamInfoTopic;
        std::string mConfImgTopic;
        std::string mConfMapTopic;
        std::string mConfidenceCamInfoTopic;
        std::string mDispTopic;
        std::string mPointcloudTopic;
        std::string mPoseTfTopic;
        std::string mOdomTfTopic;
        std::string mImuTfTopic;
        std::string mPoseTopic;
        std::string mOdomTopic;

        // Messages
        // Camera info
        camInfoMsgPtr mRgbCamInfoMsg;
        camInfoMsgPtr mLeftCamInfoMsg;
        camInfoMsgPtr mRightCamInfoMsg;
        camInfoMsgPtr mRgbCamInfoRawMsg;
        camInfoMsgPtr mLeftCamInfoRawMsg;
        camInfoMsgPtr mRightCamInfoRawMsg;
        camInfoMsgPtr mDepthCamInfoMsg;
        camInfoMsgPtr mConfidenceCamInfoMsg;
        // Pointcloud
        pointcloudMsgPtr mPointcloudMsg;

        // Frame IDs
        std::string mRightCamFrameId;
        std::string mRightCamOptFrameId;
        std::string mLeftCamFrameId;
        std::string mLeftCamOptFrameId;
        std::string mDepthFrameId;
        std::string mDepthOptFrameId;

        std::string mMapFrameId;
        std::string mOdometryFrameId;
        std::string mBaseFrameId;
        std::string mCameraFrameId;
        std::string mImuFrameId;

        // SL Pointcloud
        sl::Mat mCloud;

        // Mats
        int mCamWidth;
        int mCamHeight;
        int mMatWidth;
        int mMatHeight;

        // Thread Sync
        std::mutex mCamDataMutex;
        std::mutex mPcMutex;
        std::condition_variable mPcDataReadyCondVar;
        bool mPcDataReady = false;

        // ROS TF2
        //std::shared_ptr<tf2_ros::TransformBroadcaster> mTransformPoseBroadcaster; // TODO enable when TransformBroadcaster can be used with LifecycleNode
        //std::shared_ptr<tf2_ros::TransformBroadcaster> mTransformOdomBroadcaster; // TODO enable when TransformBroadcaster can be used with LifecycleNode
        //std::shared_ptr<tf2_ros::TransformBroadcaster> mTransformImuBroadcaster; // TODO enable when TransformBroadcaster can be used with LifecycleNode

        // TF2 Transforms
        tf2::Transform mOdom2MapTransf;
        tf2::Transform mBase2OdomTransf;

        // TF2 Listener
        std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> mTfListener;

        geometry_msgs::msg::TransformStamped::SharedPtr
        mPoseTransfStampedMsg; // To be used for intra process communication using zero_copy
        geometry_msgs::msg::TransformStamped::SharedPtr
        mOdomTransfStampedMsg; // To be used for intra process communication using zero_copy
        geometry_msgs::msg::TransformStamped::SharedPtr
        mImuTransfStampedMsg;  // To be used for intra process communication using zero_copy

        // Tracking
        bool mTrackingActive = false;
        bool mTrackingReady = false;
        bool mResetOdom = false;
        sl::Pose mLastZedPose; // Sensor to Map transform
        sl::Transform mInitialPoseSl;
        std::vector<float> mInitialTrackPose;
    };
}

#endif
