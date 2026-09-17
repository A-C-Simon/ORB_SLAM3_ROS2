#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    trackingImagePub_ = this->create_publisher<ImageMsg>("orbslam3/tracking_image", 2);

    pose_publisher_ = std::make_unique<OrbPosePublisher>(this);
    syncThread_ = std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    running_ = false;
    if (syncThread_.joinable()) {
        syncThread_.join();
    }

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bufMutex_);
    imuBuf_.push(msg);
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    std::lock_guard<std::mutex> lock(bufMutexLeft_);

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    std::lock_guard<std::mutex> lock(bufMutexRight_);

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return {};
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (running_ && rclcpp::ok())
    {
        {
            ImageMsg::SharedPtr left_msg;
            ImageMsg::SharedPtr right_msg;
            vector<ORB_SLAM3::IMU::Point> imu_measurements;
            double tImLeft = 0.0;
            double tImRight = 0.0;

            {
                std::scoped_lock lock(bufMutex_, bufMutexLeft_, bufMutexRight_);
                if (imgLeftBuf_.empty() || imgRightBuf_.empty() || imuBuf_.empty()) {
                    // Release all locks before sleeping so subscriptions can fill queues.
                } else {
                    tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                    tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
                    if (std::abs(tImLeft - tImRight) <= maxTimeDiff &&
                        tImLeft <= Utility::StampToSec(imuBuf_.back()->header.stamp)) {
                        left_msg = imgLeftBuf_.front();
                        right_msg = imgRightBuf_.front();
                        imgLeftBuf_.pop();
                        imgRightBuf_.pop();
                        while (!imuBuf_.empty() &&
                               Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft) {
                            const auto & imu = imuBuf_.front();
                            const double t = Utility::StampToSec(imu->header.stamp);
                            cv::Point3f acc(imu->linear_acceleration.x,
                                            imu->linear_acceleration.y,
                                            imu->linear_acceleration.z);
                            cv::Point3f gyr(imu->angular_velocity.x,
                                            imu->angular_velocity.y,
                                            imu->angular_velocity.z);
                            imu_measurements.emplace_back(acc, gyr, t);
                            imuBuf_.pop();
                        }
                    }
                }
            }

            if (!left_msg || !right_msg || imu_measurements.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            cv::Mat imLeft = GetImage(left_msg);
            cv::Mat imRight = GetImage(right_msg);
            if (imLeft.empty() || imRight.empty()) {
                continue;
            }

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            const auto pose = SLAM_->TrackStereo(
              imLeft, imRight, tImLeft, imu_measurements);
            cv::Mat tracking_image;
            cv::cvtColor(imLeft, tracking_image, cv::COLOR_GRAY2BGR);
            cv::drawKeypoints(
              tracking_image, SLAM_->GetTrackedKeyPointsUn(), tracking_image,
              cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            trackingImagePub_->publish(
              *cv_bridge::CvImage(left_msg->header, "bgr8", tracking_image).toImageMsg());
            const bool inertial_map_ready =
              SLAM_->GetTrackingState() == ORB_SLAM3::Tracking::OK &&
              SLAM_->GetTimeFromIMUInit() > 0.0;
            // Map points can be retired while the initializer resets/replaces
            // maps. Do not dereference them until the inertial map is stable.
            pose_publisher_->publish(
              pose, left_msg->header.stamp,
              inertial_map_ready ? SLAM_->GetTrackedMapPoints() :
                std::vector<ORB_SLAM3::MapPoint *>(),
              inertial_map_ready);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
