#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<algorithm>
#include <boost/timer.hpp>
#include "visual_odometry.h"
#include "config.h"

namespace myslam
{ 
	
	VisualOdometry::VisualOdometry() :state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0)
    {
		num_of_features_ = Config::get<int>("number_of_features");
		scale_factor_ = Config::get<double>("scale_factor");
		level_pyramid_ = Config::get<int>("level_pyramid");
		match_ratio_ = Config::get<float>("match_ratio");
		max_num_lost_ = Config::get<float>("max_num_lost");
		min_inliers_ = Config::get<int>("min_inliers");
		key_frame_min_rot = Config::get<double>("keyframe_rotation");
		key_frame_min_trans = Config::get<double>("keyframe_translation");
		orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
    }

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
	switch (state_)
	{
	case INITIALIZING:
	{
		state_ = OK;
		curr_ = ref_ = frame;
		map_->insertKeyFrame(frame);

		extractKeyPoints();
		computeDescriptors();
		setRef3DPoints();
		break;
	}

	case OK:
	{
		curr_ = frame;
		extractKeyPoints();
		computeDescriptors();
		featureMatching();
		poseEstimationPnP();
		if (checkEstimatedPose() == true)
		{
			curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
			ref_ = curr_;
			setRef3DPoints();
			num_lost_ = 0;
			if (checkKeyFrame() == true)
			{
				addKeyFrame();
			}
		}
		else
		{
			num_lost_++;
			if (num_lost_ > max_num_lost_)
			{
				state_ = LOST;
			}
			return false;
		}
		break;
	}

	case LOST:
	{
		cout << " vo has lost." << endl;
	}
	}
	return true;
}

void VisualOdometry::extractKeyPoints()
{
	orb_->detect(curr_->color_, keypoints_curr_);
}

void VisualOdometry::computeDescriptors()
{
	orb_->compute(curr_->color_, keypoints_curr_, descriptor_curr_);
}

void VisualOdometry::featureMatching()
{
	vector<cv::DMatch> matches;
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	matcher.match(descriptor_ref_, descriptor_curr_, matches);
	float min_dis;
	if (matches.size())
		min_dis = std::min_element(matches.begin(), matches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2)
	{
		return m1.distance < m2.distance;
	}
	)->distance;
	else
		min_dis = 0;

	feature_matches.clear();
	for (cv::DMatch& m : matches)
	{
		if (m.distance < max<float>(min_dis*match_ratio_, 30.0))
		{
			feature_matches.push_back(m);
		}
	}
	cout << " good matches: " << feature_matches.size() << endl;
}

void VisualOdometry::poseEstimationPnP()
{
	vector<cv::Point3f> pts3d;
	vector<cv::Point2f> pts2d;

	for (cv::DMatch m : feature_matches)
	{
		pts3d.push_back(pts_3d_ref[m.queryIdx]);
		pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
	}

	Mat K = (cv::Mat_<double>(3, 3) << ref_->camera_->fx_, 0, ref_->camera_->cx_,
		0, ref_->camera_->fy_, ref_->camera_->cy_,
		0, 0, 1
		);

	Mat rvec, tvec, inliers;
	cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
	Eigen::Matrix3d R;
	cv::cv2eigen(rvec, R);
	num_inliers_ = inliers.rows;
	cout << "pnp inliers: " <
		20100< num_inliers_ << endl;

	T_c_r_estimated_ = SE3d(Sophus::SO3d(R), Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

}

void VisualOdometry::setRef3DPoints()
{
	pts_3d_ref.clear();
	descriptor_ref_ = Mat();
	for (size_t i = 0; i < keypoints_curr_.size(); i++)
	{
		double d = ref_->findDepth(keypoints_curr_[i]);
		if (d > 0)
		{
			Vector3d p_cam = ref_->camera_->pixel2camera(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),d);
			pts_3d_ref.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
			descriptor_ref_.push_back(descriptor_curr_.row[i]);
		}
	}
}

void VisualOdometry::addKeyFrame()
{
	cout << "adding a key-frame" << endl;
	map_->insertKeyFrame(curr_);
}

bool VisualOdometry::checkEstimatedPose()
{
	if (num_inliers_ < min_inliers_)
	{
		cout << "reject because inlier is too small:" << num_inliers_ << endl;
		return false;
	}
	Sophus::Vector6d d = T_c_r_estimated_.log();
	if (d.norm() > 5.0)
	{
		cout << "reject because inlier is too large:" << endl;
		return false;
	}

	return true;
}

bool VisualOdometry::checkKeyFrame()
{
	Sophus::Vector6d d = T_c_r_estimated_.log();
	Vector3d trans = d.head<3>();
	Vector3d rot = d.tail<3>();
	if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
	{
		return true;
	}
	return false;
}
}
