#pragma once
#include"common_include.h"
#include"camera.h"
namespace myslam
{ 
 class Frame
{
public:
	typedef std::shared_ptr<Frame> Ptr;
	unsigned long id_; //id of this frame
	double time_stamp_; //when it is recorded
	SE3d T_c_w_;  //transform from world to camera
	Camera::Ptr camera_;  //Pinhole RGB-D Camera model
	Mat color_, depth_;  //color and depth image

public:
	Frame();
	Frame(long id, double time_stamp = 0, SE3d T_c_w = SE3d(), Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
	~Frame();

	static Frame::Ptr createFrame();

	double findDepth(const cv::KeyPoint& kp);

	Vector3d getCamCenter() const;

	//check if a point is in this frame
	bool isInFrame(const Vector3d& pt_world);
};
}