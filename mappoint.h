#pragma once
#include"common_include.h"
namespace myslam
{ 
	class MapPoint

	{
	public:
		typedef std::shared_ptr<MapPoint> Ptr;
		unsigned long id_;
		Vector3d pos_;
		Vector3d norm_;
		Mat descriptor;
		int observed_times_;
		int correct_times_;

		MapPoint();
		MapPoint(long id, Vector3d position, Vector3d norm);

		static MapPoint::Ptr createMapPoint();
	};
}
