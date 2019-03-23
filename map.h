#pragma once
#include "mappoint.h"
#include "frame.h"
#include "common_include.h"
namespace myslam
{
	class Map
	{
	public:
		typedef std::shared_ptr<Map> Ptr;
		unordered_map<unsigned long, MapPoint::Ptr> map_points_;

		unordered_map<unsigned long, Frame::Ptr> keyframes_;

		Map(){}

		void insertKeyFrame(Frame::Ptr frame);
		void insertMapPoint(MapPoint::Ptr map_point);
	};

}