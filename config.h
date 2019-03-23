#pragma once
#include"common_include.h"

namespace myslam
{

	class Config
	{
	private:
		static std::shared_ptr<Config> config_;
		cv::FileStorage file_;

		Config() {};
	public:
		~Config();
		static void setParamterFile(const std::string& filename);

		template<typename T>
		static T get(const std::string& key)
		{
			return T(Config::config_->file_[key]);
		}
	};
}