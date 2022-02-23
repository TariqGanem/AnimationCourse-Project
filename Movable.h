#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	Eigen::Matrix4d MakeTransd();
	void MyTranslate(Eigen::Vector3f amt);
	void MyTranslate1(Eigen::Vector3f amt);
	void MyRotate(Eigen::Vector3f rotAxis, float angle);
	Eigen::Matrix4d MakeTransScaled();
	void MyScale(Eigen::Vector3f amt);
	void reset();
	Eigen::Transform<float, 3, Eigen::Affine> T;
	Eigen::Transform<float, 3, Eigen::Affine> T1;
	Eigen::Transform<float, 3, Eigen::Affine> R;
	Eigen::Transform<float, 3, Eigen::Affine> S;
private:


};

