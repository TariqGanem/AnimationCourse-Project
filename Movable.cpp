#include "Movable.h"

Movable::Movable()
{
	T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	S = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	R = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	T1 = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

Eigen::Matrix4f Movable::MakeTrans()
{
	return T.matrix() * R.matrix() * T1.matrix() * S.matrix();
}
Eigen::Matrix4d Movable::MakeTransd()
{

	return (T.matrix() * R.matrix() * T1.matrix()).cast<double>();

}
void Movable::MyTranslate(Eigen::Vector3f amt)
{
	T.pretranslate(amt);
}
void Movable::MyTranslate1(Eigen::Vector3f amt)
{
	T1.pretranslate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	R.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	S.scale(amt);
}
void Movable::reset()
{
	T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	R = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	T1 = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}