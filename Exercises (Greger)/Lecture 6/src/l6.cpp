#include <iostream>
#include <vector>
#include <cmath>
#include <rw/math.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models.hpp>
#include <rw/loaders.hpp>

int main() {

	rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/mathias/Desktop/Robotics-E17/Exercises (Greger)/Lecture 6/URInterpolate/Scene.wc.xml");
	rw::models::Device::Ptr device = wc->findDevice("UR-6-85-5-A");
	
	if (device == NULL) {
		std::cout << "Device not found!" << std::endl;
		return 0;
	}
	
	rw::kinematics::State state = wc->getDefaultState();
	
	//rw::math::Q initQ(0, 0, 0, 0, 0, 0);
	//device->setQ(initQ, state);
	
	rw::math::Transform3D<double> baseEndTrans = device->baseTend(state);
	rw::math::Jacobian jac = device->baseJend(state);
	
	rw::math::Vector3D<double> d = baseEndTrans.P();
	rw::math::Vector3D<double> newD(d[0] + 0.00000023, d[1], d[2]);
	
	rw::math::Transform3D<double> baseEndTransDesired(newD, baseEndTrans.R());
	
	rw::math::Vector3D<double> pos = baseEndTrans.P();
	rw::math::Vector3D<double> posD = baseEndTransDesired.P();
	rw::math::Rotation3D<double> rotS = baseEndTrans.R();
	rw::math::Rotation3D<double> rotD = baseEndTransDesired.R();
	rw::math::Rotation3D<double> rot = rotS*rotD.inverse();
	
	rw::math::VelocityScrew6D<double> deltaU(	
												pos[0] - posD[0],
												pos[1] - posD[1],
												pos[2] - posD[2],
												0.5 * (rot(3,2) - rot(2,3)),
												0.5 * (rot(1,3) - rot(3,1)),
												0.5 * (rot(2,1) - rot(1,2))
												
											);

	rw::math::Q deltaQ(jac.e().inverse()*deltaU.e() );
	
	std::cout << "Delta q = " << deltaQ << std::endl;	
	
	return 0;
}

