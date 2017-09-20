#include <iostream>
#include <rw/math.hpp>

rw::math::Rotation3D<float> calcRotationRW(rw::math::Rotation3D<float> current, float R, float P, float Y);
rw::math::Rotation3D<float> calcRotation(rw::math::Rotation3D<float> current, float R, float P, float Y);
rw::math::RPY<float> calcRPY(rw::math::Rotation3D<float> rot, float threshold = 0);

int main() {

	std::cout << "Started program" << std::endl;
	
	rw::math::Rotation3D<float> rot(1.0f, 0.0f, 0.0f,
									0.0f, 1.0f, 0.0f,
									0.0f, 0.0f, 1.0f);
	
	float R = 0.2;
	float P = 0.2;
	float Y = 0.2;
	std::cout << "Input to calculation:" << std::endl << std::endl << "Matrix: " << std::endl
	<< rot << std::endl << "RPY: " << R << " " << P << " " << Y << std::endl << std::endl;

	std::cout << "Rotation has been calculated by RW to be:" << std::endl << calcRotationRW(rot, R, P, Y) << std::endl << std::endl;
	std::cout << "Rotation has been calculated by my function to be:" << std::endl << calcRotation(rot, R, P, Y) << std::endl << std::endl;
	
	rw::math::RPY<float> rpy(R, P, Y);
	rw::math::Rotation3D<float> rot2 = rpy.toRotation3D();
	std::cout << "RPY used: " << rpy << std::endl;
	//std::cout << "Rotation from RPY:" << std::endl << rot2 << std::endl;
	std::cout << "RPY calculated: " << calcRPY(rot2) << std::endl;
	std::cout << "RPY calculated: " << calcRPY(rot2, 0.9) << std::endl;
	
	
	return 0;
}

rw::math::Rotation3D<float> calcRotationRW(rw::math::Rotation3D<float> current, float R, float P, float Y) {
	
	rw::math::RPY<float> rotRPY(R, P, Y);
	rw::math::Rotation3D<float> rot = rotRPY.toRotation3D();
	return rw::math::Rotation3D<float>::multiply(current, rot);
}

rw::math::Rotation3D<float> calcRotation(rw::math::Rotation3D<float> current, float R, float P, float Y) {
	
	rw::math::Rotation3D<float> rot(cos(R)*cos(P),
	                                sin(Y)*sin(P)*cos(R) - cos(Y)*sin(R),
	                                cos(Y)*sin(P)*cos(R) + sin(R)*sin(Y),
	                                cos(P)*sin(R),
	                                sin(Y)*sin(P)*sin(R) + cos(Y)*cos(R),
	                                cos(Y)*sin(P)*sin(R) - sin(Y)*cos(R),
	                                -sin(P),
	                                sin(Y)*cos(P),
	                                cos(Y)*cos(P));
	
	return rw::math::Rotation3D<float>::multiply(current, rot);
}

rw::math::RPY<float> calcRPY(rw::math::Rotation3D<float> rot, float threshold) {
    
    std::cout << "abs is: " << fabs(rot(2,0)) << std::endl;
    
    if (fabs(rot(2,0)) < (1 - threshold)) {
        float P = asin(-rot(2,0));
        std::cout << "rot(2,0) is: " << rot(2,0) << std::endl;
        //std::cout << "P Calculated to be: " << P << std::endl;
        float Y = atan2(cos(P)*rot(2,1), cos(P)*rot(2,2));
        float R = atan2(cos(P)*rot(1,0), cos(P)*rot(0,0));
        rw::math::RPY<float> rpy(R, P, Y);
        return rpy;
    }
    
    std::cout << "Special case: " << std::endl;
    return rw::math::RPY<float>();
  
}
