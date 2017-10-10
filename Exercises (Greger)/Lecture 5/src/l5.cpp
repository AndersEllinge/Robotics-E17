#include <iostream>
#include <vector>
#include <cmath>
#include <rw/math.hpp>
#include <rw/math/Vector3D.hpp>

rw::math::Vector3D<double> cross(rw::math::Vector3D<double> v1, rw::math::Vector3D<double> v2);

rw::math::Jacobian calcJ(std::vector<rw::math::Vector3D<double>> P, std::vector<rw::math::Vector3D<double>> z, rw::math::Vector3D<double> baseTool);

int main() {

	// Inputs for the J calculation
	std::vector<rw::math::Vector3D<double>> P;
	P.push_back(rw::math::Vector3D<double>(0, 0, 3));
	P.push_back(rw::math::Vector3D<double>(0, 0, 3));
	P.push_back(rw::math::Vector3D<double>(1, 0, 3 + sqrt(3)));
	
	std::vector<rw::math::Vector3D<double>> z;
	z.push_back(rw::math::Vector3D<double>(0, 0, 1));
	z.push_back(rw::math::Vector3D<double>(0, -1, 0));
	z.push_back(rw::math::Vector3D<double>(0, -1, 0));
	
	rw::math::Vector3D<double> baseTool = rw::math::Vector3D<double>(3, 0, 3 + sqrt(3));
	
	rw::math::Jacobian J = calcJ(P, z, baseTool);
	
	//std::cout << calcJ(P, z, baseTool) << std::endl;
	
	
	return 0;
}

rw::math::Vector3D<double> cross(rw::math::Vector3D<double> v1, rw::math::Vector3D<double> v2) {
    return rw::math::Vector3D<double>(
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]);
}

rw::math::Jacobian calcJ(std::vector<rw::math::Vector3D<double>> P, std::vector<rw::math::Vector3D<double>> zT, rw::math::Vector3D<double> baseTool) {


	std::vector<rw::math::Vector3D<double>> z;
	z.push_back(rw::math::Vector3D<double>(0, 0, 1));
	z.push_back(rw::math::Vector3D<double>(0, -1, 0));
	z.push_back(rw::math::Vector3D<double>(0, -1, 0));
	std::cout << z[0] << std::endl;
	std::cout << z[1] << std::endl;
	std::cout << z[2] << std::endl;
	
	if (P.size() != z.size())
		return rw::math::Jacobian(0);
		
	rw::math::Jacobian out = rw::math::Jacobian::zero(P.size());
	rw::math::Vector3D<double> tmp;
	
	std::cout << "Size: " << P.size() << std::endl;
	
	for (int i = 0; i < P.size(); i++) {
		
		// Calculate A part
		tmp = cross(z[i], (baseTool - P[i]));
		std::cout << tmp << std::endl;
		out(i, 0) = tmp[0];
		out(i, 1) = tmp[1];
		out(i, 2) = tmp[2];
		
		// Calcualte B part
		tmp = z[i];
		std::cout << z[i] << std::endl;
		out(i, 3) = 0;
		out(i, 4) = 0;
		out(i, 5) = 0;
	}
	
	return out;
}

