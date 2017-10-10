#include <iostream>
#include <vector>
#include <cmath>
#include <rw/math.hpp>
#include <rw/math/Vector3D.hpp>

//rw::math::Vector3D<double> cross(rw::math::Vector3D<double> v1, rw::math::Vector3D<double> v2);

//rw::math::Jacobian calcJ(std::vector<rw::math::Vector3D<double>> P, std::vector<rw::math::Vector3D<double>> z, rw::math::Vector3D<double> baseTool);

int main() {

	// Inputs for the J calculation
	std::cout << "MOJN";
	std::vector<rw::math::Vector3D<double>> P;
	std::cout << "MOJN";
	P[0] = rw::math::Vector3D<double>(0, 0, 3);
	P[1] = rw::math::Vector3D<double>(0, 0, 3);
	P[2] = rw::math::Vector3D<double>(1, 0, 3 + sqrt(3));
	
	std::cout << "MOJN";
	
	std::vector<rw::math::Vector3D<double>> z;
	z[0] = rw::math::Vector3D<double>(0, 0, 1);
	z[1] = rw::math::Vector3D<double>(0, -1, 0);
	z[2] = rw::math::Vector3D<double>(0, -1, 0);
	
	rw::math::Vector3D<double> baseTool = rw::math::Vector3D<double>(3, 0, 3 + sqrt(3));
	
	//std::cout << calcJ(P, z, baseTool) << std::endl;
	
	
	return 0;
}

rw::math::Vector3D<double> cross(rw::math::Vector3D<double> v1, rw::math::Vector3D<double> v2) {
    return rw::math::Vector3D<double>(
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]);
}

rw::math::Jacobian calcJ(std::vector<rw::math::Vector3D<double>> P, std::vector<rw::math::Vector3D<double>> z, rw::math::Vector3D<double> baseTool) {

	if (P.size() != z.size())
		return rw::math::Jacobian(0);
		
	rw::math::Jacobian out = rw::math::Jacobian(P.size());
	rw::math::Vector3D<double> tmp;
	
	for (int i = 0; i < P.size(); i++) {
		
		// Calculate A part
		tmp = cross(z[i], (baseTool - P[i]));
		out(i, 0) = tmp[0];
		out(i, 0) = tmp[1];
		out(i, 0) = tmp[2];
		
		// Calcualte B part
		out(i, 0) = z[i][0];
		out(i, 0) = z[i][1];
		out(i, 0) = z[i][2];
	}
	
	return out;
}

