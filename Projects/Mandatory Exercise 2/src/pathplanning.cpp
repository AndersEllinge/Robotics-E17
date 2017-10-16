#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10.

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

QPath createNewPath(double &outTime, double epsilon, int seed, WorkCell::Ptr wc, Device::Ptr device, State state) {
	rw::math::Math::seed(seed);
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = epsilon;
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

	Q from(6,-3.142,-0.827,-3.000,-3.143,0.099,-1.573);
	Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);

	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;

	//cout << "Planning from " << from << " to " << to << endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();
	outTime = t.getTime();
	return path;
}

void createLUAPath(double epsilon, int seed, WorkCell::Ptr wc, Device::Ptr device, State state) {
	std::cout << "Creating Lua path..." << std::endl; 
	double time;
	QPath path = createNewPath(time, epsilon, seed, wc, device, state);
	cout << "Took " << time << " seconds to create the path" << std::endl;
	ofstream LUA;
	LUA.open("setQs.txt");
	for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		LUA << "setQ({" << it[0][0] << "," << it[0][1] << "," << it[0][2] << "," << it[0][3] << "," << it[0][4] << "," << it[0][5] << "})" << std::endl;
		//cout << *it << endl;
	}
	LUA.close();
}

int main(int argc, char** argv) {
	const string wcFile = "/home/student/Desktop/Robotics-E17/Projects/Mandatory Exercise 2/Kr16WallWorkCell/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	
	State state = wc->getDefaultState();
	
	// Detect initial collisions
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);
	device->setQ(from, state);
	rw::kinematics::MovableFrame* bottle = (rw::kinematics::MovableFrame*)wc->findFrame("Bottle");
	//wc->findFrame("Bottle")->attachTo(wc->findFrame("Tool"), state);
	rw::kinematics::Kinematics::gripFrame(bottle, wc->findFrame("Tool"), state);
	bottle->setTransform(Transform3D<double>(Vector3D<double>(0.0, 0.0, 0.099), RPY<double>((rw::math::Pi / 2), 0.0, -(rw::math::Pi / 2)).toRotation3D()), state);
	//state = wc->getStateStructure()->upgradeState(state);
	wc->getStateStructure()->setDefaultState(state);
	
	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;


	// Try diffenrent epsilon values
	//*
	double time;
	double deltaEpsilon = 0.1;
	double epsilon = 0.1;
	double epsilonMax = 2.01;
	int nSeeds = 20;
	QPath path;
	ofstream stats;
	stats.open("stats.csv");
	stats << "epsilon,lenght,time" << std::endl;
	
	while(epsilon < epsilonMax) {
		std::cout << "Testing epsilon " << epsilon << "..." << std::endl;
		for(int seed = 56; seed < (56+nSeeds); seed++) {
			path = createNewPath(time, epsilon, seed, wc, device, state);
			stats << epsilon << "," << path.size() << "," << time << std::endl;
			//cout << "Epsilon: " << epsilon << " Seed: " << seed << endl;
			//cout << "Path of length " << path.size() << " found in " << time << " seconds." << endl;
		}
		epsilon = epsilon + deltaEpsilon;
	}
	
	stats.close();

	cout << "Program done." << endl;
	return 0;
}
