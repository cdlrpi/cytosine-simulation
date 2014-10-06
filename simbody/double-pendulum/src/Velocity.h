#include "Simbody.h"
#include <iostream>
#include <fstream>
using namespace SimTK;

class VelocityReporter : public PeriodicEventReporter {
public:
	VelocityReporter(const MultibodySystem& system, const MobilizedBody& mobod, Real reportInterval) 
	: PeriodicEventReporter(reportInterval), system(system), mobod(mobod) {} 
	void handleEvent(const State& state) const  { 
        	system.realize(state, Stage::Velocity);
        	//Vec3 vel = mobod.getBodyOriginLocation(state);
		Vec3 vel = mobod.getBodyOriginVelocity(state);
		//Vec3 acc = mobod.getBodyOriginAcceleration(state);
        	std::cout<<"\t"<<vel[0]<<"\t"<<vel[1]<<std::endl;
	}
private:
	const MultibodySystem& system;
	const MobilizedBody& mobod;
};
