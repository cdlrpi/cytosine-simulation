#include "Simbody.h"
#include <iostream>
#include <fstream>
using namespace SimTK;

class PositionReporter : public PeriodicEventReporter {
public:
	PositionReporter(const MultibodySystem& system, const MobilizedBody& mobod, Real reportInterval) 
	: PeriodicEventReporter(reportInterval), system(system), mobod(mobod) {} 
	void handleEvent(const State& state) const  { 
        	system.realize(state, Stage::Position);
        	Vec3 pos = mobod.getBodyOriginLocation(state);
		//Vec3 vel = mobod.getBodyOriginVelocity(state);
		//Vec3 acc = mobod.getBodyOriginAcceleration(state);
        	std::cout<<state.getTime()<<"\t"<<pos[0]<<"\t"<<pos[1]<<std::endl;
	}
private:
	const MultibodySystem& system;
	const MobilizedBody& mobod;
};
