#include "Simbody.h" 
#include "Position.h"
#include "Velocity.h"
using namespace SimTK;

int main() {
// Create the system.
	MultibodySystem system;
	SimbodyMatterSubsystem matter(system);
	GeneralForceSubsystem forces(system);
	Force::UniformGravity gravity(forces, matter, Vec3(0, -9.8, 0)); 
	Body::Rigid pendulumBody(MassProperties(1.0, Vec3(0), Inertia(0)));
//	pendulumBody.addDecoration(Transform(), DecorativeSphere(0.1)); 
	MobilizedBody::Pin pendulum1(matter.Ground(), Transform(Vec3(0)),
            pendulumBody, Transform(Vec3(0, 1, 0)));
	MobilizedBody::Pin pendulum2(pendulum1, Transform(Vec3(0)),
            pendulumBody, Transform(Vec3(0, 1, 0)));
    // Set up visualization.
	//Visualizer viz(system);
	//system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
        std::cout<<"Time"<<"\t"<<"x"<<"\t"<<"y"<<"\t"<<"vx"<<"\t"<<"vy"<<std::endl;
	system.addEventReporter(new PositionReporter(system, pendulum1, 0.1));
	system.addEventReporter(new VelocityReporter(system, pendulum1, 0.1));
	//system.addEventReporter(new PositionReporter(system, pendulum2, 0.1));
	//system.addEventReporter(new VelocityReporter(system, pendulum2, 0.1));
    // Initialize the system and state.
	system.realizeTopology();
	State state = system.getDefaultState();
	pendulum2.setRate(state, 5.0);

// Simulate it.
	RungeKuttaMersonIntegrator integ(system);
	TimeStepper ts(system, integ);
	ts.initialize(state);
	ts.stepTo(10.0);
}
