#include "physics/ChSystem.h"
#include "core/ChTimer.h"
#include "core/ChRealtimeStep.h"

using namespace chrono;

int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem my_system;

	ChSharedBodyPtr my_body_A(new ChBody);
	ChSharedBodyPtr my_body_B(new ChBody);
	ChSharedBodyPtr my_body_C(new ChBody);
	
	my_body_A->SetBodyFixed(true);
	my_body_B->SetMass(2);
	my_body_B->SetPos(ChVector<>(1,-1,0));
	my_body_C->SetMass(3);
	my_body_C->SetPos(ChVector<>(1.5,-2,0));

//	ChSharedMarkerPtr my_marker_b(new ChMarker);
//	my_body_B->AddMarker(my_marker_b); 
//	my_marker_b->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(1,-1,0)));


	ChSharedPtr<ChLinkLockRevolute> my_link_AB(new ChLinkLockRevolute);
	my_link_AB->Initialize(my_body_A,my_body_B,ChCoordsys<>(ChVector<>(0,0,0)));
	ChSharedPtr<ChLinkLockRevolute> my_link_BC(new ChLinkLockRevolute);
	my_link_BC->Initialize(my_body_B,my_body_C,ChCoordsys<>(ChVector<>(1,-1,0)));

	my_system.AddBody(my_body_A);
	my_system.AddBody(my_body_B);
	my_system.AddBody(my_body_C);
	my_system.AddLink(my_link_AB);
	my_system.AddLink(my_link_BC);

	double chronoTime = 0;

	std::cout << "Time\t" << "x1\t" << "y1\t" <<"x2\t" << "y2" <<std::endl;
	while (chronoTime<5)
	{
		my_system.DoFrameDynamics(chronoTime);

		std::cout << chronoTime
			  << "\t"
			  << my_body_B->GetPos()(0)
			  << "\t"
			  << my_body_B->GetPos()(1)
			  << "\t"
			  << my_body_C->GetPos()(0)
			  << "\t"
			  << my_body_C->GetPos()(1)
	 		  << std::endl;
		chronoTime += 0.01;

	}
 

	return 0;
}
