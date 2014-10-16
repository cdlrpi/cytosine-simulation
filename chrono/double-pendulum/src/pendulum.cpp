#include "physics/ChSystem.h"
#include "core/ChTimer.h"
#include "core/ChRealtimeStep.h"


using namespace chrono;
using namespace core;

int main(int argc, char* argv[])
{



		// The physical system: it contains all physical objects.
		ChSystem double_pendulum; 

		// Fixed point 
		ChSharedPtr<ChBody> body0(new ChBody);
		body0->SetBodyFixed(true); 
		double_pendulum.AddBody(body0);

		// Pendulum1
		ChSharedBodyPtr  pen_A(new ChBody);
        	pen_A->SetName("pendulum1");
		pen_A->SetMass(1);
		pen_A->SetPos(ChVector<>(1,0,0));
		double_pendulum.AddBody(pen_A);

		// Pendulum2
		ChSharedBodyPtr  pen_B(new ChBody);
        	pen_B->SetName("pendulum2");
		pen_B->SetPos(ChVector<>(4,0,0));
		double_pendulum.AddBody(pen_B);

		// link1, between body0 and pen1




		// Create two markers and add them to two bodies:
		// they will be used as references for 'rod-crank'link.
		ChSharedMarkerPtr my_marker_b(new ChMarker);
		ChSharedMarkerPtr my_marker_c(new ChMarker);

        my_marker_b->SetName("crank_rev");
        my_marker_c->SetName("rod_rev");

		pen_B->AddMarker(my_marker_b); 
		pen_C->AddMarker(my_marker_c);

				// Set absolute position of the two markers, 
				// for the initial position of the 'rod-crank' link:
		my_marker_b->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(2,0,0)));
		my_marker_c->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(2,0,0)));
    
				// Now create a mechanical link (a revolute joint) 
				// between these two markers, and insert in system:
		ChSharedPtr<ChLinkLockRevolute>  my_link_BC(new ChLinkLockRevolute);
		my_link_BC->Initialize(my_marker_b, my_marker_c);
        my_link_BC->SetName("REVOLUTE crank-rod");
		double_pendulum.AddLink(my_link_BC);

				// Phew! All this 'marker' stuff is boring!
				// Note that there's an easier way to create a link,
				// without needing the two markers (they will be
				// automatically created and added to the two bodies)
				// i.e. is using two bodies and a position as arguments..
				// For example, to create the rod-truss constraint:
		ChSharedPtr<ChLinkLockPointLine> my_link_CA(new ChLinkLockPointLine);
		my_link_CA->Initialize(pen_C, pen_A, ChCoordsys<>(ChVector<>(6,0,0)));
		double_pendulum.AddLink(my_link_CA);

        my_link_CA->GetMarker1()->SetName("rod_poinline");
        my_link_CA->GetMarker2()->SetName("truss_pointline");
        my_link_CA->SetName("POINTLINE rod-truss");

				// Now create a 'motor' link between crank and truss,
				// in 'imposed speed' mode:
    ChSharedPtr<ChLinkEngine> my_link_AB(new ChLinkEngine);
    my_link_AB->Initialize(pen_A, pen_B, ChCoordsys<>(ChVector<>(0, 0, 0)));
    my_link_AB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (ChSharedPtr<ChFunction_Const> mfun = my_link_AB->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
      mfun->Set_yconst(CH_C_PI); // speed w=3.145 rad/sec
    double_pendulum.AddLink(my_link_AB);

        my_link_AB->GetMarker1()->SetName("truss_engine");
        my_link_AB->GetMarker2()->SetName("crank_engine");
        my_link_AB->SetName("ENGINE truss-crank");

		GetLog() << "\n\n\nHere's the system hierarchy for slider-crank: \n\n ";
		double_pendulum.ShowHierarchy( GetLog()); 

		GetLog() << "Now use an interator to scan through already-added constraints:\n\n";
		ChSystem::IteratorLinks myiter = double_pendulum.IterBeginLinks();
		while (myiter != double_pendulum.IterEndLinks())
		{ 
			GetLog() << "   Link class: " << (*myiter)->GetRTTI()->GetName() << "  , leaves n.DOFs: "  << (*myiter)->GetLeftDOF() << "\n";
			++myiter;
		}
		


		// OK! NOW GET READY FOR THE DYNAMICAL SIMULATION!


		// A very simple simulation loop..
		double chronoTime = 0;
		while(chronoTime<2.5)
		{
			chronoTime +=0.01; 

				// PERFORM SIMULATION UP TO chronoTime
			double_pendulum.DoFrameDynamics(chronoTime);
	
				// Print something on the console..
			GetLog() << "Time: "
					 << chronoTime
					 << "  Slider X position: " 
					 << my_link_CA->GetMarker1()->GetAbsCoord().pos.x 
					 << "  Engine torque: " 
					 << my_link_AB->Get_mot_retorque()
					 << "\n";
		}


	return 0;
}

