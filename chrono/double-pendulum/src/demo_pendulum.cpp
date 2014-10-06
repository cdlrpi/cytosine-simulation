//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - creating a pendulum 
//     - apply custom forces using accumulators
//     - creating constraints with limits
//     - 3D viewing with the Irrlicht library
//  
//	 CHRONO  
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
   

#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 
#include "unit_IRRLICHT/ChIrrAppInterface.h"
#include "core/ChTimer.h"
#include "core/ChRealtimeStep.h"

#include <irrlicht.h>



// Use the namespace of Chrono
using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;



// This function will be used to apply forces caused by
// a rotating fan, to all objects in front of it (a simple 
// exaple just to demonstrate how to apply custom forces).


int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem my_system;

	ChSharedBodyPtr my_body_A(new ChBody);
	ChSharedBodyPtr my_body_B(new ChBody);
	ChSharedBodyPtr my_body_C(new ChBody);

	my_body_A->SetName("fix");
        my_body_B->SetName("pen1");
        my_body_C->SetName("pen2");

	my_body_A->SetBodyFixed(true);            
	my_body_B->SetPos(ChVector<>(1,0,0));
	my_body_C->SetPos(ChVector<>(4,0,0));

	ChSharedMarkerPtr my_marker_b(new ChMarker);
	ChSharedMarkerPtr my_marker_c(new ChMarker);

        my_marker_b->SetName("fix_pen1");
        my_marker_c->SetName("pen1_pen2");
	
	my_body_B->AddMaker(my_marker_b);
	my_body_C->AddMaker(my_marker_c);

	my_marker_b->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(0,0,0)));
	my_marker_c->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(0,0,0)));


	ChSharedPtr<ChLinkLockRevolute> my_link_AB(new ChLinkLockRevolute);
	my_link_AB->Initialize( my_body_A,my_body_B);

	ChSharedPtr<ChLinkLockRevolute> my_link_BC(new ChLinkLockRevolute);
	my_link_BC->Initialize( my_body_B,my_body_C);

	my_system.AddBody(my_body_A);
	my_system.AddBody(my_body_B);
	my_system.AddBody(my_body_C);
	my_system.AddLink(my_link_AB);
	my_system.AddLink(my_link_BC);

	while (my_system.GetTime()<10)
	{
		my_system.StepDynamics(0.01);
		GetLog() << "Time: "
			 << chronoTime
			 << "  Slider X position: "
			 << my_link_CA->GetMarker1()->GetAbsCoord().pos.x
			 << "\n";

	}
 

	return 0;
}


