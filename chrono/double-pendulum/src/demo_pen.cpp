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

void apply_fan_force (	ChSystem* msystem,		// contains all bodies
						ChCoordsys<>& fan_csys, // pos and rotation of fan 
						double aradius,			// radius of fan
						double aspeed,			// speed of fan
						double adensity)		// density (heuristic)
{
	for (unsigned int i=0; i<msystem->Get_bodylist()->size(); i++)
	{
		ChBody* abody = (*msystem->Get_bodylist())[i];

		// Remember to reset 'user forces accumulators':
		abody->Empty_forces_accumulators();

		// initialize speed of air (steady, if outside fan stream): 
		ChVector<> abs_wind(0,0,0);

		// calculate the position of body COG in fan coordinates:
		ChVector<> mrelpos = fan_csys.TransformParentToLocal(abody->GetPos());
		ChVector<> mrelpos_ondisc = mrelpos; mrelpos_ondisc.z=0;
		
		if (mrelpos.z >0) // if not behind fan..
			if (mrelpos_ondisc.Length() < aradius)	
			{
				//OK! we are inside wind stream cylinder..
				// wind is directed as normal to the fan disc
				abs_wind = fan_csys.TransformLocalToParent(ChVector<>(0,0,1));
				// wind inside fan stream is constant speed 
				abs_wind *= -aspeed;
			}

		// force proportional to relative speed body-wind 
		// and fluid density (NOTE! pretty simplified physics..)
		ChVector<> abs_force = ( abs_wind - abody->GetPos_dt() ) * adensity;
		// apply this force at the body COG
		abody->Accumulate_force(abs_force, abody->GetPos(), false);
	}
}

int main(int argc, char* argv[])
{
	ChSystem my_system;


}
