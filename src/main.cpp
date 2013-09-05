// This program creates a phone-like SMEyeL node using a local camera, like a Ps3Eye.
// It communicates just like another smartphone, through the Framework.libCommunication.PhoneProxy.

#include "MyPhoneServer.h"

using namespace std;

char *inifilename = "default.ini";	// 1st command line parameter overrides it

MyPhoneServer server;

int main(int argc, char *argv[])
{
	server.init(inifilename,argc,argv);

/*	if (server.RegisterNode("avalon.aut.bme.hu","~kristof/smeyel/smeyel_reg.php?IP=127.0.0.1:6000"))	// TODO: do not hardwire the port!
	{
		cout << "Error: could not register the node..." << endl;
	}*/

	cout << "Entering infinite service loop..." << endl;
	server.Run();
	cout << "Server finished." << endl;
	return 0;
}
