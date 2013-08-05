#include "testApp.h"
#include "ofAppGlutWindow.h"

int main(int argc, char **argv) {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024, 768, OF_WINDOW);
	ofSetDataPathRoot("./");
	testApp * app = new testApp;
	
	if( argc == 1 ) {
		app->rootDir.push_back("../../SharedData/");
	} else {
		for( int i = 1 ; i < argc ; i++ ) {
			string arg = argv[i];
			app->rootDir.push_back(arg);
		}
	}
	ofRunApp(app);
}
