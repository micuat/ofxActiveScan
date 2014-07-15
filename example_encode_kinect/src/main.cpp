#include "ofApp.h"
#include "ofAppGlutWindow.h"

int main(int argc, char **argv) {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024, 768, OF_WINDOW);
	ofSetDataPathRoot("./");
	ofApp * app = new ofApp;
	
	if( argc > 1 ) {
		for( int i = 1 ; i < argc ; i++ ) {
			if(argv[i][0] == '-')
				break;
			
			string arg = argv[i];
			app->rootDir.push_back(arg);
		}
	}
	ofRunApp(app);
}
