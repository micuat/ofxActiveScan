#include "testApp.h"
#include "ofAppGlutWindow.h"

int main(int argc, char **argv) {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024, 768, OF_WINDOW);
	ofRunApp(new testApp(argc, argv));
}
