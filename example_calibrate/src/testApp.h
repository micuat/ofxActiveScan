/*
    This file is part of ofxActiveScan.

    ofxActiveScan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ofxActiveScan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ofxActiveScan.  If not, see <http://www.gnu.org/licenses/>.

    Naoto Hieda <micuat@gmail.com> 2013
 */

#pragma once

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxActiveScan.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void init();
	void update();
	void draw();
	void keyPressed(int);
	void dragEvent(ofDragInfo);
	
	vector<string> rootDir;
	
private:
	ofxActiveScan::Options options;
	
	int cw, ch;
	
	bool pathLoaded;
};
