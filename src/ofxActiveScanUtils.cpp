#include "ofxActiveScanUtils.h"

namespace ofxActiveScan {

ofImage toOf(Map2f field) {
	ofImage img;
	int w = field.size(0);
	int h = field.size(1);
	img.allocate(w, h, OF_IMAGE_COLOR);
	
	for( int y = 0 ; y < h ; y++ ) {
		for( int x = 0 ; x < w ; x++ ) {
			img.setColor(x, y, field.cell(x, y) * 255);
		}
	}
	
	return img;
}

};
