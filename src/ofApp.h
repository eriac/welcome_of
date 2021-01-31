#pragma once

#include "ofMain.h"
#include "ofxQuadWarp.h"
#include "letter_controller.h"
#include "background_controller.h"
#include "fukidashi_controller.h"
#include "photo_controller.h"
#include "toio_controller.h"

class ofApp : public ofBaseApp{
	public:
		ofApp();
		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		ofxQuadWarp warper0_;
		ofFbo fbo0_;
		ofxQuadWarp warper1_;
		ofFbo fbo1_;
		bool warp_marker_;

		int global_counter_;
		bool play_;
	    int fps_;

		BackGroundController background_controller_;
		LetterController letter_controller_;
		PhotoController photo_controller_;
		FukidashiController fukidashi_controller0_;
		FukidashiController fukidashi_controller1_;
		ToioController toio_controller0_;
		ToioController toio_controller1_;
};
