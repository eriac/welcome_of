#include "ofApp.h"

//--------------------------------------------------------------
ofApp::ofApp():toio_controller0_("/toio1"), toio_controller1_("/toio2"){
}

void ofApp::setup(){
    ofBackground(50);
    ofSetFrameRate(30);
    ofSetVerticalSync(true);
    ofEnableSmoothing();

    fbo0_.allocate(800, 800);
    warper0_.setSourceRect(ofRectangle(0, 0, 800, 800));              // this is the source rectangle which is the size of the image and located at ( 0, 0 )
    warper0_.setTopLeftCornerPosition(ofPoint(0, 0));             // this is position of the quad warp corners, centering the image on the screen.
    warper0_.setTopRightCornerPosition(ofPoint(800, 0));        // this is position of the quad warp corners, centering the image on the screen.
    warper0_.setBottomLeftCornerPosition(ofPoint(0, 800));      // this is position of the quad warp corners, centering the image on the screen.
    warper0_.setBottomRightCornerPosition(ofPoint(800, 800)); // this is position of the quad warp corners, centering the image on the screen.
    warper0_.setup();
    warper0_.load(); // reload last saved changes.

    // fbo1_.allocate(w, h);
    // warper1_.setSourceRect(ofRectangle(0, 0, w, h));              // this is the source rectangle which is the size of the image and located at ( 0, 0 )
    // warper1_.setTopLeftCornerPosition(ofPoint(x+100, y));             // this is position of the quad warp corners, centering the image on the screen.
    // warper1_.setTopRightCornerPosition(ofPoint(x + w, y));        // this is position of the quad warp corners, centering the image on the screen.
    // warper1_.setBottomLeftCornerPosition(ofPoint(x, y + h));      // this is position of the quad warp corners, centering the image on the screen.
    // warper1_.setBottomRightCornerPosition(ofPoint(x + w, y + h)); // this is position of the quad warp corners, centering the image on the screen.
    // warper1_.setup();

    fps_ = 30;
    global_counter_ = -5*fps_;
    // global_counter_ = 0;
    play_ = true;
    warp_marker_ = false;
}

//--------------------------------------------------------------
void ofApp::update(){
    if(global_counter_ == 0 * fps_){
        background_controller_.setup("background/table.jpg");
        toio_controller0_.setup(ToioMoveType::G1);
        toio_controller1_.setup(ToioMoveType::B1);
    }
    else if(global_counter_ == 1.0 * fps_){
        letter_controller_.setup();
    }
    else if(global_counter_ == 6.0 * fps_){
        toio_controller0_.setup(ToioMoveType::G2);
        toio_controller1_.setup(ToioMoveType::B2);
    }
    else if(global_counter_ == 8.0 * fps_){
        toio_controller0_.setup(ToioMoveType::G3);
        toio_controller1_.setup(ToioMoveType::B3);
    }
	else if(global_counter_ == 10.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N1R, "こんにちは\nえりおです");
    }
	else if(global_counter_ == 11.0 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N1L, "こんにちは\nいちこです");
    }
	else if(global_counter_ == 13.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N1R, "ぼくたちの思い出を\n紹介します");
    }
	else if(global_counter_ == 14.0 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N1L, "どうぞ\nごらんください");
    }
    else if(global_counter_ == 17.0 * fps_){
        toio_controller0_.setup(ToioMoveType::G4);
        toio_controller1_.setup(ToioMoveType::B4);
    }
	else if(global_counter_ == 19.0 * fps_){
        photo_controller_.setup("picture/pic_01.jpg");
        background_controller_.setup("background/summer.jpg");
    }
	else if(global_counter_ == 20.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N2R, "初ツーショット");
    }
	else if(global_counter_ == 21.5 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N2L, "サークルの夏合宿\nでの1枚です。");
    }
	else if(global_counter_ == 25.0 * fps_){
        photo_controller_.setup("picture/pic_02.jpg", true);
        background_controller_.setup("background/autum.jpg");
    }
	else if(global_counter_ == 26.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N2R, "メイカーフェア\nでの展示");
    }
	else if(global_counter_ == 27.5 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N2L, "私も手伝いました。");
    }
	else if(global_counter_ == 31.0 * fps_){
        photo_controller_.setup("picture/pic_03.jpg");
        background_controller_.setup("background/spring.jpg");
    }
	else if(global_counter_ == 32.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N2L, "私の大学の卒業式");
    }
	else if(global_counter_ == 33.5 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N2R, "僕も見に行っちゃい\nました");
    }
	else if(global_counter_ == 37.0 * fps_){
        photo_controller_.setup("picture/pic_04.jpg",true);
        background_controller_.setup("background/winter.png");
    }
	else if(global_counter_ == 38.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N2R, "あざらしさんカフェ");
    }
	else if(global_counter_ == 39.5 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N2L, "懐かしの池袋\nアニメプラザ");
    }
	else if(global_counter_ == 43.0 * fps_){
        photo_controller_.setup("picture/pic_05.jpg");
        background_controller_.setup("background/summer.jpg");
    }
	else if(global_counter_ == 44.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N2R, "浴衣を着て\n鎌倉デート");
    }
	else if(global_counter_ == 45.5 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N2L, "そうだ鎌倉に\n行こう");
    }
	else if(global_counter_ == 49.0 * fps_){
        photo_controller_.setup("picture/pic_06.jpg");
        background_controller_.setup("background/spring.jpg");
    }
	else if(global_counter_ == 50.0 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N2R, "そしてこの度\n今年の3月3日に");
    }
	else if(global_counter_ == 51.5 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N2L, "結婚しました\nやったね");
    }

    else if(global_counter_ == 55.0 * fps_){
        toio_controller0_.setup(ToioMoveType::G3);
        toio_controller1_.setup(ToioMoveType::B3);
    }
	else if(global_counter_ == 56.5 * fps_){
        fukidashi_controller0_.setup(FukidashiType::N1R, "どれも楽しい\n思い出だったね");
    }
	else if(global_counter_ == 57.5 * fps_){
        fukidashi_controller1_.setup(FukidashiType::N1L, "引き続き披露宴も\nお楽しみください");
    }
    else if(global_counter_ == 61.0 * fps_){
        background_controller_.setup("background/table.jpg");
        toio_controller0_.setup(ToioMoveType::G2);
        toio_controller1_.setup(ToioMoveType::B2);
    }
    else if(global_counter_ == 63.0 * fps_){
        letter_controller_.setup(LetterType::END);
    }
    else if(global_counter_ == 64.0 * fps_){
        toio_controller0_.setup(ToioMoveType::G2);
        toio_controller1_.setup(ToioMoveType::B2);
    }
    else if(global_counter_ == 68.0 * fps_){
        toio_controller0_.setup(ToioMoveType::G1);
        toio_controller1_.setup(ToioMoveType::B1);
    }
    else if(global_counter_ == 70.0 * fps_){
        background_controller_.setup("black");
    }

    if(0 <= global_counter_){
        background_controller_.update();
        letter_controller_.updata();
        photo_controller_.update();
        toio_controller0_.update();
        toio_controller1_.update();
        fukidashi_controller0_.update();
        fukidashi_controller1_.update();
    }

    if(play_){
        global_counter_++;
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255);
    
    //======================== draw image into fbo.
    
    fbo0_.begin();
    ofClear(255,255,255, 0);
    background_controller_.draw();
    letter_controller_.draw();
    photo_controller_.draw();
    toio_controller0_.draw();
    toio_controller1_.draw();
    fukidashi_controller0_.draw();
    fukidashi_controller1_.draw();
    // if(!play_){
    //     ofSetColor(255, 0, 0);
    //     ofCircle(400, 750, 30); 
    // }
    fbo0_.end();

    // fbo1_.begin();
    // fbo1_.end();

    //======================== get our quad warp matrix.
    
    ofMatrix4x4 mat0 = warper0_.getMatrix();
    ofMatrix4x4 mat1 = warper1_.getMatrix();
    
    //======================== use the matrix to transform our fbo.
    
    ofPushMatrix();
    ofMultMatrix(mat0);
    fbo0_.draw(0, 0);
    ofPopMatrix();

    ofPushMatrix();
    ofMultMatrix(mat1);
    fbo1_.draw(0, 0);
    ofPopMatrix();


    //======================== use the matrix to transform points.

    // ofSetLineWidth(2);
    // ofSetColor(ofColor::cyan);
    
    // for(int i=0; i<9; i++) {
    //     int j = i + 1;
        
    //     ofVec3f p1 = mat.preMult(ofVec3f(points[i].x, points[i].y, 0));
    //     ofVec3f p2 = mat.preMult(ofVec3f(points[j].x, points[j].y, 0));
        
    //     ofDrawLine(p1.x, p1.y, p2.x, p2.y);
    // }
    
    //======================== draw quad warp ui.
    
    if(warp_marker_){
        ofSetColor(ofColor::magenta);
        warper0_.drawQuadOutline();
        warper1_.drawQuadOutline();
        
        ofSetColor(ofColor::yellow);
        warper0_.drawCorners();
        warper1_.drawCorners();
        
        ofSetColor(ofColor::magenta);
        warper0_.drawHighlightedCorner();
        warper1_.drawHighlightedCorner();
        
        ofSetColor(ofColor::red);
        warper0_.drawSelectedCorner();
        warper1_.drawSelectedCorner();
    }
    
    //======================== info.
    
    // ofSetColor(ofColor::white);
    // ofDrawBitmapString("to warp the image, drag the corners of the image.", 20, 30);
    // ofDrawBitmapString("press 's' to toggle quad warp UI. this will also disable quad warp interaction.", 20, 50);
    // ofDrawBitmapString("press & hold 1, 2, 3, 4 to snap that point to the mouse", 20, 70);
    // ofDrawBitmapString("when a corner is selected (red), use keyboard arrow keys to nudge the corner position.", 20, 90);

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
  switch (key) {
    case 'r':
        printf("#### reset ####\n");
        global_counter_ = 0;
        break;
    case ' ':
        printf("#### play ####\n");
        play_ = !play_;
        break;
    case 'd':
        printf("#### double speed ####\n");
        ofSetFrameRate(60);
        break;
    case 'w':
        printf("#### toggle warp marker ####\n");
        warp_marker_ = !warp_marker_;
        break;
    case 'q':
        printf("#### end ####\n");
        warper0_.save();
        OF_EXIT_APP(0);
        break;
  }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}