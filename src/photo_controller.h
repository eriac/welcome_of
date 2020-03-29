#pragma once

#include "ofApp.h"
#include <cmath>
#include <string>
#include "ofxEasing.h"

class PhotoController{
public:
    PhotoController(void){ }

    void setup(std::string file_name){
        enable_ = true;
        local_counter_ = 0;
        image_.load(file_name);
        duration1_ = 2.0;
        duration2_ = 6.0;
        duration3_ = 2.0;
    }

    void update(){
        if(enable_){
            local_counter_++;
            float local_time = local_counter_ / 30.0;            

            if(local_time < duration1_){
                pos_ = ofxeasing::map(local_time, 0.0, duration1_, 1.0, 0.0, ofxeasing::quad::easeOut);
            }
            else if(local_time < duration1_ + duration2_){
                pos_ = 0.0;
            }
            else if(local_time < duration1_ + duration2_ + duration3_){
                pos_ = ofxeasing::map(local_time,  duration1_ +  duration2_, duration1_ +  duration2_ +  duration3_, 0.0, 1.0, ofxeasing::quad::easeIn);
            }
            else{
                pos_ = 1.0;
                enable_=false;
            }
        }
        local_counter_++;
    }

    void draw(){
        if(enable_){
            ofSetColor(255, 255, 255, 250);
	    	image_.draw(300, 400 + 800 * pos_, 400, 300);
            // printf("%f %f %f %f\n", local_counter_ / 30.0, pos_);
        }
    }

    bool enable_;
	ofImage image_;
    float duration1_;
    float duration2_;
    float duration3_;
    int local_counter_;
    float pos_;
};