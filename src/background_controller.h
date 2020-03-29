#pragma once
#include "ofApp.h"
#include <cmath>
#include <string>
#include "ofxEasing.h"

class BackGroundController{
public:
    BackGroundController(void){
        image1_ptr_.reset();
        image2_ptr_.reset();
    }

    void setup(std::string file_name){
        move_ = true;
        local_counter_ = 0;
        image2_ptr_ = image1_ptr_;
        image1_ptr_ = std::make_shared<ofImage>();
        image1_ptr_->load(file_name);
        duration1_ = 2.0;
        rate_ = 0.0;

        window_w_ = 800;
        window_h_ = 800;
    }

    void update(){
        if(move_){
            local_counter_++;
            float local_time = local_counter_ / 30.0;            
            if(local_time < duration1_){
                rate_ = ofxeasing::map(local_time, 0.0, duration1_, 0.0, 1.0, ofxeasing::linear::easeOut);
            }
            else{
                rate_ = 1.0;
                move_ = false;
            }
        }
        local_counter_++;
    }

    void draw(){
        if(move_){
            if(image1_ptr_ && !image2_ptr_){
                ofClear(0, 0, 0);
                ofSetColor(255, 255, 255, (int)(255*rate_));
                image1_ptr_->draw(0, 0, window_w_, window_h_);
            }
            else if(image1_ptr_ && image2_ptr_){
                ofSetColor(255, 255, 255);
                image2_ptr_->draw(0, 0, window_w_, window_h_);
                ofSetColor(255, 255, 255, (int)(255*rate_));
                image1_ptr_->draw(0, 0, window_w_, window_h_);
            }
        }
        else{
            if(image1_ptr_){
                ofSetColor(255, 255, 255);
                image1_ptr_->draw(0, 0, window_w_, window_h_);
            }
        }
    }

    bool move_;
	std::shared_ptr<ofImage> image1_ptr_;
	std::shared_ptr<ofImage> image2_ptr_;
    float duration1_;
    int window_w_;
    int window_h_;
    int local_counter_;
    float rate_;
};