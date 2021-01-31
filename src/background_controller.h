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
        if(file_name == "lines"){
            type_ = 1;
            duration1_ = 2.0;
            rate_ = 0.0;
        }
        else if(file_name == "black"){
            type_ = 2;
            image2_ptr_ = image1_ptr_;
            duration1_ = 2.0;
            rate_ = 0.0;
        }
        else{
            type_ = 0;
            image2_ptr_ = image1_ptr_;
            image1_ptr_ = std::make_shared<ofImage>();
            image1_ptr_->load(file_name);
            duration1_ = 2.0;
            rate_ = 0.0;
        }

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
        ofSeedRandom(39);
    }

    void draw(){
        if(type_ == 0){
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
        else if(type_ == 1){
            ofColor line_color;
            int span = 50;
            float threshold = 0.35;
            float param_x = ofRandom(1000);
            float param_y = ofRandom(1000);

            for (int x = 0; x < window_w_; x += span) {
        
                float noise_value = ofNoise(param_x, x * 0.005, ofGetFrameNum() * 0.005);
                if (noise_value > threshold) {
        
                    line_color.setHsb(ofMap(x, 0, window_w_, 0, 255), 239, 239);
                    ofSetColor(line_color, ofMap(noise_value, threshold, 1.0, 0, 255));
        
                    ofDrawLine(ofPoint(x, 0), ofPoint(x, window_h_));
                }
            }
        
            for (int y = 0; y < window_h_; y += span) {
        
                float noise_value = ofNoise(param_y, y * 0.005, ofGetFrameNum() * 0.005);
                if (noise_value > threshold) {
        
                    line_color.setHsb(ofMap(y, 0, window_h_, 255, 0), 239, 239);
                    ofSetColor(line_color, ofMap(noise_value, threshold, 1.0, 0, 255));
        
                    ofDrawLine(ofPoint(0, y), ofPoint(window_w_, y));
                }
            }
        
            for (int x = 0; x < window_w_; x += 50) {
        
                for (int y = 0; y < window_h_; y += 50) {
        
                    float noise_value_1 = ofNoise(param_x, x * 0.005, ofGetFrameNum() * 0.005);
                    float noise_value_2 = ofNoise(param_y, y * 0.005, ofGetFrameNum() * 0.005);
                    if (noise_value_1 > threshold && noise_value_2 > threshold) {
        
                        ofColor color_1, color_2;
                        color_1.setHsb(ofMap(x, 0, window_w_, 0, 255), 239, 239);
                        color_2.setHsb(ofMap(y, 0, window_h_, 255, 0), 239, 239);
                        ofColor blend_color = color_1 + color_2;
        
                        ofSetColor(blend_color, ofMap(noise_value_1 > noise_value_2 ? noise_value_1 : noise_value_2, threshold, 1.0, 0, 255));
                        ofDrawCircle(x, y, 5);
                    }
                }
            }
        
            if(move_ && image1_ptr_){
                ofClear(0, 0, 0);
                ofSetColor(255, 255, 255, (int)(255*(1.0-rate_)));
                image1_ptr_->draw(0, 0, window_w_, window_h_);
            }
        }
        if(type_ == 2){
            if(move_){
                ofClear(0, 0, 0);
                if(image2_ptr_){
                    ofSetColor(255, 255, 255, (int)(255*(1.0-rate_)));
                    image2_ptr_->draw(0, 0, window_w_, window_h_);
                }
            }
            else{
                ofSetColor(0, 0, 0);
            }
        }

    }

    bool move_;
    int type_; // 0: image, 1: lines
	std::shared_ptr<ofImage> image1_ptr_;
	std::shared_ptr<ofImage> image2_ptr_;
    float duration1_;
    int window_w_;
    int window_h_;
    int local_counter_;
    float rate_;
};