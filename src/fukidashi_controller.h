#pragma once
#include "ofApp.h"
#include <cmath>
#include <string>
#include "ofxEasing.h"
#include "ofxTrueTypeFontUC.h"

enum class FukidashiType { N1R, N2R, N1L, N2L };

class FukidashiController{
public:
    FukidashiController(void){
        enable_ = false;
     }

    void setup(FukidashiType type, std::string text){
        enable_ = true;
        text_ = text;
        local_counter_ = 0;
        if(type == FukidashiType::N1R){
            image_.load("picture/hukidashi1_left.png");
            fukidashi_x_ = 560;
            fukidashi_y_ = 650;
            fukidashi_w_ = 300;
            fukidashi_h_ = 150;
        }
        else if(type == FukidashiType::N2R){
            image_.load("picture/hukidashi1_right.png");
            fukidashi_x_ = 650;
            fukidashi_y_ = 650;
            fukidashi_w_ = 300;
            fukidashi_h_ = 150;
        }
        else if(type == FukidashiType::N1L){
            image_.load("picture/hukidashi1_right.png");
            fukidashi_x_ = 240;
            fukidashi_y_ = 650;
            fukidashi_w_ = 300;
            fukidashi_h_ = 150;
        }
        else if(type == FukidashiType::N2L){
            image_.load("picture/hukidashi1_left.png");
            fukidashi_x_ = 150;
            fukidashi_y_ = 650;
            fukidashi_w_ = 300;
            fukidashi_h_ = 150;
        }

        duration1_ = 0.2;
        duration2_ = 2.6;
        duration3_ = 0.2;

        font_.load("verdana.ttf", 24);
        font_.setLineHeight(10);
        font_.setLetterSpacing(1.0);
        jfont_.loadFont("logotypejp_mp_m_1.1.ttf", 24);
    }

    void update(){
        if(enable_){
            float local_time = local_counter_ / 30.0;            

            if(local_time < duration1_){
                size_ = ofxeasing::map(local_time, 0.0, duration1_, 0.0, 1.0, ofxeasing::linear::easeOut);
                show_text_ = false;
            }
            else if(local_time < duration1_ + duration2_){
                size_ = 1.0;
                show_text_ = true;
            }
            else if(local_time < duration1_ + duration2_ + duration3_){
                size_ = ofxeasing::map(local_time,  duration1_ +  duration2_, duration1_ +  duration2_ +  duration3_, 1.0, 0.0, ofxeasing::linear::easeIn);
                show_text_ = false;
            }
            else{
                size_ = 1.0;
                enable_=false;
            }
            // printf("fukidashi %u %f %f\n", local_counter_, local_time, size_);
        }
        local_counter_++;
    }

    void draw(){
        if(enable_){
            ofSetColor(255, 255, 255, 250);
	    	image_.draw(fukidashi_x_ - fukidashi_w_*size_/2 , fukidashi_y_ - fukidashi_h_*size_/2, fukidashi_w_*size_, fukidashi_h_*size_);
            if(show_text_){
                ofSetColor(0, 0, 0, 250);
                int line_count = std::count(text_.cbegin(), text_.cend(), '\n');
                jfont_.drawString(text_, fukidashi_x_-120, fukidashi_y_+10);
            }

        }
    }

    bool enable_;
	ofImage image_;
    std::string text_;
    ofTrueTypeFont font_;
    ofxTrueTypeFontUC jfont_;
    float duration1_;
    float duration2_;
    float duration3_;
    float duration4_;
    int fukidashi_x_;
    int fukidashi_y_;
    int fukidashi_w_;
    int fukidashi_h_;
    int local_counter_;
    float size_;
    bool show_text_;
};