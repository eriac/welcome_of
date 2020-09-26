#pragma once
#include "ofMain.h"
#include "ofxEasing.h"

class LetterController
{
public:
	LetterController(void) {}
	void setup()
	{
		enable_ = true;
		local_counter_ = 0;
		letter_image_.load("background/letter.png");
		shadow_image_.load("background/letter_shadow.png");
		logo_image_.load("background/OurMemories.png");
		duration1_ = 3.0;
		duration2_ = 4.0;
		duration3_ = 1.0;
		logo_duration1_ = 3.5;
		logo_duration2_ = 0.5;
		logo_duration3_ = 2.0;
		logo_duration4_ = 1.0;
	}

	void updata()
	{
		if (enable_)
		{
			local_counter_++;
			float local_time = local_counter_ / 30.0;

			// for x_pos_ & z_pos_
			if (local_time < duration1_)
			{
				x_pos_ = ofxeasing::map(local_time, 0.0, duration1_, -1.2, 0.0, ofxeasing::back::easeOut);
				z_pos_ = ofxeasing::map(local_time, 0.0, duration1_, 1.0, 0.0, ofxeasing::quad::easeOut);
			}
			else if (local_time < duration1_ + duration2_)
			{
				x_pos_ = 0;
				z_pos_ = 0;
			}
			else if (local_time < duration1_ + duration2_ + duration3_)
			{
				x_pos_ = ofxeasing::map(local_time, duration1_ + duration2_, duration1_ + duration2_ + duration3_, 0.0, 1.2, ofxeasing::quad::easeIn);
				z_pos_ = ofxeasing::map(local_time, duration1_ + duration2_, duration1_ + duration2_ + duration3_, 0.0, 1.0, ofxeasing::quad::easeIn);
			}
			else
			{
				x_pos_ = 1.2;
				z_pos_ = 1.0;
				enable_ = false;
			}

			// for logo_alpha_
			if (local_time < logo_duration1_)
			{
				logo_alpha_ = 0.0;
			}
			else if (local_time < logo_duration1_ + logo_duration2_)
			{
				logo_alpha_ = ofxeasing::map(local_time, logo_duration1_, logo_duration1_ + logo_duration2_, 0.0, 1.0, ofxeasing::linear::easeIn);
			}
			else if (local_time < logo_duration1_ + logo_duration2_ + logo_duration3_)
			{
				logo_alpha_ = 1.0;
			}
			else if (local_time < logo_duration1_ + logo_duration2_ + logo_duration3_ + logo_duration4_)
			{
				logo_alpha_ = ofxeasing::map(local_time, logo_duration1_ + logo_duration2_ + logo_duration3_, logo_duration1_ + logo_duration2_ + logo_duration3_ + logo_duration4_, 1.0, 0.0, ofxeasing::linear::easeIn);
			}
			else
			{
				logo_alpha_ = 0.0;
			}
		}
	}

	void draw()
	{
		if (enable_)
		{
			int center_x = 400;
			int center_y = 500;
			int letter_w = 400;
			int letter_h = 300;
			ofSetColor(255, 255, 255, 100);
			shadow_image_.draw((center_x-letter_w/2) + 800 * x_pos_ + 50 * z_pos_, (center_y-letter_h/2) + 100 * z_pos_, letter_w, letter_h);
			ofSetColor(255, 255, 255, 255);
			letter_image_.draw((center_x-letter_w/2) + 800 * x_pos_, (center_y-letter_h/2) - 50 * z_pos_, letter_w, letter_h);

			int logo_w = 300;
			int logo_h = 150;
			ofSetColor(255, 255, 255, logo_alpha_ * 250);
			logo_image_.draw((center_x-logo_w/2), (center_y-logo_h/2), logo_w, logo_h);

			// printf("%f %f %f %f\n", local_counter_ / 30.0, x_pos_, z_pos_, logo_alpha_);
		}
	}

private:
	bool enable_;
	int local_counter_;
	float counter_;
	ofImage letter_image_;
	ofImage shadow_image_;
	ofImage logo_image_;
	float x_pos_;
	float z_pos_;
	float logo_alpha_;
	float duration1_;
	float duration2_;
	float duration3_;
	float logo_duration1_;
	float logo_duration2_;
	float logo_duration3_;
	float logo_duration4_;
};
