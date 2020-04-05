#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "ofApp.h"
#include <cmath>
#include <string>
#include "ofxEasing.h"

enum class ToioMoveType { G1, G2, G3, G4, G5, G6, B1, B2, B3, B4, B5, B6};

class ToioController{
public:
    ToioController(const std::string ns){
        if(ros::master::check()){
            printf("rosmaster exist\n");
            nh_ptr_ = std::make_shared<ros::NodeHandle>();
            string_pub_ = nh_ptr_->advertise<std_msgs::String>(ns + "/toio_command", 10);
            odom_sub_ =  nh_ptr_->subscribe(ns + "/odom", 10, &ToioController::odomCallback, this);
            spinner_ptr_ = std::make_shared<ros::AsyncSpinner>(1);
            spinner_ptr_->start();
        }
        else printf("rosmaster not exist\n");

        circle_image_.load("circle2.tiff");
    }

    void setup(ToioMoveType type){
        if(!nh_ptr_){
            return;
        }

        if(type == ToioMoveType::G1){
            std_msgs::String msg;
            msg.data = "G1";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::G2){
            std_msgs::String msg;
            msg.data = "G2";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::G3){
            std_msgs::String msg;
            msg.data = "G3";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::G4){
            std_msgs::String msg;
            msg.data = "G4";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::B1){
            std_msgs::String msg;
            msg.data = "B1";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::B2){
            std_msgs::String msg;
            msg.data = "B2";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::B3){
            std_msgs::String msg;
            msg.data = "B3";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::B4){
            std_msgs::String msg;
            msg.data = "B4";
            string_pub_.publish(msg);
        }
    }

    void update(){
        if(last_odom_ptr_){
            float scale = 400 / 0.28;
            int pos_x = 400 + scale *last_odom_ptr_->pose.pose.position.y;
            int pos_y = 400 + scale *last_odom_ptr_->pose.pose.position.x;
            // int vel_x = scale * sin(tf::getYaw( last_odom_ptr_->pose.pose.orientation)) * last_odom_ptr_->twist.twist.linear.x; 
            // int vel_y = scale * cos(tf::getYaw( last_odom_ptr_->pose.pose.orientation)) * last_odom_ptr_->twist.twist.linear.x; 
            int vel_x = scale * last_odom_ptr_->twist.twist.linear.y; 
            int vel_y = scale * last_odom_ptr_->twist.twist.linear.x; 
            float dt = (ros::Time::now() - last_odom_ptr_->header.stamp).toSec();
            // printf("%i %i %i %i %f %f\n", pos_x, pos_y, vel_x, vel_y ,dt, tf::getYaw( last_odom_ptr_->pose.pose.orientation));

            circle_x_ = pos_x + vel_x * dt;
            circle_y_ = pos_y + vel_y * dt;
            circle_gray_ = 1.0;
        }
    }

    void draw(){
        // ofEnableAlphaBlending();
        // ofEnableBlen/ddMode(OF_BLENDMODE_SCREEN);
        // ofSetColor((int)(250*circle_gray_), (int)(250*circle_gray_), (int)(250*circle_gray_), 250);
        ofSetColor(255, 255, 255);
        ofCircle(circle_x_, circle_y_, 60); 
        float image_w = 160;
        float image_h = 160;
	    // circle_image_.draw(circle_x_ - image_w/2, circle_y_ - image_h/2, image_w, image_h);
        // ofDisableAlphaBlending();
    }

    void odomCallback(const nav_msgs::Odometry& msg){
        last_odom_ptr_ = std::make_shared<nav_msgs::Odometry>(msg);
        // printf("%f %f\n", msg.pose.pose.position.x, msg.pose.pose.position.y);
    }

    std::shared_ptr<ros::NodeHandle> nh_ptr_;
    std::shared_ptr<ros::AsyncSpinner> spinner_ptr_;
    ros::Publisher string_pub_;
    ros::Subscriber odom_sub_;
    std::shared_ptr<nav_msgs::Odometry> last_odom_ptr_;
    ofImage circle_image_;
    int circle_x_;
    int circle_y_;
    float circle_gray_;
};