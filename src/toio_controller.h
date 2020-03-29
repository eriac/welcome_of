#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "ofApp.h"
#include <cmath>
#include <string>
#include "ofxEasing.h"

enum class ToioMoveType { G0, G1, G2, G3, G4 };

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
    }

    void setup(ToioMoveType type){
        if(!nh_ptr_){
            return;
        }

        if(type == ToioMoveType::G0){
            std_msgs::String msg;
            msg.data = "G0";
            string_pub_.publish(msg);
        }
        else if(type == ToioMoveType::G1){
            std_msgs::String msg;
            msg.data = "G1";
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
            circle_gray_ = 0.0;
        }
    }

    void draw(){
        ofSetColor((int)(255*circle_gray_), (int)(255*circle_gray_), (int)(255*circle_gray_));
        ofCircle(circle_x_, circle_y_, 30); 
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
    int circle_x_;
    int circle_y_;
    float circle_gray_;
};