//
// Created by tammo on 24.10.18.
//


#ifndef IAI_IMAGE_LOGGING_MOTION_DETECTOR_H
#define IAI_IMAGE_LOGGING_MOTION_DETECTOR_H

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
class MotionDetector
{
private:
    double threshold;

public:
    MotionDetector() : threshold(1.0) {};

    bool detectMotion(const tf::tfMessageConstPtr& transforms, std::string reference, std::string camera){
        geometry_msgs::Transform ref_point, cam_point;
        double ref_euclid, cam_euclid;

        for (int i = 0; i < transforms->transforms.size(); i++){
            if (transforms->transforms.at(i).child_frame_id.find(reference) != std::string::npos){
                ROS_WARN_STREAM("found ref");
                ref_point = transforms->transforms.at(i).transform;
            }
            if (transforms->transforms.at(i).child_frame_id.find(camera) != std::string::npos){
                ROS_WARN_STREAM("found cam");

                cam_point = transforms->transforms.at(i).transform;
            }
        }

        ref_euclid = sqrt(pow(ref_point.translation.x,2.0)+ pow(ref_point.translation.y,2.0)
                + pow(ref_point.translation.z,2.0));
        cam_euclid = sqrt(pow(cam_point.translation.x,2.0)+ pow(cam_point.translation.y,2.0)
                          + pow(cam_point.translation.z,2.0));
        ROS_WARN_STREAM("diff motion " << ref_euclid << " and " << cam_euclid);

        return (ref_euclid < cam_euclid - threshold || ref_euclid > cam_euclid + threshold);
    }


};
#endif  // IAI_IMAGE_LOGGING_MOTION_DETECTOR_H
