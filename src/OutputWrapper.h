/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>



#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;


namespace IOWrap
{

class OutputWrapper : public Output3DWrapper
{
public:
        int count;
        ros::Publisher pose_pub;
        ros::Publisher pointcloud_pub;
        const int queue_size = 1000;

        inline OutputWrapper(ros::NodeHandle nh)
        {
            int count = 0;
            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", queue_size);
            pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", queue_size);

            // printf("OUT: Created OutputWrapper\n");
        }

        virtual ~OutputWrapper()
        {
            // printf("OUT: Destroyed OutputWrapper\n");
        }

        virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override
        {
            // printf("OUT: got graph with %d edges\n", (int)connectivity.size());

            // int maxWrite = 5;

            // for(const std::pair<uint64_t,Eigen::Vector2i> &p : connectivity)
            // {
            //     int idHost = p.first>>32;
            //     int idTarget = p.first & ((uint64_t)0xFFFFFFFF);
            //     printf("OUT: Example Edge %d -> %d has %d active and %d marg residuals\n", idHost, idTarget, p.second[0], p.second[1]);
            //     maxWrite--;
            //     if(maxWrite==0) break;
            // }
        }



        virtual void publishKeyframes(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override
        {
            // Follows discussion at https://github.com/JakobEngel/dso/issues/55

            float fx = HCalib->fxl();
            float fy = HCalib->fyl();
            float cx = HCalib->cxl();
            float cy = HCalib->cyl();
            float fxi = 1/fx;
            float fyi = 1/fy;
            float cxi = -cx / fx;
            float cyi = -cy / fy;

            // Open stream to write in file "points.ply"
            // std::ofstream output_points;
            // output_points.open("points.ply", std::ios_base::app);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZ>);


            for(FrameHessian* f : frames)
            {
                // printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
                //        f->frameID,
                //        final ? "final" : "non-final",
                //        f->shell->incoming_id,
                //        f->shell->timestamp,
                //        (int)f->pointHessians.size(), (int)f->pointHessiansMarginalized.size(), (int)f->immaturePoints.size());
                // std::cout << f->shell->camToWorld.matrix3x4() << "\n";

                // int maxWrite = 5;
                // for(PointHessian* p : f->pointHessians)
                // {
                //     printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. %f, %d inlier-residuals\n",
                //            p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian), p->numGoodResiduals );
                //     maxWrite--;
                //     if(maxWrite==0) break;
                // }

                auto const & m =  f->shell->camToWorld.matrix3x4();
                auto const & points = f->pointHessiansMarginalized;

                for (auto const * p : points) {
                    float depth = 1.0f / p->idepth;
                    auto const x = (p->u * fxi + cxi) * depth;
                    auto const y = (p->v * fyi + cyi) * depth;
                    auto const z = depth * (1 + 2*fxi);

                    Eigen::Vector4d camPoint(x, y, z, 1.f);
                    Eigen::Vector3d worldPoint = m * camPoint;
                    
                    // output_points << worldPoint.transpose() << std::endl;
                    auto point = worldPoint.transpose();
                    pcl::PointXYZ pt(point[0], point[1], point[2]);
                    pcl_points->push_back(pt);
                }
            }

            sensor_msgs::PointCloud2 pointcloud;            
            pcl::PCLPointCloud2 pcl_pc2;
            pcl::toPCLPointCloud2(*pcl_points, pcl_pc2);    // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
            pcl_conversions::fromPCL(pcl_pc2, pointcloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
            pointcloud.header.frame_id = "world";  
            pointcloud.header.stamp = ros::Time::now(); 
            pointcloud_pub.publish(pointcloud);

        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
        {
            // printf("OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
            //        frame->incoming_id,
            //        frame->timestamp,
            //        frame->id);

            auto timestamp = frame->timestamp;
            auto translation = frame->camToWorld.translation().cast<float>();
            auto quaternion = frame->camToWorld.unit_quaternion().coeffs().cast<float>();

            std::stringstream ss;
            ss << std::setprecision(16) << timestamp << ' ' << std::setprecision(8);
            ss << translation[0] << ' ' << translation[1] << ' ' << translation[2] << ' ';
            ss << quaternion[0] << ' ' << quaternion[1] << ' ' << quaternion[2] << ' ' << quaternion[3] << ' ';
            std::cout << ss.str() << std::endl;

            ros::Time ros_time;
            geometry_msgs::PoseStamped odom_pose;
            odom_pose.header.stamp = ros_time.fromSec(timestamp);
            odom_pose.header.frame_id = "odom";
            // tf::poseTFToMsg(last_transform.inverse()*new_transform, pose_inc_cov.pose.pose);
            odom_pose.pose.position.x = translation[0];
            odom_pose.pose.position.y = translation[1];
            odom_pose.pose.position.z = translation[2];

            odom_pose.pose.orientation.x = quaternion[0];
            odom_pose.pose.orientation.y = quaternion[1];
            odom_pose.pose.orientation.z = quaternion[2];
            odom_pose.pose.orientation.w = quaternion[3];

            pose_pub.publish(odom_pose);

            ++count;           
        }


        virtual void pushLiveFrame(FrameHessian* image) override
        {
            // can be used to get the raw image / intensity pyramid.
        }

        virtual void pushDepthImage(MinimalImageB3* image) override
        {
            // can be used to get the raw image with depth overlay.
        }
        
        virtual bool needPushDepthImage() override
        {
            return false;
        }

        virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF ) override
        {
            // printf("OUT: Predicted depth for KF %d (id %d, time %f, internal frame-ID %d). CameraToWorld:\n",
            //        KF->frameID,
            //        KF->shell->incoming_id,
            //        KF->shell->timestamp,
            //        KF->shell->id);
            // std::cout << KF->shell->camToWorld.matrix3x4() << "\n";

            // int maxWrite = 5;
            // for(int y=0;y<image->h;y++)
            // {
            //     for(int x=0;x<image->w;x++)
            //     {
            //         if(image->at(x,y) <= 0) continue;

            //         printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x,y,image->at(x,y));
            //         maxWrite--;
            //         if(maxWrite==0) break;
            //     }
            //     if(maxWrite==0) break;
            // }
        }


};



}



}
