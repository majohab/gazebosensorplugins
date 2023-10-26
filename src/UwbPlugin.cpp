#include "gazebosensorplugins/UwbPlugin.hpp"

namespace gazebosensorplugins
{

    UwbPlugin::UwbPlugin() :
            ModelPlugin(),
            sequence(0)
    {
        this->updatePeriod = gazebo::common::Time(0.0);
    }

    void UwbPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        auto logger = rclcpp::get_logger("gazebo_uwb_plugin");

        if (!_sdf->HasElement("update_rate"))
        {
            RCLCPP_ERROR(logger, "UWB Plugin needs the parameter: update_rate");
        } else
        {
            RCLCPP_INFO(logger, "Update rate is set to %f", _sdf->Get<double>("update_rate"));
        }

        this->ros_node_ = gazebo_ros::Node::Get(_sdf);
        RCLCPP_INFO(ros_node_->get_logger(), "Loading Boldbot Gazebo Plugin");

        this->model = _parent;
        this->world = _parent->GetWorld();
        this->SetUpdateRate(_sdf->Get<double>("update_rate"));
        this->nlosSoftWallWidth = 0.25;
        this->tagZOffset = 0;
        this->tagId = 0;
        this->maxDBDistance = 14;
        this->stepDBDistance = 0.1;
        this->allBeaconsAreLOS = false;
        this->useParentAsReference = false;


        if (_sdf->HasElement("all_los"))
        {
            this->allBeaconsAreLOS = _sdf->Get<double>("all_los");
        }

        if (_sdf->HasElement("tag_id"))
        {
                this->tagId = _sdf->Get<double>("tag_id");
        }

        if (_sdf->HasElement("tag_z_offset"))
        {
            this->tagZOffset = _sdf->Get<double>("tag_z_offset");
        }

        if (_sdf->HasElement("nlosSoftWallWidth"))
        {
            this->nlosSoftWallWidth = _sdf->Get<double>("nlosSoftWallWidth");
        }

        if (_sdf->HasElement("tag_link"))
        {
            std::string tag_link = _sdf->Get<std::string>("tag_link");
            this->tagLink = _parent->GetLink(tag_link);

            RCLCPP_INFO(this->ros_node_->get_logger(), "Parent name: %s ChildCount: %d", _parent->GetName().c_str(), _parent->GetChildCount());
            if (this->tagLink == NULL)
            {
                std::vector<gazebo::physics::LinkPtr> links = _parent->GetLinks();
                for (int i = 0; i < links.size(); ++i)
                {
                    RCLCPP_INFO(this->ros_node_->get_logger(), "Link[%d]: %s", i, links[i]->GetName().c_str());
                }
                RCLCPP_INFO(this->ros_node_->get_logger(), "UWB Plugin Tag link Is NULL We use The Parent As Reference");
                this->useParentAsReference = true;
            }
        }

        if (_sdf->HasElement("anchor_prefix"))
        {
            this->anchorPrefix = _sdf->Get<std::string>("anchor_prefix");
        }
        else
        {
            this->anchorPrefix = "uwb_anchor";
        }

        RCLCPP_INFO(this->ros_node_->get_logger(), "GTEC UWB Plugin is running. Tag %d", this->tagId);
        RCLCPP_INFO(this->ros_node_->get_logger(), "GTEC UWB Plugin All parameters loaded");

        this->lastUpdateTime = gazebo::common::Time(0.0);

        std::string topicRanging = "/gtec/toa/ranging";

        RCLCPP_INFO(this->ros_node_->get_logger(), "GTEC UWB Plugin Ranging Publishing in %s", topicRanging.c_str());

        /*  stringStream.str("");
            stringStream.clear();
            stringStream << "/gtec/toa/anchors" << this->tagId;*/
        std::string topicAnchors = "/gtec/toa/anchors";

        RCLCPP_INFO(this->ros_node_->get_logger(), "GTEC UWB Plugin Anchors Position Publishing in %s", topicAnchors.c_str());

        this->gtecUwbPub = this->ros_node_->create_publisher<gtec_msgs::msg::Ranging>(topicRanging, 1000);
        this->gtecAnchors = this->ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(topicAnchors, 1000);

        this->firstRay = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
                             this->world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));

        this->secondRay = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
                              this->world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));

        this->updateConnection =
            gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&UwbPlugin::OnUpdate, this, _1));
    }

    void UwbPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info)
    {
        gazebo::common::Time simTime = _info.simTime;
        gazebo::common::Time elapsed = simTime - this->lastUpdateTime;
        if (elapsed >= this->updatePeriod)
        {
            this->lastUpdateTime = _info.simTime;
            ignition::math::Pose3d tagPose;
            if (!this->useParentAsReference)
            {
                tagPose = this->tagLink->WorldPose();
            }
            else
            {
                tagPose = this->model->WorldPose();
            }
            ignition::math::Vector3d posCorrectedZ(tagPose.Pos().X(), tagPose.Pos().Y(), tagPose.Pos().Z() + this->tagZOffset);
            tagPose.Set(posCorrectedZ, tagPose.Rot());
            ignition::math::Vector3d currentTagPose(tagPose.Pos());
            tf2::Quaternion q(tagPose.Rot().X(),
                             tagPose.Rot().Y(),
                             tagPose.Rot().Z(),
                             tagPose.Rot().W());
            tf2::Matrix3x3 m(q);
            double roll, pitch, currentYaw;
            m.getRPY(roll, pitch, currentYaw);
            // if (currentYaw < 0)
            // {
            //     currentYaw = 2 * M_PI + currentYaw;
            // }
            double startAngle = currentYaw;
            double currentAngle = 0;
            double arc = 3 * M_PI / 2;
            int numAnglesToTestBySide = 30;
            double incrementAngle = arc / numAnglesToTestBySide;
            int totalNumberAnglesToTest = 1 + 2 * numAnglesToTestBySide;
            double anglesToTest[totalNumberAnglesToTest];
            anglesToTest[0] = startAngle;
            for (int i = 1; i < totalNumberAnglesToTest; ++i)
            {
                double angleToTest;
                if (i % 2 == 0)
                {
                    angleToTest = startAngle - (i / 2) * incrementAngle;
                    // if (angleToTest < 0)
                    // {
                    //     angleToTest = 2 * M_PI + angleToTest;
                    // }
                }
                else
                {
                    angleToTest = startAngle + (i - (i - 1) / 2) * incrementAngle;
                    // if (angleToTest > 2 * M_PI)
                    // {
                    //     angleToTest = angleToTest - 2 * M_PI;
                    // }
                }
                anglesToTest[i] = angleToTest;
            }
            visualization_msgs::msg::MarkerArray markerArray;
            visualization_msgs::msg::MarkerArray interferencesArray;
            gazebo::physics::Model_V models = this->world->Models();
            for (gazebo::physics::Model_V::iterator iter = models.begin(); iter != models.end(); ++iter)
            {
                if ((*iter)->GetName().find(this->anchorPrefix) == 0)
                {
                    gazebo::physics::ModelPtr anchor = *iter;
                    std::string aidStr = anchor->GetName().substr(this->anchorPrefix.length());
                    int aid = std::stoi(aidStr);
                    ignition::math::Pose3d anchorPose = anchor->WorldPose();
                    LOSType losType = LOS;
                    double distance = tagPose.Pos().Distance(anchorPose.Pos());
                    double distanceAfterRebounds = 0;
                    if (!allBeaconsAreLOS)
                    {
                        //We check if a ray can reach the anchor:
                        double distanceToObstacleFromTag;
                        std::string obstacleName;
                        ignition::math::Vector3d directionToAnchor = (anchorPose.Pos() - tagPose.Pos()).Normalize();
                        this->firstRay->Reset();
                        this->firstRay->SetPoints(tagPose.Pos(), anchorPose.Pos());
                        this->firstRay->GetIntersection(distanceToObstacleFromTag, obstacleName);
                        if (obstacleName.compare("") == 0)
                        {
                            //There is no obstacle between anchor and tag, we use the LOS model
                            losType = LOS;
                            distanceAfterRebounds = distance;
                        }
                        else
                        {
                            //We use a second ray to measure the distance from anchor to tag, so we can
                            //know what is the width of the walls
                            double distanceToObstacleFromAnchor;
                            std::string otherObstacleName;
                            this->secondRay->Reset();
                            this->secondRay->SetPoints(anchorPose.Pos(), tagPose.Pos());
                            this->secondRay->GetIntersection(distanceToObstacleFromAnchor, otherObstacleName);
                            double wallWidth = distance - distanceToObstacleFromTag - distanceToObstacleFromAnchor;
                            if (wallWidth <= this->nlosSoftWallWidth && obstacleName.compare(otherObstacleName) == 0)
                            {
                                //We use NLOS - SOFT model
                                losType = NLOS_S;
                                distanceAfterRebounds = distance;
                            }
                            else
                            {
                                //We try to find a rebound to reach the anchor from the tag
                                bool end = false;
                                double maxDistance = 30;
                                double distanceToRebound = 0;
                                double distanceToFinalObstacle = 0;
                                double distanceNlosHard = 0;
                                double stepFloor = 1;
                                double startFloorDistanceCheck = 2;
                                int numStepsFloor = 6;
                                std::string finalObstacleName;
                                int indexRay = 0;
                                bool foundNlosH = false;
                                int currentFloorDistance = 0;
                                while (!end)
                                {
                                    currentAngle = anglesToTest[indexRay];
                                    double x = currentTagPose.X() + maxDistance * cos(currentAngle);
                                    double y = currentTagPose.Y() + maxDistance * sin(currentAngle);
                                    double z = currentTagPose.Z();
                                    if (currentFloorDistance>0){
                                      double tanAngleFloor = (startFloorDistanceCheck + stepFloor*(currentFloorDistance-1))/currentTagPose.Z();
                                      double angleFloor = atan(tanAngleFloor);
                                      double h = sin(angleFloor)*maxDistance;
                                      double horizontalDistance = sqrt(maxDistance*maxDistance - h*h);
                                      x = currentTagPose.X() + horizontalDistance * cos(currentAngle);
                                      y = currentTagPose.Y() + horizontalDistance * sin(currentAngle);
                                      z = -1*(h - currentTagPose.Z());
                                    }
                                    ignition::math::Vector3d rayPoint(x, y, z);
                                    this->firstRay->Reset();
                                    this->firstRay->SetPoints(currentTagPose, rayPoint);
                                    this->firstRay->GetIntersection(distanceToRebound, obstacleName);
                                    if (obstacleName.compare("") != 0)
                                    {
                                        ignition::math::Vector3d collisionPoint(currentTagPose.X() + distanceToRebound * cos(currentAngle), currentTagPose.Y() + distanceToRebound * sin(currentAngle), currentTagPose.Z());
                                        if (currentFloorDistance>0){
                                            //if (obstacleName.compare("FloorStatic)") == 0){
                                              // ROS_INFO("TOUCHED GROUND %s - Z: %f", obstacleName.c_str(), z);   
                                           //}
                                            
                                            collisionPoint.Set(currentTagPose.X() + distanceToRebound * cos(currentAngle), currentTagPose.Y() + distanceToRebound * sin(currentAngle), 0.0);
                                        }
                                        //We try to reach the anchor from here
                                        this->secondRay->Reset();
                                        this->secondRay->SetPoints(collisionPoint, anchorPose.Pos());
                                        this->secondRay->GetIntersection(distanceToFinalObstacle, finalObstacleName);
                                        if (finalObstacleName.compare("") == 0)
                                        {
                                            //We reach the anchor after one rebound
                                            distanceToFinalObstacle = anchorPose.Pos().Distance(collisionPoint);
                                            if (currentFloorDistance>0 ){
                                                  //ROS_INFO("Rebound in GROUND %s - Distance: %f", obstacleName.c_str(), distanceToFinalObstacle);   
                                            }
                                            if (distanceToRebound + distanceToFinalObstacle <= maxDBDistance)
                                            {
                                                foundNlosH = true;
                                                //We try to find the shortest rebound
                                                if (distanceNlosHard < 0.1)
                                                {
                                                    distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                }
                                                else if (distanceNlosHard > distanceToRebound + distanceToFinalObstacle)
                                                {
                                                    distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                }
                                            }
                                        }
                                    }
                                    if (indexRay < totalNumberAnglesToTest - 1)
                                    {
                                        indexRay += 1;
                                    }
                                    else
                                    {
                                      if (currentFloorDistance<numStepsFloor){
                                        currentFloorDistance+=1;
                                        indexRay= 0;
                                      } else {
                                          end = true;  
                                      }
                                    }
                                }
                                if (foundNlosH)
                                {
                                    //We use the NLOS Hard Model with distance = distanceNlosHard
                                    losType = NLOS_H;
                                    distanceAfterRebounds = distanceNlosHard;
                                }
                                else
                                {
                                    //We can not reach the anchor, no ranging.
                                    losType = NLOS;
                                }
                            }
                        }
                    }
                    else
                    {
                        //All beacons are LOS
                        losType = LOS;
                        distanceAfterRebounds = distance;
                    }
                    if ((losType == LOS || losType == NLOS_S) && distanceAfterRebounds > maxDBDistance)
                    {
                        losType = NLOS;
                    }
                    if (losType == NLOS_H && distanceAfterRebounds > maxDBDistance)
                    {
                        losType = NLOS;
                    }
                    if (losType != NLOS)
                    {
                        int indexScenario = 0;
                        if (losType == NLOS_S)
                        {
                            indexScenario = 2;
                        }
                        else if (losType == NLOS_H)
                        {
                            indexScenario = 1;
                        }
                        int indexRangingOffset = (int) round(distanceAfterRebounds / stepDBDistance);
                        double distanceAfterReboundsWithOffset = distanceAfterRebounds;
                        if (losType == LOS)
                        {
                            distanceAfterReboundsWithOffset = distanceAfterRebounds + rangingOffset[indexRangingOffset][0] / 1000.0;
                        }
                        else if (losType == NLOS_S)
                        {
                            distanceAfterReboundsWithOffset = distanceAfterRebounds + rangingOffset[indexRangingOffset][1] / 1000.0;
                        }
                        int indexRanging = (int) round(distanceAfterReboundsWithOffset / stepDBDistance);
                        std::normal_distribution<double> distributionRanging(distanceAfterReboundsWithOffset * 1000, rangingStd[indexRanging][indexScenario]);
                        std::normal_distribution<double> distributionRss(rssMean[indexRanging][indexScenario], rssStd[indexRanging][indexScenario]);
                        double rangingValue = distributionRanging(this->random_generator);
                        double powerValue = distributionRss(this->random_generator);
                        if (powerValue < minPower[indexScenario])
                        {
                            losType = NLOS;
                        }
                        
                        if (losType!=NLOS)
                        {
                            gtec_msgs::msg::Ranging ranging_msg;
                            ranging_msg.anchor_id = aid;
                            ranging_msg.tag_id = this->tagId;
                            ranging_msg.range = rangingValue;
                            ranging_msg.seq = this->sequence;
                            ranging_msg.rss = powerValue;
                            ranging_msg.error_estimation = 0.00393973;
                            this->gtecUwbPub->publish(ranging_msg);
                        }
                    }
                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "world";
                    marker.header.stamp = this->ros_node_->get_clock()->now();
                    marker.id = aid;
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.pose.position.x = anchorPose.Pos().X();
                    marker.pose.position.y = anchorPose.Pos().Y();
                    marker.pose.position.z = anchorPose.Pos().Z();
                    marker.pose.orientation.x = anchorPose.Rot().X();
                    marker.pose.orientation.y = anchorPose.Rot().Y();
                    marker.pose.orientation.z = anchorPose.Rot().Z();
                    marker.pose.orientation.w = anchorPose.Rot().W();
                    marker.scale.x = 0.2;
                    marker.scale.y = 0.2;
                    marker.scale.z = 0.5;
                    marker.color.a = 1.0;
                    if (losType == LOS)
                    {
                        marker.color.r = 0.0;
                        marker.color.g = 0.6;
                        marker.color.b = 0.0;
                    }
                    else if (losType == NLOS_S)
                    {
                        marker.color.r = 0.6;
                        marker.color.g = 0.6;
                        marker.color.b = 0.0;
                    }
                    else if (losType == NLOS_H)
                    {
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.6;
                    }
                    else if (losType == NLOS)
                    {
                        marker.color.r = 0.6;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                    }
                    markerArray.markers.push_back(marker);
                }
            }
            this->gtecAnchors->publish(markerArray);
            this->sequence++;
        }
    }

    void UwbPlugin::SetUpdateRate(double _rate)
    {
        if (_rate > 0.0)
        {
            this->updatePeriod = 1.0 / _rate;
        }
        else
        {
            this->updatePeriod = 0.0;
        }
    }

    void UwbPlugin::Reset()
    {
        RCLCPP_INFO(this->ros_node_->get_logger(), "GTEC UWB Plugin RESET");
        this->lastUpdateTime = gazebo::common::Time(0.0);
    }

    GZ_REGISTER_MODEL_PLUGIN(UwbPlugin)

}  // namespace gazebosensorplugins
