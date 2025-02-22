/*    
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  LiDAR360.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 1/08/2018.
//  Copyright (c) 2018-2021 Patryk Cieslak. All rights reserved.
/* Author: Michele Grimaldi
* Email: michelegrmld@gmail.com
* Description: 360 LiDAR sensor 
*/


#include "sensors/scalar/LiDAR360.h"

#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "utils/UnitSystem.h"
#include "sensors/Sample.h"
#include "graphics/OpenGLPipeline.h"

#include <iostream>
#include <ostream>


namespace sf
{

LiDAR360::LiDAR360(std::string uniqueName, unsigned int resolution, unsigned int layers, Scalar frequency, int historyLength) : LinkSensor(uniqueName, frequency, historyLength)
{
    ang_range_hori_ = UnitSystem::Angle(true, 360);
    ang_range_vert_ = UnitSystem::Angle(true, 42.4);

    resolution_ = resolution;
    layers_ = layers;

    channels.push_back(SensorChannel("Distance", QuantityType::LENGTH));
    channels.back().rangeMin = Scalar(0);
    channels.back().rangeMax = BT_LARGE_FLOAT;
    
    // Precompute horizontal angles (size: resolution_+1)
    for (unsigned int i = 0; i < resolution_; ++i)
    {
        // Map [0, resolution_] to [-180°, +180°] (in radians)
        angles_hori_.push_back(i / (Scalar)resolution_ * ang_range_hori_ - Scalar(0.5) * ang_range_hori_);
    }
    
    // Precompute vertical angles (size: layers_+1)
    for (unsigned int j = 0; j < layers_; ++j)
    {
        // Map [0, layers_] to [-21.2°, +21.2°]
        angles_vert_.push_back(j / (Scalar)layers_ * ang_range_vert_ - Scalar(0.5) * ang_range_vert_);
    }
    std::cout << "LiDAR360: resolution: " << resolution_ << ", layers: " << layers_ << std::endl;
    std::cout << "LiDAR360: horizontal FOV: " << ang_range_hori_ << ", vertical FOV: " << ang_range_vert_ << std::endl;
    std::cout << "LiDAR360: horizontal steps: " << angles_hori_.size() << ", vertical steps: " << angles_vert_.size() << std::endl;
    std::cout << "LiDAR360: range: [" << channels[0].rangeMin << ", " << channels[0].rangeMax << "]" << std::endl;
    std::cout << "vertical angles: ";
    for (unsigned int j = 0; j < layers_; ++j)
        std::cout << angles_vert_[j] << " ";
    std::cout << std::endl;

    
    distances_ = std::vector<Scalar>(resolution_ * layers_, Scalar(0));
}

void LiDAR360::InternalUpdate(Scalar dt)
{
    // Get sensor frame in world
    Transform mbTrans = getSensorFrame();

     // Loop over vertical layers and horizontal steps.
    for (unsigned int j = 0; j < layers_; ++j)
    {
        // Get the vertical angle for layer j.
        Scalar vAngle = angles_vert_[j];
        for (unsigned int i = 0; i < resolution_; ++i)
        {
            // Get the horizontal angle for step i.
            Scalar hAngle = angles_hori_[i];

            // Calculate direction based on current angle step
            Vector3 dir = mbTrans.getBasis().getColumn(0) * btCos(hAngle) + mbTrans.getBasis().getColumn(1) * btSin(hAngle) + mbTrans.getBasis().getColumn(2) * btSin(vAngle);

            Vector3 from = mbTrans.getOrigin() + dir * channels[0].rangeMin;
            Vector3 to = mbTrans.getOrigin() + dir * channels[0].rangeMax;

            btCollisionWorld::ClosestRayResultCallback closest(from, to);
            closest.m_collisionFilterGroup = MASK_DYNAMIC;
            closest.m_collisionFilterMask = MASK_STATIC | MASK_DYNAMIC | MASK_ANIMATED_COLLIDING;
            SimulationApp::getApp()->getSimulationManager()->getDynamicsWorld()->rayTest(from, to, closest);

            unsigned int index = j * (resolution_) + i;
            if (closest.hasHit())
            {
                Vector3 p = from.lerp(to, closest.m_closestHitFraction);
                distances_[index] = (p - mbTrans.getOrigin()).length();
            }
            else
            {
                distances_[index] = 0;
            }
        }
    }

   Sample s(resolution_ * layers_, distances_.data());
   AddSampleToHistory(s);
}

std::vector<Renderable> LiDAR360::Render()
{
    std::vector<Renderable> items = Sensor::Render();

    
    if (isRenderable())
    {
        Renderable item;
        item.type = RenderableType::SENSOR_LINES;
        // Set the model matrix to the sensor's transformation
        item.model = glMatrixFromTransform(getSensorFrame());
        
        // total rays per vertical layer:
        unsigned int raysPerLayer = resolution_;
        
        // Loop over each vertical layer
        for (unsigned int j = 0; j < layers_; ++j)
        {
            // Get the vertical angle for this layer.
            Scalar vAngle = angles_vert_[j];
            
            // Loop over each horizontal beam in this layer.
            for (unsigned int i = 0; i < resolution_; ++i)
            {
                // Compute the horizontal angle for this beam with the current offset.
                Scalar hAngle = angles_hori_[i];
                
                // Calculate the ray direction using spherical coordinates.
                // Here, we assume the sensor’s local coordinate system:
                // x = cos(vAngle) * cos(hAngle)
                // y = cos(vAngle) * sin(hAngle)
                // z = sin(vAngle)
                Vector3 dir = Vector3(
                    btCos(vAngle) * btCos(hAngle),
                    btCos(vAngle) * btSin(hAngle),
                    btSin(vAngle)
                );
                
                // Compute the index into the distances vector.
                unsigned int index = j * raysPerLayer + i;
                
                // Draw a line from the sensor origin (0,0,0) to the measured point.
                item.points.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
                item.points.push_back(glm::vec3(
                    dir.x() * distances_[index],
                    dir.y() * distances_[index],
                    dir.z() * distances_[index]
                ));
            }
        }
        
        items.push_back(item);
    }
    
    return items;
}

void LiDAR360::setRange(Scalar rangeMin, Scalar rangeMax)
{
    btClamp(rangeMin, Scalar(0), Scalar(BT_LARGE_FLOAT));
    btClamp(rangeMax, Scalar(0), Scalar(BT_LARGE_FLOAT)); 

   
    channels.back().rangeMin = rangeMin;
    channels.back().rangeMax = rangeMax;
    max_range_ = rangeMax;
    min_range_ = rangeMin;

}

void LiDAR360::setNoise(Scalar rangeStdDev)
{
    btClamp(rangeStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT));
    channels[0].setStdDev(rangeStdDev);
}

ScalarSensorType LiDAR360::getScalarSensorType() const
{
    return ScalarSensorType::LiDAR360;
}

Scalar LiDAR360::getAngleRangeHori() const
{
    return ang_range_hori_;
}

Scalar LiDAR360::getAngleRangeVert() const
{
    return ang_range_vert_;
}



}
