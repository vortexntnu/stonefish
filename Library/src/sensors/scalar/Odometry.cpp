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
//  Odometry.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 09/11/2017.
//  Copyright (c) 2014-2025 Patryk Cieslak. All rights reserved.
//

#include "sensors/scalar/Odometry.h"

#include "entities/MovingEntity.h"
#include "sensors/Sample.h"

namespace sf
{


Odometry::Odometry(std::string uniqueName,
                   Scalar frequency,
                   int historyLength)
    : LinkSensor(uniqueName, frequency, historyLength),
      ornStdDev(Scalar(0)),
      ornNoise(Scalar(0), Scalar(1))
{
    channels.push_back(
        SensorChannel("Position X", QuantityType::LENGTH));
    channels.push_back(
        SensorChannel("Position Y", QuantityType::LENGTH));
    channels.push_back(
        SensorChannel("Position Z", QuantityType::LENGTH));

    channels.push_back(
        SensorChannel("Velocity X", QuantityType::VELOCITY));
    channels.push_back(
        SensorChannel("Velocity Y", QuantityType::VELOCITY));
    channels.push_back(
        SensorChannel("Velocity Z", QuantityType::VELOCITY));

    channels.push_back(
        SensorChannel("Orientation X", QuantityType::UNITLESS));
    channels.push_back(
        SensorChannel("Orientation Y", QuantityType::UNITLESS));
    channels.push_back(
        SensorChannel("Orientation Z", QuantityType::UNITLESS));
    channels.push_back(
        SensorChannel("Orientation W", QuantityType::UNITLESS));

    channels.push_back(
        SensorChannel(
            "Angular velocity X",
            QuantityType::ANGULAR_VELOCITY));
    channels.push_back(
        SensorChannel(
            "Angular velocity Y",
            QuantityType::ANGULAR_VELOCITY));
    channels.push_back(
        SensorChannel(
            "Angular velocity Z",
            QuantityType::ANGULAR_VELOCITY));
}


void Odometry::InternalUpdate(Scalar dt)
{
    // Calculate transformation from global to IMU frame.
    Transform odomTrans = getSensorFrame();

    Vector3 pos = odomTrans.getOrigin();

    Vector3 v =
        odomTrans.getBasis().inverse() *
        attach->getLinearVelocityInLocalPoint(
            odomTrans.getOrigin() -
            attach->getCGTransform().getOrigin());

    Quaternion orn = odomTrans.getRotation();

    Scalar angleNoise = Scalar(0);

    if (ornStdDev > Scalar(0)) {
        angleNoise = ornNoise(randomGenerator);
    }

    const Scalar angle = orn.getAngle() + angleNoise;
    orn = Quaternion(orn.getAxis(), angle);

    Vector3 av =
        odomTrans.getBasis().inverse() *
        attach->getAngularVelocity();

    Sample s{
        std::vector<Scalar>({
            pos.x(),
            pos.y(),
            pos.z(),
            v.x(),
            v.y(),
            v.z(),
            orn.x(),
            orn.y(),
            orn.z(),
            orn.w(),
            av.x(),
            av.y(),
            av.z()
        })
    };

    AddSampleToHistory(s);
}
   

void Odometry::setNoise(Scalar positionStdDev,
                        Scalar velocityStdDev,
                        Scalar angleStdDev,
                        Scalar angularVelocityStdDev)
{
    const Scalar clampedPositionStdDev =
        btClamped(
            positionStdDev,
            Scalar(0),
            Scalar(BT_LARGE_FLOAT));

    const Scalar clampedVelocityStdDev =
        btClamped(
            velocityStdDev,
            Scalar(0),
            Scalar(BT_LARGE_FLOAT));

    const Scalar clampedAngularVelocityStdDev =
        btClamped(
            angularVelocityStdDev,
            Scalar(0),
            Scalar(BT_LARGE_FLOAT));

    channels[0].setStdDev(clampedPositionStdDev);
    channels[1].setStdDev(clampedPositionStdDev);
    channels[2].setStdDev(clampedPositionStdDev);

    channels[3].setStdDev(clampedVelocityStdDev);
    channels[4].setStdDev(clampedVelocityStdDev);
    channels[5].setStdDev(clampedVelocityStdDev);

    channels[10].setStdDev(clampedAngularVelocityStdDev);
    channels[11].setStdDev(clampedAngularVelocityStdDev);
    channels[12].setStdDev(clampedAngularVelocityStdDev);

    ornStdDev =
        btClamped(
            angleStdDev,
            Scalar(0),
            Scalar(BT_LARGE_FLOAT));

    if (ornStdDev > Scalar(0)) {
        ornNoise.param(
            std::normal_distribution<Scalar>::param_type(
                Scalar(0),
                ornStdDev));
    }
}

ScalarSensorType Odometry::getScalarSensorType() const
{
    return ScalarSensorType::ODOM;
}


}
