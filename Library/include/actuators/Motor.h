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
//  Motor.h
//  Stonefish
//
//  Created by Patryk Cieslak on 15/09/2015.
//  Copyright (c) 2015-2024 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_Motor__
#define __Stonefish_Motor__

#include "actuators/JointActuator.h"

namespace sf
{
    //! A class representing a motor.
    class Motor : public JointActuator
    {
    public:
        //! A constructor.
        /*!
         \param uniqueName a name for the motor
         */
        Motor(std::string uniqueName);
        
        //! A method used to update the internal state of the actuator.
        /*!
         \param dt the time step of the simulation [s]
         */
        virtual void Update(Scalar dt);

        //! A method to set the motor torque.
        /*!
         \param tau a value of the motor torque [Nm]
         */
        virtual void setIntensity(Scalar tau);

        //! A method used to set the torque limits.
        /*!
         \param lower value of the lower limit
         \param upper value of the upper limit
        */
        void setTorqueLimits(Scalar lower, Scalar upper);
                
        //! A method returning the torque generated by the motor.
        virtual Scalar getTorque() const;
        
        //! A method returning the angular position of the motor.
        virtual Scalar getAngle() const;
        
        //! A method returning the angular velocity of the motor.
        virtual Scalar getAngularVelocity() const;
        
        //! A method returning the type of the actuator.
        ActuatorType getType() const;
        
    protected:
        Scalar torque;
        std::pair<Scalar, Scalar> limits;

    private:
        void WatchdogTimeout() override;
    };
}

#endif
