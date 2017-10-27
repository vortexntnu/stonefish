//
//  FeatherstoneEntity.h
//  Stonefish
//
//  Created by Patryk Cieslak on 8/20/13.
//  Copyright(c) 2013-2017 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_FeatherstoneEntity__
#define __Stonefish_FeatherstoneEntity__

#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointFeedback.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include "Entity.h"
#include "SolidEntity.h"
#include "StaticEntity.h"

struct FeatherstoneLink
{
    FeatherstoneLink(SolidEntity* s, const btTransform& t) : solid(s), trans(t) {}
    
    SolidEntity* solid;
    btTransform trans;
};

struct FeatherstoneJoint
{
    FeatherstoneJoint(btMultibodyLink::eFeatherstoneJointType t, unsigned int p, unsigned int c)
                        : type(t), feedback(NULL), limit(NULL), motor(NULL), parent(p), child(c), sigDamping(btScalar(0)), velDamping(btScalar(0))  {}
    
    btMultibodyLink::eFeatherstoneJointType type;
    btMultiBodyJointFeedback* feedback;
    btMultiBodyJointLimitConstraint* limit;
    btMultiBodyJointMotor* motor;
    
    unsigned int parent;
    unsigned int child;
    btScalar sigDamping;
    btScalar velDamping;
};

//! Featherstone multi-body dynamics class.
/*!
	Class implements simplified creation of multi-body trees, using Roy Featherstone algorithm. 
 */
class FeatherstoneEntity : public Entity
{
public:
    FeatherstoneEntity(std::string uniqueName, unsigned int totalNumOfLinks, SolidEntity* baseSolid, btMultiBodyDynamicsWorld* world, bool fixedBase = false);
    virtual ~FeatherstoneEntity();
    
    //Multibody definition
    void AddLink(SolidEntity* solid, const btTransform& transform, btMultiBodyDynamicsWorld* world);
    int AddRevoluteJoint(unsigned int parent, unsigned int child, const btVector3& pivot, const btVector3& axis, bool collisionBetweenJointLinks = false);
    int AddPrismaticJoint(unsigned int parent, unsigned int child, const btVector3& axis, bool collisionBetweenJointLinks = false);
	int AddFixedJoint(unsigned int parent, unsigned int child);
    void AddJointMotor(unsigned int index);
    void AddJointLimit(unsigned int index, btScalar lower, btScalar upper);
	
    //Multibody control
    void MotorPositionSetpoint(unsigned int index, btScalar pos, btScalar kp);
    void MotorVelocitySetpoint(unsigned int index, btScalar vel, btScalar kd);
    void DriveJoint(unsigned int index, btScalar forceTorque);
    void ApplyGravity(const btVector3& g);
    void ApplyDamping();
    void AddLinkForce(unsigned int index, const btVector3& F);
    void AddLinkTorque(unsigned int index, const btVector3& tau);
    
    //Joints
    void setJointIC(unsigned int index, btScalar position, btScalar velocity);
    void setJointDamping(unsigned int  index, btScalar constantFactor, btScalar viscousFactor);
    void getJointPosition(unsigned int index, btScalar& position, btMultibodyLink::eFeatherstoneJointType& jointType);
    void getJointVelocity(unsigned int index, btScalar& velocity, btMultibodyLink::eFeatherstoneJointType& jointType);
    btScalar getJointTorque(unsigned int index); //Only shows sum of manually applied torques
    
    /*
     * Both vectors are in the CoG frame of the child link. 
     * The force is equal to the sum of reaction forces acting on the CoG.
     * The torque is calculated as a cross product of a vector from CoG to joint pivot and the force defined above. 
     * The force acting on every link causes a reaction force and a torque on this link. 
     * If the rection torque acts around the axis of the joint, then this torque does not transfer directly to previous joints (only force is transferred)
    */
    void getJointFeedback(unsigned int index, btVector3& force, btVector3& torque); 
    
	//Links
	void setBaseTransform(const btTransform& trans);
    void setBaseRenderable(bool render);
    FeatherstoneLink getLink(unsigned int index);
    btTransform getLinkTransform(unsigned int index);
    btVector3 getLinkLinearVelocity(unsigned int index);
    btVector3 getLinkAngularVelocity(unsigned int index);
    unsigned int getNumOfJoints();
    unsigned int getNumOfLinks();
	btMultiBody* getMultiBody();
    void setSelfCollision(bool enabled);
    
    //Common
    void AddToDynamicsWorld(btMultiBodyDynamicsWorld* world);
    void AddToDynamicsWorld(btMultiBodyDynamicsWorld* world, const btTransform& worldTransform);
    std::vector<Renderable> Render();
    void GetAABB(btVector3& min, btVector3& max);
    EntityType getType();
    
private:
    btMultiBody* multiBody;
    std::vector<FeatherstoneLink> links;
    std::vector<FeatherstoneJoint> joints;
	bool baseRenderable;
};

#endif