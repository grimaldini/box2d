// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_WHEEL_JOINT_H
#define B2_WHEEL_JOINT_H

#include "b2_joint.h"

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct b2WheelJointDef : public b2JointDef
{
	b2WheelJointDef()
	{
		type = e_wheelJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		localAxisA.Set(fixed_one, fixed_zero);
		enableLimit = false;
		lowerTranslation = fixed_zero;
		upperTranslation = fixed_zero;
		enableMotor = false;
		maxMotorTorque = fixed_zero;
		motorSpeed = fixed_zero;
		stiffness = fixed_zero;
		damping = fixed_zero;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The local translation axis in bodyA.
	b2Vec2 localAxisA;

	/// Enable/disable the joint limit.
	bool enableLimit;

	/// The lower translation limit, usually in meters.
	fixed lowerTranslation;

	/// The upper translation limit, usually in meters.
	fixed upperTranslation;

	/// Enable/disable the joint motor.
	bool enableMotor;

	/// The maximum motor torque, usually in N-m.
	fixed maxMotorTorque;

	/// The desired motor speed in radians per second.
	fixed motorSpeed;

	/// Suspension stiffness. Typically in units N/m.
	fixed stiffness;

	/// Suspension damping. Typically in units of N*s/m.
	fixed damping;
};

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper. The spring/damper is
/// initialized upon creation. This joint is designed for vehicle suspensions.
class b2WheelJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(fixed inv_dt) const override;
	fixed GetReactionTorque(fixed inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	const b2Vec2& GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the current joint translation, usually in meters.
	fixed GetJointTranslation() const;

	/// Get the current joint linear speed, usually in meters per second.
	fixed GetJointLinearSpeed() const;

	/// Get the current joint angle in radians.
	fixed GetJointAngle() const;

	/// Get the current joint angular speed in radians per second.
	fixed GetJointAngularSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	/// Enable/disable the joint translation limit.
	void EnableLimit(bool flag);

	/// Get the lower joint translation limit, usually in meters.
	fixed GetLowerLimit() const;

	/// Get the upper joint translation limit, usually in meters.
	fixed GetUpperLimit() const;

	/// Set the joint translation limits, usually in meters.
	void SetLimits(fixed lower, fixed upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed, usually in radians per second.
	void SetMotorSpeed(fixed speed);

	/// Get the motor speed, usually in radians per second.
	fixed GetMotorSpeed() const;

	/// Set/Get the maximum motor force, usually in N-m.
	void SetMaxMotorTorque(fixed torque);
	fixed GetMaxMotorTorque() const;

	/// Get the current motor torque given the inverse time step, usually in N-m.
	fixed GetMotorTorque(fixed inv_dt) const;

	/// Access spring stiffness
	void SetStiffness(fixed stiffness);
	fixed GetStiffness() const;

	/// Access damping
	void SetDamping(fixed damping);
	fixed GetDamping() const;

	/// Dump to b2Log
	void Dump() override;

	///
	void Draw(b2Draw* draw) const override;

protected:

	friend class b2Joint;
	b2WheelJoint(const b2WheelJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Vec2 m_localXAxisA;
	b2Vec2 m_localYAxisA;

	fixed m_impulse;
	fixed m_motorImpulse;
	fixed m_springImpulse;

	fixed m_lowerImpulse;
	fixed m_upperImpulse;
	fixed m_translation;
	fixed m_lowerTranslation;
	fixed m_upperTranslation;

	fixed m_maxMotorTorque;
	fixed m_motorSpeed;

	bool m_enableLimit;
	bool m_enableMotor;

	fixed m_stiffness;
	fixed m_damping;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	fixed m_invMassA;
	fixed m_invMassB;
	fixed m_invIA;
	fixed m_invIB;

	b2Vec2 m_ax, m_ay;
	fixed m_sAx, m_sBx;
	fixed m_sAy, m_sBy;

	fixed m_mass;
	fixed m_motorMass;
	fixed m_axialMass;
	fixed m_springMass;

	fixed m_bias;
	fixed m_gamma;

};

inline fixed b2WheelJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

inline fixed b2WheelJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

#endif
