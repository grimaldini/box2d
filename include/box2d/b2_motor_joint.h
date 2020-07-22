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

#ifndef B2_MOTOR_JOINT_H
#define B2_MOTOR_JOINT_H

#include "b2_joint.h"

/// Motor joint definition.
struct b2MotorJointDef : public b2JointDef
{
	b2MotorJointDef()
	{
		type = e_motorJoint;
		linearOffset.SetZero();
		angularOffset = fixed_zero;
		maxForce = fixed_one;
		maxTorque = fixed_one;
		correctionFactor = fixed(3, 10);
	}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(b2Body* bodyA, b2Body* bodyB);

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	b2Vec2 linearOffset;

	/// The bodyB angle minus bodyA angle in radians.
	fixed angularOffset;
	
	/// The maximum motor force in N.
	fixed maxForce;

	/// The maximum motor torque in N-m.
	fixed maxTorque;

	/// Position correction factor in the range [0,1].
	fixed correctionFactor;
};

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
class b2MotorJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(fixed inv_dt) const override;
	fixed GetReactionTorque(fixed inv_dt) const override;

	/// Set/get the target linear offset, in frame A, in meters.
	void SetLinearOffset(const b2Vec2& linearOffset);
	const b2Vec2& GetLinearOffset() const;

	/// Set/get the target angular offset, in radians.
	void SetAngularOffset(fixed angularOffset);
	fixed GetAngularOffset() const;

	/// Set the maximum friction force in N.
	void SetMaxForce(fixed force);

	/// Get the maximum friction force in N.
	fixed GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(fixed torque);

	/// Get the maximum friction torque in N*m.
	fixed GetMaxTorque() const;

	/// Set the position correction factor in the range [0,1].
	void SetCorrectionFactor(fixed factor);

	/// Get the position correction factor in the range [0,1].
	fixed GetCorrectionFactor() const;

	/// Dump to b2Log
	void Dump() override;

protected:

	friend class b2Joint;

	b2MotorJoint(const b2MotorJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	// Solver shared
	b2Vec2 m_linearOffset;
	fixed m_angularOffset;
	b2Vec2 m_linearImpulse;
	fixed m_angularImpulse;
	fixed m_maxForce;
	fixed m_maxTorque;
	fixed m_correctionFactor;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	b2Vec2 m_linearError;
	fixed m_angularError;
	fixed m_invMassA;
	fixed m_invMassB;
	fixed m_invIA;
	fixed m_invIB;
	b2Mat22 m_linearMass;
	fixed m_angularMass;
};

#endif
