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

#ifndef B2_GEAR_JOINT_H
#define B2_GEAR_JOINT_H

#include "b2_joint.h"

/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
struct b2GearJointDef : public b2JointDef
{
	b2GearJointDef()
	{
		type = e_gearJoint;
		joint1 = nullptr;
		joint2 = nullptr;
		ratio = fixed_one;
	}

	/// The first revolute/prismatic joint attached to the gear joint.
	b2Joint* joint1;

	/// The second revolute/prismatic joint attached to the gear joint.
	b2Joint* joint2;

	/// The gear ratio.
	/// @see b2GearJoint for explanation.
	fixed ratio;
};

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning You have to manually destroy the gear joint if joint1 or joint2
/// is destroyed.
class b2GearJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(fixed inv_dt) const override;
	fixed GetReactionTorque(fixed inv_dt) const override;

	/// Get the first joint.
	b2Joint* GetJoint1() { return m_joint1; }

	/// Get the second joint.
	b2Joint* GetJoint2() { return m_joint2; }

	/// Set/Get the gear ratio.
	void SetRatio(fixed ratio);
	fixed GetRatio() const;

	/// Dump joint to dmLog
	void Dump() override;

protected:

	friend class b2Joint;
	b2GearJoint(const b2GearJointDef* data);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Joint* m_joint1;
	b2Joint* m_joint2;

	b2JointType m_typeA;
	b2JointType m_typeB;

	// Body A is connected to body C
	// Body B is connected to body D
	b2Body* m_bodyC;
	b2Body* m_bodyD;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Vec2 m_localAnchorC;
	b2Vec2 m_localAnchorD;

	b2Vec2 m_localAxisC;
	b2Vec2 m_localAxisD;

	fixed m_referenceAngleA;
	fixed m_referenceAngleB;

	fixed m_constant;
	fixed m_ratio;

	fixed m_impulse;

	// Solver temp
	int32 m_indexA, m_indexB, m_indexC, m_indexD;
	b2Vec2 m_lcA, m_lcB, m_lcC, m_lcD;
	fixed m_mA, m_mB, m_mC, m_mD;
	fixed m_iA, m_iB, m_iC, m_iD;
	b2Vec2 m_JvAC, m_JvBD;
	fixed m_JwA, m_JwB, m_JwC, m_JwD;
	fixed m_mass;
};

#endif
