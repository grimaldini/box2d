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

#include "box2d/b2_body.h"
#include "box2d/b2_draw.h"
#include "box2d/b2_wheel_joint.h"
#include "box2d/b2_time_step.h"

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

void b2WheelJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	localAxisA = bodyA->GetLocalVector(axis);
}

b2WheelJoint::b2WheelJoint(const b2WheelJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_localXAxisA = def->localAxisA;
	m_localYAxisA = b2Cross(fixed_one, m_localXAxisA);

	m_mass = fixed_zero;
	m_impulse = fixed_zero;
	m_motorMass = fixed_zero;
	m_motorImpulse = fixed_zero;
	m_springMass = fixed_zero;
	m_springImpulse = fixed_zero;

	m_axialMass = fixed_zero;
	m_lowerImpulse = fixed_zero;
	m_upperImpulse = fixed_zero;
	m_lowerTranslation = def->lowerTranslation;
	m_upperTranslation = def->upperTranslation;
	m_enableLimit = def->enableLimit;

	m_maxMotorTorque = def->maxMotorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableMotor = def->enableMotor;

	m_bias = fixed_zero;
	m_gamma = fixed_zero;

	m_ax.SetZero();
	m_ay.SetZero();

	m_stiffness = def->stiffness;
	m_damping = def->damping;
}

void b2WheelJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	fixed mA = m_invMassA, mB = m_invMassB;
	fixed iA = m_invIA, iB = m_invIB;

	b2Vec2 cA = data.positions[m_indexA].c;
	fixed aA = data.positions[m_indexA].a;
	b2Vec2 vA = data.velocities[m_indexA].v;
	fixed wA = data.velocities[m_indexA].w;

	b2Vec2 cB = data.positions[m_indexB].c;
	fixed aB = data.positions[m_indexB].a;
	b2Vec2 vB = data.velocities[m_indexB].v;
	fixed wB = data.velocities[m_indexB].w;

	b2Rot qA(aA), qB(aB);

	// Compute the effective masses.
	b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	b2Vec2 d = cB + rB - cA - rA;

	// Point to line constraint
	{
		m_ay = b2Mul(qA, m_localYAxisA);
		m_sAy = b2Cross(d + rA, m_ay);
		m_sBy = b2Cross(rB, m_ay);

		m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

		if (m_mass > fixed_zero)
		{
			m_mass = fixed_one / m_mass;
		}
	}

	// Spring constraint
	m_ax = b2Mul(qA, m_localXAxisA);
	m_sAx = b2Cross(d + rA, m_ax);
	m_sBx = b2Cross(rB, m_ax);

	const fixed invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;
	if (invMass > fixed_zero)
	{
		m_axialMass = fixed_one / invMass;
	}
	else
	{
		m_axialMass = fixed_zero;
	}

	m_springMass = fixed_zero;
	m_bias = fixed_zero;
	m_gamma = fixed_zero;

	if (m_stiffness > fixed_zero && invMass > fixed_zero)
	{
		m_springMass = fixed_one / invMass;

		fixed C = b2Dot(d, m_ax);

		// magic formulas
		fixed h = data.step.dt;
		m_gamma = h * (m_damping + h * m_stiffness);
		if (m_gamma > fixed_zero)
		{
			m_gamma = fixed_one / m_gamma;
		}

		m_bias = C * h * m_stiffness * m_gamma;

		m_springMass = invMass + m_gamma;
		if (m_springMass > fixed_zero)
		{
			m_springMass = fixed_one / m_springMass;
		}
	}
	else
	{
		m_springImpulse = fixed_zero;
	}

	if (m_enableLimit)
	{
		m_translation = b2Dot(m_ax, d);
	}
	else
	{
		m_lowerImpulse = fixed_zero;
		m_upperImpulse = fixed_zero;
	}

	if (m_enableMotor)
	{
		m_motorMass = iA + iB;
		if (m_motorMass > fixed_zero)
		{
			m_motorMass = fixed_one / m_motorMass;
		}
	}
	else
	{
		m_motorMass = fixed_zero;
		m_motorImpulse = fixed_zero;
	}

	if (data.step.warmStarting)
	{
		// Account for variable time step.
		m_impulse *= data.step.dtRatio;
		m_springImpulse *= data.step.dtRatio;
		m_motorImpulse *= data.step.dtRatio;

		fixed axialImpulse = m_springImpulse + m_lowerImpulse - m_upperImpulse;
		b2Vec2 P = m_impulse * m_ay + axialImpulse * m_ax;
		fixed LA = m_impulse * m_sAy + axialImpulse * m_sAx + m_motorImpulse;
		fixed LB = m_impulse * m_sBy + axialImpulse * m_sBx + m_motorImpulse;

		vA -= m_invMassA * P;
		wA -= m_invIA * LA;

		vB += m_invMassB * P;
		wB += m_invIB * LB;
	}
	else
	{
		m_impulse = fixed_zero;
		m_springImpulse = fixed_zero;
		m_motorImpulse = fixed_zero;
		m_lowerImpulse = fixed_zero;
		m_upperImpulse = fixed_zero;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2WheelJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	fixed mA = m_invMassA, mB = m_invMassB;
	fixed iA = m_invIA, iB = m_invIB;

	b2Vec2 vA = data.velocities[m_indexA].v;
	fixed wA = data.velocities[m_indexA].w;
	b2Vec2 vB = data.velocities[m_indexB].v;
	fixed wB = data.velocities[m_indexB].w;

	// Solve spring constraint
	{
		fixed Cdot = b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
		fixed impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
		m_springImpulse += impulse;

		b2Vec2 P = impulse * m_ax;
		fixed LA = impulse * m_sAx;
		fixed LB = impulse * m_sBx;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	// Solve rotational motor constraint
	{
		fixed Cdot = wB - wA - m_motorSpeed;
		fixed impulse = -m_motorMass * Cdot;

		fixed oldImpulse = m_motorImpulse;
		fixed maxImpulse = data.step.dt * m_maxMotorTorque;
		m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	if (m_enableLimit)
	{
		// Lower limit
		{
			fixed C = m_translation - m_lowerTranslation;
			fixed Cdot = b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
			fixed impulse = -m_axialMass * (Cdot + b2Max(C, fixed_zero) * data.step.inv_dt);
			fixed oldImpulse = m_lowerImpulse;
			m_lowerImpulse = b2Max(m_lowerImpulse + impulse, fixed_zero);
			impulse = m_lowerImpulse - oldImpulse;

			b2Vec2 P = impulse * m_ax;
			fixed LA = impulse * m_sAx;
			fixed LB = impulse * m_sBx;

			vA -= mA * P;
			wA -= iA * LA;
			vB += mB * P;
			wB += iB * LB;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			fixed C = m_upperTranslation - m_translation;
			fixed Cdot = b2Dot(m_ax, vA - vB) + m_sAx * wA - m_sBx * wB;
			fixed impulse = -m_axialMass * (Cdot + b2Max(C, fixed_zero) * data.step.inv_dt);
			fixed oldImpulse = m_upperImpulse;
			m_upperImpulse = b2Max(m_upperImpulse + impulse, fixed_zero);
			impulse = m_upperImpulse - oldImpulse;

			b2Vec2 P = impulse * m_ax;
			fixed LA = impulse * m_sAx;
			fixed LB = impulse * m_sBx;

			vA += mA * P;
			wA += iA * LA;
			vB -= mB * P;
			wB -= iB * LB;
		}
	}

	// Solve point to line constraint
	{
		fixed Cdot = b2Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
		fixed impulse = -m_mass * Cdot;
		m_impulse += impulse;

		b2Vec2 P = impulse * m_ay;
		fixed LA = impulse * m_sAy;
		fixed LB = impulse * m_sBy;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2WheelJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[m_indexA].c;
	fixed aA = data.positions[m_indexA].a;
	b2Vec2 cB = data.positions[m_indexB].c;
	fixed aB = data.positions[m_indexB].a;

	fixed linearError = fixed_zero;

	if (m_enableLimit)
	{
		b2Rot qA(aA), qB(aB);

		b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
		b2Vec2 d = (cB - cA) + rB - rA;

		b2Vec2 ax = b2Mul(qA, m_localXAxisA);
		fixed sAx = b2Cross(d + rA, m_ax);
		fixed sBx = b2Cross(rB, m_ax);

		fixed C = fixed_zero;
		fixed translation = b2Dot(ax, d);
		if (b2Abs(m_upperTranslation - m_lowerTranslation) < fixed_two * b2_linearSlop)
		{
			C = translation;
		}
		else if (translation <= m_lowerTranslation)
		{
			C = b2Min(translation - m_lowerTranslation, fixed_zero);
		}
		else if (translation >= m_upperTranslation)
		{
			C = b2Max(translation - m_upperTranslation, fixed_zero);
		}

		if (C != fixed_zero)
		{

			fixed invMass = m_invMassA + m_invMassB + m_invIA * sAx * sAx + m_invIB * sBx * sBx;
			fixed impulse = fixed_zero;
			if (invMass != fixed_zero)
			{
				impulse = -C / invMass;
			}

			b2Vec2 P = impulse * ax;
			fixed LA = impulse * sAx;
			fixed LB = impulse * sBx;

			cA -= m_invMassA * P;
			aA -= m_invIA * LA;
			cB += m_invMassB * P;
			aB += m_invIB * LB;

			linearError = b2Abs(C);
		}
	}

	// Solve perpendicular constraint
	{
		b2Rot qA(aA), qB(aB);

		b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
		b2Vec2 d = (cB - cA) + rB - rA;

		b2Vec2 ay = b2Mul(qA, m_localYAxisA);

		fixed sAy = b2Cross(d + rA, ay);
		fixed sBy = b2Cross(rB, ay);

		fixed C = b2Dot(d, ay);

		fixed invMass = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

		fixed impulse = fixed_zero;
		if (invMass != fixed_zero)
		{
			impulse = - C / invMass;
		}

		b2Vec2 P = impulse * ay;
		fixed LA = impulse * sAy;
		fixed LB = impulse * sBy;

		cA -= m_invMassA * P;
		aA -= m_invIA * LA;
		cB += m_invMassB * P;
		aB += m_invIB * LB;

		linearError = b2Max(linearError, b2Abs(C));
	}

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return linearError <= b2_linearSlop;
}

b2Vec2 b2WheelJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2WheelJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2WheelJoint::GetReactionForce(fixed inv_dt) const
{
	return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
}

fixed b2WheelJoint::GetReactionTorque(fixed inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

fixed b2WheelJoint::GetJointTranslation() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 pA = bA->GetWorldPoint(m_localAnchorA);
	b2Vec2 pB = bB->GetWorldPoint(m_localAnchorB);
	b2Vec2 d = pB - pA;
	b2Vec2 axis = bA->GetWorldVector(m_localXAxisA);

	fixed translation = b2Dot(d, axis);
	return translation;
}

fixed b2WheelJoint::GetJointLinearSpeed() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
	b2Vec2 rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
	b2Vec2 p1 = bA->m_sweep.c + rA;
	b2Vec2 p2 = bB->m_sweep.c + rB;
	b2Vec2 d = p2 - p1;
	b2Vec2 axis = b2Mul(bA->m_xf.q, m_localXAxisA);

	b2Vec2 vA = bA->m_linearVelocity;
	b2Vec2 vB = bB->m_linearVelocity;
	fixed wA = bA->m_angularVelocity;
	fixed wB = bB->m_angularVelocity;

	fixed speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
	return speed;
}

fixed b2WheelJoint::GetJointAngle() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;
	return bB->m_sweep.a - bA->m_sweep.a;
}

fixed b2WheelJoint::GetJointAngularSpeed() const
{
	fixed wA = m_bodyA->m_angularVelocity;
	fixed wB = m_bodyB->m_angularVelocity;
	return wB - wA;
}

bool b2WheelJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void b2WheelJoint::EnableLimit(bool flag)
{
	if (flag != m_enableLimit)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_enableLimit = flag;
		m_lowerImpulse = fixed_zero;
		m_upperImpulse = fixed_zero;
	}
}

fixed b2WheelJoint::GetLowerLimit() const
{
	return m_lowerTranslation;
}

fixed b2WheelJoint::GetUpperLimit() const
{
	return m_upperTranslation;
}

void b2WheelJoint::SetLimits(fixed lower, fixed upper)
{
	b2Assert(lower <= upper);
	if (lower != m_lowerTranslation || upper != m_upperTranslation)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_lowerTranslation = lower;
		m_upperTranslation = upper;
		m_lowerImpulse = fixed_zero;
		m_upperImpulse = fixed_zero;
	}
}

bool b2WheelJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b2WheelJoint::EnableMotor(bool flag)
{
	if (flag != m_enableMotor)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_enableMotor = flag;
	}
}

void b2WheelJoint::SetMotorSpeed(fixed speed)
{
	if (speed != m_motorSpeed)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_motorSpeed = speed;
	}
}

void b2WheelJoint::SetMaxMotorTorque(fixed torque)
{
	if (torque != m_maxMotorTorque)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_maxMotorTorque = torque;
	}
}

fixed b2WheelJoint::GetMotorTorque(fixed inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void b2WheelJoint::SetStiffness(fixed stiffness)
{
	m_stiffness = stiffness;
}

fixed b2WheelJoint::GetStiffness() const
{
	return m_stiffness;
}

void b2WheelJoint::SetDamping(fixed damping)
{
	m_damping = damping;
}

fixed b2WheelJoint::GetDamping() const
{
	return m_damping;
}

void b2WheelJoint::Dump()
{
	// FLT_DECIMAL_DIG == 9

	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Dump("  b2WheelJointDef jd;\n");
	b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
	b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Dump("  jd.localAxisA.Set(%.9g, %.9g);\n", m_localXAxisA.x, m_localXAxisA.y);
	b2Dump("  jd.enableMotor = bool(%d);\n", m_enableMotor);
	b2Dump("  jd.motorSpeed = %.9g;\n", m_motorSpeed);
	b2Dump("  jd.maxMotorTorque = %.9g;\n", m_maxMotorTorque);
	b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
	b2Dump("  jd.damping = %.9g;\n", m_damping);
	b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

///
void b2WheelJoint::Draw(b2Draw* draw) const
{
	const b2Transform& xfA = m_bodyA->GetTransform();
	const b2Transform& xfB = m_bodyB->GetTransform();
	b2Vec2 pA = b2Mul(xfA, m_localAnchorA);
	b2Vec2 pB = b2Mul(xfB, m_localAnchorB);

	b2Vec2 axis = b2Mul(xfA.q, m_localXAxisA);

	b2Color c1(fixed(7, 10), fixed(7, 10), fixed(7, 10));
	b2Color c2(fixed(3, 10), fixed(9, 10), fixed(3, 10));
	b2Color c3(fixed(9, 10), fixed(3, 10), fixed(3, 10));
	b2Color c4(fixed(3, 10), fixed(3, 10), fixed(9, 10));
	b2Color c5(fixed(4, 10), fixed(4, 10), fixed(4, 10));

	draw->DrawSegment(pA, pB, c5);

	if (m_enableLimit)
	{
		b2Vec2 lower = pA + m_lowerTranslation * axis;
		b2Vec2 upper = pA + m_upperTranslation * axis;
		b2Vec2 perp = b2Mul(xfA.q, m_localYAxisA);
		draw->DrawSegment(lower, upper, c1);
		draw->DrawSegment(lower - fixed_half * perp, lower + fixed_half * perp, c2);
		draw->DrawSegment(upper - fixed_half * perp, upper + fixed_half * perp, c3);
	}
	else
	{
		draw->DrawSegment(pA - fixed_one * axis, pA + fixed_one * axis, c1);
	}

	draw->DrawPoint(pA, fixed_five, c1);
	draw->DrawPoint(pB, fixed_five, c4);
}
