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

#include "box2d/b2_draw.h"
#include "box2d/b2_rope.h"

#include <stdio.h>

struct b2RopeStretch
{
	int32 i1, i2;
	fixed invMass1, invMass2;
	fixed L;
	fixed lambda;
	fixed spring;
	fixed damper;
};

struct b2RopeBend
{
	int32 i1, i2, i3;
	fixed invMass1, invMass2, invMass3;
	fixed invEffectiveMass;
	fixed lambda;
	fixed L1, L2;
	fixed alpha1, alpha2;
	fixed spring;
	fixed damper;
};

b2Rope::b2Rope()
{
	m_position.SetZero();
	m_count = 0;
	m_stretchCount = 0;
	m_bendCount = 0;
	m_stretchConstraints = nullptr;
	m_bendConstraints = nullptr;
	m_bindPositions = nullptr;
	m_ps = nullptr;
	m_p0s = nullptr;
	m_vs = nullptr;
	m_invMasses = nullptr;
	m_gravity.SetZero();
}

b2Rope::~b2Rope()
{
	b2Free(m_stretchConstraints);
	b2Free(m_bendConstraints);
	b2Free(m_bindPositions);
	b2Free(m_ps);
	b2Free(m_p0s);
	b2Free(m_vs);
	b2Free(m_invMasses);
}

void b2Rope::Create(const b2RopeDef& def)
{
	b2Assert(def.count >= 3);
	m_position = def.position;
	m_count = def.count;
	m_bindPositions = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_ps = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_p0s = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_vs = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_invMasses = (fixed*)b2Alloc(m_count * sizeof(fixed));

	for (int32 i = 0; i < m_count; ++i)
	{
		m_bindPositions[i] = def.vertices[i];
		m_ps[i] = def.vertices[i] + m_position;
		m_p0s[i] = def.vertices[i] + m_position;
		m_vs[i].SetZero();

		fixed m = def.masses[i];
		if (m > fixed_zero)
		{
			m_invMasses[i] = fixed_one / m;
		}
		else
		{
			m_invMasses[i] = fixed_zero;
		}
	}

	m_stretchCount = m_count - 1;
	m_bendCount = m_count - 2;

	m_stretchConstraints = (b2RopeStretch*)b2Alloc(m_stretchCount * sizeof(b2RopeStretch));
	m_bendConstraints = (b2RopeBend*)b2Alloc(m_bendCount * sizeof(b2RopeBend));

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		b2RopeStretch& c = m_stretchConstraints[i];

		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i+1];

		c.i1 = i;
		c.i2 = i + 1;
		c.L = b2Distance(p1, p2);
		c.invMass1 = m_invMasses[i];
		c.invMass2 = m_invMasses[i + 1];
		c.lambda = fixed_zero;
		c.damper = fixed_zero;
		c.spring = fixed_zero;
	}

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		b2RopeBend& c = m_bendConstraints[i];

		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i + 1];
		b2Vec2 p3 = m_ps[i + 2];

		c.i1 = i;
		c.i2 = i + 1;
		c.i3 = i + 2;
		c.invMass1 = m_invMasses[i];
		c.invMass2 = m_invMasses[i + 1];
		c.invMass3 = m_invMasses[i + 2];
		c.invEffectiveMass = fixed_zero;
		c.L1 = b2Distance(p1, p2);
		c.L2 = b2Distance(p2, p3);
		c.lambda = fixed_zero;

		// Pre-compute effective mass (TODO use flattened config)
		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p2;
		fixed L1sqr = e1.LengthSquared();
		fixed L2sqr = e2.LengthSquared();

		if (L1sqr * L2sqr == fixed_zero)
		{
			continue;
		}

		b2Vec2 Jd1 = (-fixed_one / L1sqr) * e1.Skew();
		b2Vec2 Jd2 = (fixed_one / L2sqr) * e2.Skew();

		b2Vec2 J1 = -Jd1;
		b2Vec2 J2 = Jd1 - Jd2;
		b2Vec2 J3 = Jd2;

		c.invEffectiveMass = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
	
		b2Vec2 r = p3 - p1;

		fixed rr = r.LengthSquared();
		if (rr == fixed_zero)
		{
			continue;
		}

		// a1 = h2 / (h1 + h2)
		// a2 = h1 / (h1 + h2)
		c.alpha1 = b2Dot(e2, r) / rr;
		c.alpha2 = b2Dot(e1, r) / rr;
	}

	m_gravity = def.gravity;

	SetTuning(def.tuning);
}

void b2Rope::SetTuning(const b2RopeTuning& tuning)
{
	m_tuning = tuning;

	// Pre-compute spring and damper values based on tuning

	const fixed bendOmega = b2_two_pi * m_tuning.bendHertz;

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		b2RopeBend& c = m_bendConstraints[i];

		fixed L1sqr = c.L1 * c.L1;
		fixed L2sqr = c.L2 * c.L2;

		if (L1sqr * L2sqr == fixed_zero)
		{
			c.spring = fixed_zero;
			c.damper = fixed_zero;
			continue;
		}

		// Flatten the triangle formed by the two edges
		fixed J2 = fixed_one / c.L1 + fixed_one / c.L2;
		fixed sum = c.invMass1 / L1sqr + c.invMass2 * J2 * J2 + c.invMass3 / L2sqr;
		if (sum == fixed_zero)
		{
			c.spring = fixed_zero;
			c.damper = fixed_zero;
			continue;
		}

		fixed mass = fixed_one / sum;

		c.spring = mass * bendOmega * bendOmega;
		c.damper = fixed_two * mass * m_tuning.bendDamping * bendOmega;
	}
	
	const fixed stretchOmega = b2_two_pi * m_tuning.stretchHertz;

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		b2RopeStretch& c = m_stretchConstraints[i];

		fixed sum = c.invMass1 + c.invMass2;
		if (sum == fixed_zero)
		{
			continue;
		}

		fixed mass = fixed_one / sum;

		c.spring = mass * stretchOmega * stretchOmega;
		c.damper = fixed_two * mass * m_tuning.stretchDamping * stretchOmega;
	}
}

void b2Rope::Step(fixed dt, int32 iterations, const b2Vec2& position)
{
	if (dt == fixed_zero)
	{
		return;
	}

	const fixed inv_dt = fixed_one / dt;
	fixed d = b2Exp(- dt * m_tuning.damping);

	// Apply gravity and damping
	for (int32 i = 0; i < m_count; ++i)
	{
		if (m_invMasses[i] > fixed_zero)
		{
			m_vs[i] *= d;
			m_vs[i] += dt * m_gravity;
		}
		else
		{
			m_vs[i] = inv_dt * (m_bindPositions[i] + position - m_p0s[i]);
		}
	}

	// Apply bending spring
	if (m_tuning.bendingModel == b2_springAngleBendingModel)
	{
		ApplyBendForces(dt);
	}

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		m_bendConstraints[i].lambda = fixed_zero;
	}

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		m_stretchConstraints[i].lambda = fixed_zero;
	}

	// Update position
	for (int32 i = 0; i < m_count; ++i)
	{
		m_ps[i] += dt * m_vs[i];
	}

	// Solve constraints
	for (int32 i = 0; i < iterations; ++i)
	{
		if (m_tuning.bendingModel == b2_pbdAngleBendingModel)
		{
			SolveBend_PBD_Angle();
		}
		else if (m_tuning.bendingModel == b2_xpbdAngleBendingModel)
		{
			SolveBend_XPBD_Angle(dt);
		}
		else if (m_tuning.bendingModel == b2_pbdDistanceBendingModel)
		{
			SolveBend_PBD_Distance();
		}
		else if (m_tuning.bendingModel == b2_pbdHeightBendingModel)
		{
			SolveBend_PBD_Height();
		}

		if (m_tuning.stretchingModel == b2_pbdStretchingModel)
		{
			SolveStretch_PBD();
		}
		else if (m_tuning.stretchingModel == b2_xpbdStretchingModel)
		{
			SolveStretch_XPBD(dt);
		}
	}

	// Constrain velocity
	for (int32 i = 0; i < m_count; ++i)
	{
		m_vs[i] = inv_dt * (m_ps[i] - m_p0s[i]);
		m_p0s[i] = m_ps[i];
	}
}

void b2Rope::Reset(const b2Vec2& position)
{
	m_position = position;

	for (int32 i = 0; i < m_count; ++i)
	{
		m_ps[i] = m_bindPositions[i] + m_position;
		m_p0s[i] = m_bindPositions[i] + m_position;
		m_vs[i].SetZero();
	}

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		m_bendConstraints[i].lambda = fixed_zero;
	}

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		m_stretchConstraints[i].lambda = fixed_zero;
	}
}

void b2Rope::SolveStretch_PBD()
{
	const fixed stiffness = m_tuning.stretchStiffness;

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		const b2RopeStretch& c = m_stretchConstraints[i];

		b2Vec2 p1 = m_ps[c.i1];
		b2Vec2 p2 = m_ps[c.i2];

		b2Vec2 d = p2 - p1;
		fixed L = d.Normalize();

		fixed sum = c.invMass1 + c.invMass2;
		if (sum == fixed_zero)
		{
			continue;
		}

		fixed s1 = c.invMass1 / sum;
		fixed s2 = c.invMass2 / sum;

		p1 -= stiffness * s1 * (c.L - L) * d;
		p2 += stiffness * s2 * (c.L - L) * d;

		m_ps[c.i1] = p1;
		m_ps[c.i2] = p2;
	}
}

void b2Rope::SolveStretch_XPBD(fixed dt)
{
	b2Assert(dt > fixed_zero);

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		b2RopeStretch& c = m_stretchConstraints[i];

		b2Vec2 p1 = m_ps[c.i1];
		b2Vec2 p2 = m_ps[c.i2];

		b2Vec2 dp1 = p1 - m_p0s[c.i1];
		b2Vec2 dp2 = p2 - m_p0s[c.i2];

		b2Vec2 u = p2 - p1;
		fixed L = u.Normalize();

		b2Vec2 J1 = -u;
		b2Vec2 J2 = u;

		fixed sum = c.invMass1 + c.invMass2;
		if (sum == fixed_zero)
		{
			continue;
		}

		const fixed alpha = fixed_one / (c.spring * dt * dt);	// 1 / kg
		const fixed beta = dt * dt * c.damper;				// kg * s
		const fixed sigma = alpha * beta / dt;				// non-dimensional
		fixed C = L - c.L;

		// This is using the initial velocities
		fixed Cdot = b2Dot(J1, dp1) + b2Dot(J2, dp2);

		fixed B = C + alpha * c.lambda + sigma * Cdot;
		fixed sum2 = (fixed_one + sigma) * sum + alpha;

		fixed impulse = -B / sum2;

		p1 += (c.invMass1 * impulse) * J1;
		p2 += (c.invMass2 * impulse) * J2;

		m_ps[c.i1] = p1;
		m_ps[c.i2] = p2;
		c.lambda += impulse;
	}
}

void b2Rope::SolveBend_PBD_Angle()
{
	const fixed stiffness = m_tuning.bendStiffness;

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		const b2RopeBend& c = m_bendConstraints[i];

		b2Vec2 p1 = m_ps[c.i1];
		b2Vec2 p2 = m_ps[c.i2];
		b2Vec2 p3 = m_ps[c.i3];

		b2Vec2 d1 = p2 - p1;
		b2Vec2 d2 = p3 - p2;
		fixed a = b2Cross(d1, d2);
		fixed b = b2Dot(d1, d2);

		fixed angle = b2Atan2(a, b);

		fixed L1sqr, L2sqr;
		
		if (m_tuning.isometric)
		{
			L1sqr = c.L1 * c.L1;
			L2sqr = c.L2 * c.L2;
		}
		else
		{
			L1sqr = d1.LengthSquared();
			L2sqr = d2.LengthSquared();
		}

		if (L1sqr * L2sqr == fixed_zero)
		{
			continue;
		}

		b2Vec2 Jd1 = (-fixed_one / L1sqr) * d1.Skew();
		b2Vec2 Jd2 = (fixed_one / L2sqr) * d2.Skew();

		b2Vec2 J1 = -Jd1;
		b2Vec2 J2 = Jd1 - Jd2;
		b2Vec2 J3 = Jd2;

		fixed sum;
		if (m_tuning.fixedEffectiveMass)
		{
			sum = c.invEffectiveMass;
		}
		else
		{
			sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
		}

		if (sum == fixed_zero)
		{
			sum = c.invEffectiveMass;
		}

		fixed impulse = -stiffness * angle / sum;

		p1 += (c.invMass1 * impulse) * J1;
		p2 += (c.invMass2 * impulse) * J2;
		p3 += (c.invMass3 * impulse) * J3;

		m_ps[c.i1] = p1;
		m_ps[c.i2] = p2;
		m_ps[c.i3] = p3;
	}
}

void b2Rope::SolveBend_XPBD_Angle(fixed dt)
{
	b2Assert(dt > fixed_zero);

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		b2RopeBend& c = m_bendConstraints[i];

		b2Vec2 p1 = m_ps[c.i1];
		b2Vec2 p2 = m_ps[c.i2];
		b2Vec2 p3 = m_ps[c.i3];

		b2Vec2 dp1 = p1 - m_p0s[c.i1];
		b2Vec2 dp2 = p2 - m_p0s[c.i2];
		b2Vec2 dp3 = p3 - m_p0s[c.i3];

		b2Vec2 d1 = p2 - p1;
		b2Vec2 d2 = p3 - p2;

		fixed L1sqr, L2sqr;

		if (m_tuning.isometric)
		{
			L1sqr = c.L1 * c.L1;
			L2sqr = c.L2 * c.L2;
		}
		else
		{
			L1sqr = d1.LengthSquared();
			L2sqr = d2.LengthSquared();
		}

		if (L1sqr * L2sqr == fixed_zero)
		{
			continue;
		}

		fixed a = b2Cross(d1, d2);
		fixed b = b2Dot(d1, d2);

		fixed angle = b2Atan2(a, b);

		b2Vec2 Jd1 = (-fixed_one / L1sqr) * d1.Skew();
		b2Vec2 Jd2 = (fixed_one / L2sqr) * d2.Skew();

		b2Vec2 J1 = -Jd1;
		b2Vec2 J2 = Jd1 - Jd2;
		b2Vec2 J3 = Jd2;

		fixed sum;
		if (m_tuning.fixedEffectiveMass)
		{
			sum = c.invEffectiveMass;
		}
		else
		{
			sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
		}

		if (sum == fixed_zero)
		{
			continue;
		}

		const fixed alpha = fixed_one / (c.spring * dt * dt);
		const fixed beta = dt * dt * c.damper;
		const fixed sigma = alpha * beta / dt;
		fixed C = angle;

		// This is using the initial velocities
		fixed Cdot = b2Dot(J1, dp1) + b2Dot(J2, dp2) + b2Dot(J3, dp3);

		fixed B = C + alpha * c.lambda + sigma * Cdot;
		fixed sum2 = (fixed_one + sigma) * sum + alpha;

		fixed impulse = -B / sum2;

		p1 += (c.invMass1 * impulse) * J1;
		p2 += (c.invMass2 * impulse) * J2;
		p3 += (c.invMass3 * impulse) * J3;

		m_ps[c.i1] = p1;
		m_ps[c.i2] = p2;
		m_ps[c.i3] = p3;
		c.lambda += impulse;
	}
}

void b2Rope::ApplyBendForces(fixed dt)
{
	// omega = 2 * pi * hz
	const fixed omega = b2_two_pi * m_tuning.bendHertz;

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		const b2RopeBend& c = m_bendConstraints[i];

		b2Vec2 p1 = m_ps[c.i1];
		b2Vec2 p2 = m_ps[c.i2];
		b2Vec2 p3 = m_ps[c.i3];

		b2Vec2 v1 = m_vs[c.i1];
		b2Vec2 v2 = m_vs[c.i2];
		b2Vec2 v3 = m_vs[c.i3];

		b2Vec2 d1 = p2 - p1;
		b2Vec2 d2 = p3 - p2;

		fixed L1sqr, L2sqr;

		if (m_tuning.isometric)
		{
			L1sqr = c.L1 * c.L1;
			L2sqr = c.L2 * c.L2;
		}
		else
		{
			L1sqr = d1.LengthSquared();
			L2sqr = d2.LengthSquared();
		}

		if (L1sqr * L2sqr == fixed_zero)
		{
			continue;
		}

		fixed a = b2Cross(d1, d2);
		fixed b = b2Dot(d1, d2);

		fixed angle = b2Atan2(a, b);

		b2Vec2 Jd1 = (-fixed_one / L1sqr) * d1.Skew();
		b2Vec2 Jd2 = (fixed_one / L2sqr) * d2.Skew();

		b2Vec2 J1 = -Jd1;
		b2Vec2 J2 = Jd1 - Jd2;
		b2Vec2 J3 = Jd2;

		fixed sum;
		if (m_tuning.fixedEffectiveMass)
		{
			sum = c.invEffectiveMass;
		}
		else
		{
			sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
		}

		if (sum == fixed_zero)
		{
			continue;
		}

		fixed mass = fixed_one / sum;

		const fixed spring = mass * omega * omega;
		const fixed damper = fixed_two * mass * m_tuning.bendDamping * omega;

		fixed C = angle;
		fixed Cdot = b2Dot(J1, v1) + b2Dot(J2, v2) + b2Dot(J3, v3);

		fixed impulse = -dt * (spring * C + damper * Cdot);

		m_vs[c.i1] += (c.invMass1 * impulse) * J1;
		m_vs[c.i2] += (c.invMass2 * impulse) * J2;
		m_vs[c.i3] += (c.invMass3 * impulse) * J3;
	}
}

void b2Rope::SolveBend_PBD_Distance()
{
	const fixed stiffness = m_tuning.bendStiffness;

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		const b2RopeBend& c = m_bendConstraints[i];

		int32 i1 = c.i1;
		int32 i2 = c.i3;

		b2Vec2 p1 = m_ps[i1];
		b2Vec2 p2 = m_ps[i2];

		b2Vec2 d = p2 - p1;
		fixed L = d.Normalize();

		fixed sum = c.invMass1 + c.invMass3;
		if (sum == fixed_zero)
		{
			continue;
		}

		fixed s1 = c.invMass1 / sum;
		fixed s2 = c.invMass3 / sum;

		p1 -= stiffness * s1 * (c.L1 + c.L2 - L) * d;
		p2 += stiffness * s2 * (c.L1 + c.L2 - L) * d;

		m_ps[i1] = p1;
		m_ps[i2] = p2;
	}
}

// Constraint based implementation of:
// P. Volino: Simple Linear Bending Stiffness in Particle Systems
void b2Rope::SolveBend_PBD_Height()
{
	const fixed stiffness = m_tuning.bendStiffness;

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		const b2RopeBend& c = m_bendConstraints[i];

		b2Vec2 p1 = m_ps[c.i1];
		b2Vec2 p2 = m_ps[c.i2];
		b2Vec2 p3 = m_ps[c.i3];

		// Barycentric coordinates are held constant
		b2Vec2 d = c.alpha1 * p1 + c.alpha2 * p3 - p2;
		fixed dLen = d.Length();

		if (dLen == fixed_zero)
		{
			continue;
		}

		b2Vec2 dHat = (fixed_one / dLen) * d;

		b2Vec2 J1 = c.alpha1 * dHat;
		b2Vec2 J2 = -dHat;
		b2Vec2 J3 = c.alpha2 * dHat;

		fixed sum = c.invMass1 * c.alpha1 * c.alpha1 + c.invMass2 + c.invMass3 * c.alpha2 * c.alpha2;

		if (sum == fixed_zero)
		{
			continue;
		}

		fixed C = dLen;
		fixed mass = fixed_one / sum;
		fixed impulse = -stiffness * mass * C;

		p1 += (c.invMass1 * impulse) * J1;
		p2 += (c.invMass2 * impulse) * J2;
		p3 += (c.invMass3 * impulse) * J3;

		m_ps[c.i1] = p1;
		m_ps[c.i2] = p2;
		m_ps[c.i3] = p3;
	}
}

void b2Rope::Draw(b2Draw* draw) const
{
	b2Color c(fixed(4, 10), fixed(5, 10), fixed(7, 10));
	b2Color pg(fixed_tenth, fixed(8, 10), fixed_tenth);
	b2Color pd(fixed(7, 10), fixed(2, 10), fixed(4, 10));

	for (int32 i = 0; i < m_count - 1; ++i)
	{
		draw->DrawSegment(m_ps[i], m_ps[i+1], c);

		const b2Color& pc = m_invMasses[i] > fixed_zero ? pd : pg;
		draw->DrawPoint(m_ps[i], fixed_five, pc);
	}

	const b2Color& pc = m_invMasses[m_count - 1] > fixed_zero ? pd : pg;
	draw->DrawPoint(m_ps[m_count - 1], fixed_five, pc);
}
