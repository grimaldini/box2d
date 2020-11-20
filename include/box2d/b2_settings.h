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

#ifndef B2_SETTINGS_H
#define B2_SETTINGS_H

#include <assert.h>
#include <fpm/fixed.hpp>

#if !defined(NDEBUG)
	#define b2DEBUG
#endif

#define B2_NOT_USED(x) ((void)(x))
#define b2Assert(A) assert(A)

typedef signed char	int8;
typedef signed short int16;
typedef signed int int32;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

using fixed =									fpm::fixed<__int64_t, __int128_t, 32u>;
static const fixed fixed_zero =					fixed(0);
static const fixed fixed_one =					fixed(1);
static const fixed fixed_two =					fixed(2);
static const fixed fixed_three =				fixed(3);
static const fixed fixed_four =					fixed(4);
static const fixed fixed_five =					fixed(5);
static const fixed fixed_six =					fixed(6);
static const fixed fixed_seven =				fixed(7);
static const fixed fixed_eight =				fixed(8);
static const fixed fixed_nine =					fixed(9);
static const fixed fixed_ten =					fixed(10);
static const fixed fixed_twelve =				fixed(12);
static const fixed fixed_fifty =				fixed(50);
static const fixed fixed_hundred =				fixed(100);
static const fixed fixed_thousand =				fixed(1000);
static const fixed fixed_half =					fixed(1, 2);
static const fixed fixed_third =				fixed(1, 3);
static const fixed fixed_quarter =				fixed(1, 4);
static const fixed fixed_fifth =				fixed(1, 5);
static const fixed fixed_three_quarters =		fixed(3, 4);
static const fixed fixed_tenth =				fixed(1, 10);
static const fixed fixed_hundredth =			fixed(1, 100);
static const fixed fixed_thousandth =			fixed(1, 1000);
static const fixed fixed_tenthousandth =		fixed(1, 10000);
static const fixed fixed_hundredthousandth =	fixed(1, 100000);
static const fixed fixed_millionth =			fixed(1, 1000000);

static const fixed b2_max =						std::numeric_limits<fixed>::max();
static const fixed b2_epsilon =					std::numeric_limits<fixed>::epsilon();
static const fixed b2_two_pi =					fixed::two_pi();
static const fixed b2_pi =						fixed::pi();

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
static const int b2_maxManifoldPoints = 2;

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
static const int b2_maxPolygonVertices = 8;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
static const fixed b2_aabbExtension = fixed_tenth;

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
static const fixed b2_aabbMultiplier = fixed_four;

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
static const fixed b2_linearSlop = fixed(5, 1000);

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
static const fixed b2_angularSlop = b2_two_pi / fixed(180);

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
static const fixed b2_polygonRadius = fixed_two * b2_linearSlop;

/// Maximum number of sub-steps per contact in continuous physics simulation.
static const int b2_maxSubSteps = 8;


// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
static const int b2_maxTOIContacts = 32;

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
static const fixed b2_velocityThreshold = fixed_one;

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
static const fixed b2_maxLinearCorrection = fixed(2, 10);

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
static const fixed b2_maxAngularCorrection = b2_pi * fixed(8, 180);

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
static const fixed b2_maxTranslation = fixed_two;
static const fixed b2_maxTranslationSquared = b2_maxTranslation * b2_maxTranslation;

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
static const fixed b2_maxRotation = fixed_half * b2_pi;
static const fixed b2_maxRotationSquared = b2_maxRotation * b2_maxRotation;

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
static const fixed b2_baumgarte = fixed(2, 10);
static const fixed b2_toiBaumgarte = fixed_three_quarters;


// Sleep

/// The time that a body must be still before it will go to sleep.
static const fixed b2_timeToSleep = fixed_half;

/// A body cannot sleep if its linear velocity is above this tolerance.
static const fixed b2_linearSleepTolerance = fixed_hundredth;

/// A body cannot sleep if its angular velocity is above this tolerance.
static const fixed b2_angularSleepTolerance = b2_two_pi * fixed(1, 180);

// Memory Allocation

/// Implement this function to use your own memory allocator.
void* b2Alloc(int32 size);

/// If you implement b2Alloc, you should also implement this function.
void b2Free(void* mem);

/// Logging function.
void b2Log(const char* string, ...);

/// Dump to a file. Only one dump file allowed at a time.
void b2OpenDump(const char* fileName);
void b2Dump(const char* string, ...);
void b2CloseDump();

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct b2Version
{
	int32 major;		///< significant changes
	int32 minor;		///< incremental changes
	int32 revision;		///< bug fixes
};

/// Current version.
extern b2Version b2_version;

#endif
