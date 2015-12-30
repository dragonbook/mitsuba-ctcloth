/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2012 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/core/warp.h>

MTS_NAMESPACE_BEGIN

Vector Warp::squareToUniformSphere(const Point2 &sample) {
	Float z = 1.0f - 2.0f * sample.y;
	Float r = math::safe_sqrt(1.0f - z*z);
	Float sinPhi, cosPhi;
	math::sincos(2.0f * M_PI * sample.x, &sinPhi, &cosPhi);
	return Vector(r * cosPhi, r * sinPhi, z);
}

Vector Warp::squareToUniformHemisphere(const Point2 &sample) {
	Float z = sample.x;
	Float tmp = math::safe_sqrt(1.0f - z*z);

	Float sinPhi, cosPhi;
	math::sincos(2.0f * M_PI * sample.y, &sinPhi, &cosPhi);

	return Vector(cosPhi * tmp, sinPhi * tmp, z);
}

Vector Warp::squareToCosineHemisphere(const Point2 &sample) {
	Point2 p = Warp::squareToUniformDiskConcentric(sample);
	Float z = math::safe_sqrt(1.0f - p.x*p.x - p.y*p.y);

	/* Guard against numerical imprecisions */
	if (EXPECT_NOT_TAKEN(z == 0))
		z = 1e-10f;

	return Vector(p.x, p.y, z);
}


Vector Warp::squareToUniformCone(Float cosCutoff, const Point2 &sample) {
	Float cosTheta = (1-sample.x) + sample.x * cosCutoff;
	Float sinTheta = math::safe_sqrt(1.0f - cosTheta * cosTheta);

	Float sinPhi, cosPhi;
	math::sincos(2.0f * M_PI * sample.y, &sinPhi, &cosPhi);

	return Vector(cosPhi * sinTheta,
		sinPhi * sinTheta, cosTheta);
}

Point2 Warp::squareToUniformDisk(const Point2 &sample) {
	Float r = std::sqrt(sample.x);
	Float sinPhi, cosPhi;
	math::sincos(2.0f * M_PI * sample.y, &sinPhi, &cosPhi);

	return Point2(
		cosPhi * r,
		sinPhi * r
	);
}

Point2 Warp::squareToUniformTriangle(const Point2 &sample) {
	Float a = math::safe_sqrt(1.0f - sample.x);
	return Point2(1 - a, a * sample.y);
}

Point2 Warp::squareToUniformDiskConcentric(const Point2 &sample) {
	Float r1 = 2.0f*sample.x - 1.0f;
	Float r2 = 2.0f*sample.y - 1.0f;

	Point2 coords;
	if (r1 == 0 && r2 == 0) {
		coords = Point2(0, 0);
	} else if (r1 > -r2) { /* Regions 1/2 */
		if (r1 > r2)
			coords = Point2(r1, (M_PI/4.0f) * r2/r1);
		else
			coords = Point2(r2, (M_PI/4.0f) * (2.0f - r1/r2));
	} else { /* Regions 3/4 */
		if (r1<r2)
			coords = Point2(-r1, (M_PI/4.0f) * (4.0f + r2/r1));
		else
			coords = Point2(-r2, (M_PI/4.0f) * (6.0f - r1/r2));
	}

	Point2 result;
	math::sincos(coords.y, &result.y, &result.x);
	return result*coords.x;
}

Point2 Warp::uniformDiskToSquareConcentric(const Point2 &p) {
	Float r   = std::sqrt(p.x * p.x + p.y * p.y),
		  phi = std::atan2(p.y, p.x),
		  a, b;

	if (phi < -M_PI/4) {
  		/* in range [-pi/4,7pi/4] */
		phi += 2*M_PI;
	}

	if (phi < M_PI/4) { /* region 1 */
		a = r;
		b = phi * a / (M_PI/4);
	} else if (phi < 3*M_PI/4) { /* region 2 */
		b = r;
		a = -(phi - M_PI/2) * b / (M_PI/4);
	} else if (phi < 5*M_PI/4) { /* region 3 */
		a = -r;
		b = (phi - M_PI) * a / (M_PI/4);
	} else { /* region 4 */
		b = -r;
		a = -(phi - 3*M_PI/2) * b / (M_PI/4);
	}

	return Point2(0.5f * (a+1), 0.5f * (b+1));
}

Point2 Warp::squareToStdNormal(const Point2 &sample) {
	Float r   = std::sqrt(-2 * math::fastlog(1-sample.x)),
		  phi = 2 * M_PI * sample.y;
	Point2 result;
	math::sincos(phi, &result.y, &result.x);
	return result * r;
}

Float Warp::squareToStdNormalPdf(const Point2 &pos) {
	return INV_TWOPI * math::fastexp(-(pos.x*pos.x + pos.y*pos.y)/2.0f);
}

static Float intervalToTent(Float sample) {
	Float sign;

	if (sample < 0.5f) {
		sign = 1;
		sample *= 2;
	} else {
		sign = -1;
		sample = 2 * (sample - 0.5f);
	}

	return sign * (1 - std::sqrt(sample));
}

Point2 Warp::squareToTent(const Point2 &sample) {
	return Point2(
		intervalToTent(sample.x),
		intervalToTent(sample.y)
	);
}

Float Warp::intervalToNonuniformTent(Float a, Float b, Float c, Float sample) {
	Float factor;

	if (sample * (c-a) < b-a) {
		factor = a-b;
		sample *= (a-c)/(a-b);
	} else {
		factor = c-b;
		sample = (a-c)/(b-c) * (sample - (a-b)/(a-c));
	}

	return b + factor * (1-math::safe_sqrt(sample));
}

MTS_NAMESPACE_END
