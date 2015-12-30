/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2011 by Wenzel Jakob and others.

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

#include <mitsuba/render/volume2.h>

#if defined(__LINUX__)
#include <sys/mman.h>
#include <fcntl.h>
#endif

MTS_NAMESPACE_BEGIN

VolumeDataSourceEx::VolumeDataSourceEx(Stream *stream, InstanceManager *manager) :
	VolumeDataSource(stream, manager) {
}

VolumeDataSourceEx::VolumeDataSourceEx(const Properties &props) : VolumeDataSource(props) { }

VolumeDataSourceEx::~VolumeDataSourceEx() { }

Float VolumeDataSourceEx::lookupFloatEx(uint32_t id, const Point &p) const {
	Log(EError, "'%s': does not implement lookupFloat()!", getClass()->getName().c_str());
	return 0.0f;
}

Spectrum VolumeDataSourceEx::lookupSpectrumEx(uint32_t id, const Point &p) const {
	Log(EError, "'%s': does not implement lookupSpectrum()!", getClass()->getName().c_str());
	return Spectrum(0.0f);
}

Vector VolumeDataSourceEx::lookupVectorEx(uint32_t id, const Point &p) const {
	Log(EError, "'%s': does not implement lookupVector()!", getClass()->getName().c_str());
	return Vector();
}

bool VolumeDataSourceEx::supportsBundleLookups() const {
    return false;
}

void VolumeDataSourceEx::lookupBundle(const Point &p,
    Float *density, Vector *direction, Spectrum *albedo, Float *gloss) const {
	Log(EError, "'%s': does not implement lookup()!", getClass()->getName().c_str());
}

MTS_IMPLEMENT_CLASS(VolumeDataSourceEx, true, VolumeDataSource)
MTS_NAMESPACE_END
