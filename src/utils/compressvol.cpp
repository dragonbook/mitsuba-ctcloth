/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2010 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/util.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/core/plugin.h>

MTS_NAMESPACE_BEGIN

class CompressVolume : public Utility {
public:
	int run(int argc, char **argv) {
		if (argc != 3) {
			cout << "Syntax: mtsutil compressvol <input.vol> <output.vol>" << endl;
			Log(EError, "Invalid number of arguments!");
		}

		ref<FileStream> is = new FileStream(argv[1], FileStream::EReadOnly);
		ref<FileStream> os = new FileStream(argv[2], FileStream::ETruncReadWrite);
		char header[3];
		is->read(header, 3);
		if (header[0] != 'V' || header[1] != 'O' || header[2] != 'L')
			Log(EError, "Invalid format!");
		os->write(header, 3);
		uint8_t filever;
		is->read(&filever, 1);
		if (filever != 3) 
			Log(EError, "Invalid format!");
		os->write(&filever, 1);
		if (is->readInt() != 1)
			Log(EError, "Expected an uncompressed volume as input!");

		int xres = is->readInt(), yres=is->readInt(), zres=is->readInt();
        int channels = is->readInt();

        os->writeInt( channels == 1 ? 3 : 4 );
		os->writeInt(xres);
		os->writeInt(yres);
		os->writeInt(zres);
		
		os->writeInt(channels);
		Float xmin = is->readSingle(), ymin = is->readSingle(), zmin = is->readSingle();
		Float xmax = is->readSingle(), ymax = is->readSingle(), zmax = is->readSingle();
		os->writeFloat(xmin);	
		os->writeFloat(ymin);	
		os->writeFloat(zmin);	
		os->writeFloat(xmax);	
		os->writeFloat(ymax);	
		os->writeFloat(zmax);	
		size_t totalSize = xres*yres*zres;

#if 0
		Float cosPhi[255], sinPhi[255];
		Float cosTheta[255], sinTheta[255];
		for (int i=0; i<255; i++) {
			Float angle = (Float) i * ((Float) M_PI / 255.0f);
			cosPhi[i] = std::cos(2.0f * angle);
			sinPhi[i] = std::sin(2.0f * angle);
			/* Theta has twice the angular resolution */
			cosTheta[i] = std::cos(angle);
			sinTheta[i] = std::sin(angle);
		}
#endif

		if (channels == 1) {
			float *data = new float[totalSize];
			uint8_t *data8 = new uint8_t[totalSize];
			is->read(data, totalSize*sizeof(Float));
			Float maxValue = -std::numeric_limits<Float>::infinity();

			for (size_t i=0; i<totalSize; ++i)
				maxValue = std::max(static_cast<Float>(data[i]), maxValue);
			cout << "Maximum value = " << maxValue << endl;
			cout << "Converting to 8-bit" << endl;
			for (size_t i=0; i<totalSize; ++i)
				data8[i] = (int) (data[i]/maxValue*255);

			//os->writeSingle(maxValue/255.0f);
			os->write(data8, totalSize*sizeof(uint8_t));

			delete[] data;
			delete[] data8;
		} else if (channels == 3) {
			Vector *data = new Vector[totalSize];
			uint8_t *data8 = new uint8_t[totalSize*2];
			//os->writeSingle(0); // bogus scale
			is->read(data, totalSize*sizeof(Vector));
			cout << "Converting to 16-bit (spherical coordinates)" << endl;
			for (size_t i=0; i<totalSize; ++i) {
				Vector dir = data[i];
				Float length = dir.length();
				if (length == 0) {
					data8[2*i] = 255;
					data8[2*i+1] = 255;
					continue;
				}
				dir /= length;

				const int nDiscretizationMax = 254; /* 255 is reserved for 'no direction' */
				/* Convert the direction into an approximate spherical 
				   coordinate format to reduce storage requirements */
				uint8_t phi, theta = (uint8_t) std::min(nDiscretizationMax,
					(int) (std::acos(dir.z) * ((nDiscretizationMax+1) / M_PI)));

				int tmp = std::min(nDiscretizationMax,
					(int) (std::atan2(dir.y, dir.x) * ((nDiscretizationMax+1) / (2.0 * M_PI))));
				if (tmp < 0)
					phi = (uint8_t) (tmp + 255);
				else
					phi = (uint8_t) tmp;
				data8[2*i] = theta;
				data8[2*i+1] = phi;
			}
			os->write(data8, totalSize*2*sizeof(uint8_t));
			delete[] data;
			delete[] data8;
		} else {
			Log(EError, "Unsupported number of channels!");
		}

		return 0;
	}

	MTS_DECLARE_UTILITY()
};

MTS_EXPORT_UTILITY(CompressVolume, "Compress CT data volumes")
MTS_NAMESPACE_END
