#include <mitsuba/render/util.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/core/plugin.h>

MTS_NAMESPACE_BEGIN

class VolumeReader : public Utility {
public:
	enum EVolumeType {
		EFloat32 = 1,
		EFloat16 = 2,
		EUInt8 =3,
		EQuantizedDirections = 4
	};

	int run(int argc, char **argv) {
		if (argc != 2) {
			cout << "Syntax: mtsutil compressvol <input.vol> <output.vol>" << endl;
			Log(EError, "Invalid number of arguments!");
		}

		ref<FileStream> is = new  FileStream(argv[1], FileStream::EReadOnly);
		is->setByteOrder(Stream::ELittleEndian);

		char header[3];
		is->read(header, 3);

		cout << "header: " << header[0] << ", " << header[1] << ", " << header[2] << endl;

		uint8_t version;
		is->read(&version, 1);
		int type = is->readInt();

		int xres = is->readInt(),
			yres = is->readInt(),
			zres = is->readInt();
		int channels = is->readInt();

		Float xmin = is->readSingle(),
     		  ymin = is->readSingle(),
     		  zmin = is->readSingle();
		Float xmax = is->readSingle(),
     		  ymax = is->readSingle(),
     		  zmax = is->readSingle();

		cout << "type: " << type << endl;
		cout << "x, y, z res: " << xres << ", " << yres << ", " << zres << endl;
		cout << "channels: " << channels << endl;

		cout << "xmin, ymin, zmin: " << xmin << ", " << ymin << ", " << zmin << endl;
		cout << "xmax, ymax, zmax: " << xmax << ", " << ymax << ", " << zmax << endl;

		size_t totalSize = xres * yres * zres * channels;
		size_t type_size;

		switch (type) {
			case EFloat32:
				type_size = sizeof(float);
				break;
			case EFloat16:
				type_size = 2;
				break;
			case EUInt8:
				type_size = 1;
			default:
				Log(EError, "Encountered a volume data file of unknown type");
				break;
		}

		uint8_t *datau8 = new uint8_t[type_size * totalSize];
		is->read(datau8, type_size * totalSize);

		float* dataf = reinterpret_cast<float *>(datau8);

		/*
		for (int i = 0; i < totalSize; i++) {
			cout << dataf[i] << ", ";
		}
		cout << endl;*/

		return 0;
	}

	MTS_DECLARE_UTILITY()
};

MTS_EXPORT_UTILITY(VolumeReader, "Read CT data volumes")
MTS_NAMESPACE_END