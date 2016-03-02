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

	void readBlockInfo(const std::string &filename) {
		cout << "Read block information in file: <" << filename << "> .." << endl;
		ref<FileStream> stream = new FileStream(filename, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

		int type = stream->readInt();
		cout << "type: " << type << endl;

		int n_blocks = stream->readInt();
		cout << "n_blocks: " << n_blocks << endl;

		cout << "filename, scale: ---" << endl;
		for (size_t i = 0; i < n_blocks; ++i) {
			std::string fn = stream->readString();
			Vector s = Vector(stream);

			cout << fn << ", (" << s.x << "," << s.y << "," << s.z << ")." << endl;
		}
		cout << "--------------------" << endl;
		cout << "Done.";
	}

	void readIndices(const std::string &filename) {
		cout << "Read Indices information in file: <" << filename << "> .." << endl;
		ref<FileStream> stream = new FileStream(filename, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

		Vector2i res = Vector2i(stream);
		cout << "res: " << res.x << "," << res.y << endl;
		AABB dataAABB = AABB(stream);
		cout << "AABB: " << endl;
		cout << "min: " << dataAABB.min.x << "," << dataAABB.min.y << "," << dataAABB.min.z << endl;
		cout << "min: " << dataAABB.max.x << "," << dataAABB.max.y << "," << dataAABB.max.z << endl;

		int *tileId = new int[res.x * res.y];
		stream->readIntArray(tileId, res.x * res.y);

		Float *transforms = new Float[3 * res.x*res.y];
		stream->readFloatArray(&transforms[0], 3 * res.x*res.y);

		bool foundId[8];
		for (int i = 0; i < 8; ++i) foundId[i] = false;

		cout << "tile ids and transforms:" << endl;
		for (int i = 0; i < res.x * res.y; ++i) {
//			if (!foundId[tileId[i]]) {
				cout << tileId[i] << ", ";
				cout << "(" << transforms[3 * i] << "," << transforms[3 * i + 1] << "," << transforms[3 * i + 2] << ")," << endl;

//				foundId[tileId[i]] = true;
//			}
		}
		cout << endl;

		int nSpectrumMaps = stream->readInt();
		cout << "nSpectrumMaps: " << nSpectrumMaps << endl;
		for (size_t i = 0; i < nSpectrumMaps; ++i) {
			cout << "map" << i << ":" << endl;
			int nitem = stream->readInt();
			cout << "nitem: " << nitem << endl;
			for (int j = 0; j < nitem; ++j) {
				int v = stream->readInt();
				float r = stream->readFloat();
				float g = stream->readFloat();
				float b = stream->readFloat();
				cout << "v,(r,g,b): " << v << ",(" << r << "," << g << "," << b << ")" << endl;
//				Spectrum s;
//				s.fromLinearRGB(r, g, b);
			}
		}

		int *tileSpectrumMap = new int[res.x * res.y];
		stream->readIntArray(tileSpectrumMap, res.x*res.y);
		for (int i = 0; i < res.x; ++i) {
			cout << tileSpectrumMap[i] << ",";
		}
		cout << endl;
		cout << "Done." << endl;
	}

	int run(int argc, char **argv) {
		if (argc != 3) {
			cout << "Syntax: mtsutil readvol <volumeTypeId> <input.vol> " << endl;
			cout << "ID-Type: 0-simpleVolume, 1-blockInfo, 2-blockIndices " << endl;
			Log(EError, "Invalid number of arguments!");
		}

		if (argv[1][0] == '1') {
			readBlockInfo(argv[2]);
			return 0;
		}
		else if (argv[1][0] == '2') {
			readIndices(argv[2]);
			return 0;
		}

		ref<FileStream> is = new  FileStream(argv[2], FileStream::EReadOnly);
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
				break;
			case EQuantizedDirections:
				type_size = 2;//?
				break;
			default:
				Log(EError, "Encountered a volume data file of unknown type");
				break;
		}

//		uint8_t *datau8 = new uint8_t[type_size * totalSize];
//		is->read(datau8, type_size * totalSize);

//		float* dataf = reinterpret_cast<float *>(datau8);

		/*
		for (int i = 0; i < totalSize; i++) {
			cout << dataf[i] << ", ";
		}
		cout << endl;*/

		return 0;
	}

	MTS_DECLARE_UTILITY()
};

MTS_EXPORT_UTILITY(VolumeReader, "Read CT data volumes(0) or BlockInfo(1) or BlockIndices(2)")
MTS_NAMESPACE_END