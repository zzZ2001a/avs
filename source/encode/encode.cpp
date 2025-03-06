

//#include <iostream>

#include <string>
#include <iostream>
#include <fstream>

#include "draco_encoder.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"



#include "draco/io/stdio_file_reader.h"
#include "draco/io/file_reader_factory.h"
#include "draco/io/stdio_file_writer.h"
#include "draco/io/file_writer_factory.h"
extern "C" {
#include "hpm_encoder.h"
}


bool draco::StdioFileReader::registered_in_factory_ =draco::FileReaderFactory::RegisterReader(draco::StdioFileReader::Open);
bool draco::StdioFileWriter::registered_in_factory_ = draco::FileWriterFactory::RegisterWriter(draco::StdioFileWriter::Open);
Options::Options()
    : is_point_cloud(false),
    pos_quantization_bits(11),
    tex_coords_quantization_bits(10),
    tex_coords_deleted(false),
    normals_quantization_bits(8),
    normals_deleted(false),
    generic_quantization_bits(8),
    generic_deleted(false),
    compression_level(7),
    preserve_polygons(false),
    use_metadata(false) {}

Options_hpm::Options_hpm()
    :has_map(false),
    q(nullptr),
    w(nullptr),
    h(nullptr),
    input(nullptr),
    output(nullptr),
    config(nullptr),
    yuv(nullptr) {}

int StringToInt(const std::string& s) {
    char* end;
    return strtol(s.c_str(), &end, 10);  
}
void Usage() {
    printf("Usage: avs_mesh_encoder\n");
    printf("Main options:\n");
    printf("  -h | -?                           show help.\n");
    printf("  -iMesh <input mesh>              *input mesh file name.\n");
    printf("  -iMap <input map>                 input texture map file name.\n");
    printf("  -oMesh <output mesh>              output mesh bitstream.\n");
    printf("  -oMap <output map>                output texture map bitstream.\n");
    printf("  -o <output bitstream>             output total bitstream.\n");
    printf("  -encodeMap <flag>                 encode texture map(1) or not(0). default=0\n");
    printf("---------------------draco parameters---------------------------\n");
    printf("  -point_cloud                      forces the input to be encoded as a point "
           "cloud.\n");
    printf("  -draco_qp <value>                 quantization bits for the position "
           "attribute, default=11.\n");
    printf("  -draco_qt <value>                 quantization bits for the texture coordinate "
           "attribute, default=10.\n");
    printf("  -draco_qn <value>                 quantization bits for the normal vector "
           "attribute, default=8.\n");
    printf("  -draco_qg <value>                 quantization bits for any generic attribute, "
           "default=8.\n");
    printf("  -draco_cl <value>                 compression level [0-10], most=10, least=0, "
           "default=7.\n");
    printf("  --skip ATTRIBUTE_NAME             skip a given attribute (NORMAL, TEX_COORD, "
           "GENERIC)\n");
    printf("  --metadata                        use metadata to encode extra information in "
           "mesh files.\n");
    // Mesh with polygonal faces loaded from OBJ format is converted to triangular
    // mesh and polygon reconstruction information is encoded into a new generic
    // attribute.
    printf("  -preserve_polygons                encode polygon info as an attribute.\n");
    printf("Use negative quantization values to skip the specified attribute\n");
    printf("---------------------hpm parameters---------------------------\n");
    printf("  -hpm_cfg <hpm config>             hpm encode config path.\n");
    printf("  -hpm_q <qp>                       hpm qp.\n");
    printf("  -hpm_w <width>                    texture map width.\n");
    printf("  -hpm_h <height>                   texture map height.\n");
    printf("  -hpm_yuv <yuv file>               texture map yuv path.\n");


}
std::string removeExtension(const std::string filename) {
    std::string file = filename;
    size_t dotPos = file.rfind('.');
    return file.substr(0, dotPos);
}

char* removeExtension(char* filename) {
    char* file = filename;
    std::string strFilename(file);
    size_t dotPos = strFilename.rfind('.');
  
    std::string result = strFilename.substr(0, dotPos);

    char* newFilename = new char[result.size() + 1];
    std::strcpy(newFilename, result.c_str());
    return newFilename;
}
bool load_options(int argc, char** argv, Options &options, Options_hpm &options_hpm, char* &outputPath) {
    const int argc_check = argc - 1;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp("-h", argv[i]) || !strcmp("-?", argv[i])) {
            Usage();
            return 0;
        }
        else if (!strcmp("-iMesh", argv[i]) && i < argc_check) {
            options.input = argv[++i];
        }
        else if (!strcmp("-iMap", argv[i]) && i < argc_check) {
            options_hpm.input = argv[++i];
            options_hpm.has_map = true;
        }
        else if (!strcmp("-hpm_cfg", argv[i]) && i < argc_check) {
            options_hpm.config = argv[++i];
        }
        else if (!strcmp("-hpm_q", argv[i]) && i < argc_check) {
            options_hpm.q = argv[++i];
            int qp = std::stoi(options_hpm.q);
            if (qp < 0 || qp>63) {
                printf(
                    "Error: The maximum number of quantization bits for the video encoder "
                    " is 63.\n");
                return false;
            }
        }
        else if (!strcmp("-hpm_w", argv[i]) && i < argc_check) {
            options_hpm.w = argv[++i];
        }
        else if (!strcmp("-hpm_h", argv[i]) && i < argc_check) {
            options_hpm.h = argv[++i];
        }
        else if (!strcmp("-oMesh", argv[i]) && i < argc_check) {
            options.output = argv[++i];
        }
        else if (!strcmp("-oMap", argv[i]) && i < argc_check) {
            options_hpm.output = argv[++i];
        }
        else if (!strcmp("-hpm_yuv", argv[i]) && i < argc_check) {
            options_hpm.yuv = argv[++i];
        }
        else if (!strcmp("-outputPath", argv[i]) && i < argc_check) {
            outputPath = argv[++i];
        }
        else if (!strcmp("-encodeMap", argv[i]) && i < argc_check) {
            int encodeMap = StringToInt(argv[++i]);
            if (encodeMap == 1) {
                options_hpm.has_map = true;
            }
        }
        else if (!strcmp("-point_cloud", argv[i])) {
            options.is_point_cloud = true;
        }
        else if (!strcmp("-draco_qp", argv[i]) && i < argc_check) {
            options.pos_quantization_bits = StringToInt(argv[++i]);
            if (options.pos_quantization_bits > 30) {
                printf(
                    "Error: The maximum number of quantization bits for the position "
                    "attribute is 30.\n");
                return -1;
            }
        }
        else if (!strcmp("-draco_qt", argv[i]) && i < argc_check) {
            options.tex_coords_quantization_bits = StringToInt(argv[++i]);
            if (options.tex_coords_quantization_bits > 30) {
                printf(
                    "Error: The maximum number of quantization bits for the texture "
                    "coordinate attribute is 30.\n");
                return -1;
            }
        }
        else if (!strcmp("-draco_qn", argv[i]) && i < argc_check) {
            options.normals_quantization_bits = StringToInt(argv[++i]);
            if (options.normals_quantization_bits > 30) {
                printf(
                    "Error: The maximum number of quantization bits for the normal "
                    "attribute is 30.\n");
                return -1;
            }
        }
        else if (!strcmp("-draco_qg", argv[i]) && i < argc_check) {
            options.generic_quantization_bits = StringToInt(argv[++i]);
            if (options.generic_quantization_bits > 30) {
                printf(
                    "Error: The maximum number of quantization bits for generic "
                    "attributes is 30.\n");
                return -1;
            }
        }
        else if (!strcmp("-draco_cl", argv[i]) && i < argc_check) {
            options.compression_level = StringToInt(argv[++i]);
        }
        else if (!strcmp("--skip", argv[i]) && i < argc_check) {
            if (!strcmp("NORMAL", argv[i + 1])) {
                options.normals_quantization_bits = -1;
            }
            else if (!strcmp("TEX_COORD", argv[i + 1])) {
                options.tex_coords_quantization_bits = -1;
            }
            else if (!strcmp("GENERIC", argv[i + 1])) {
                options.generic_quantization_bits = -1;
            }
            else {
                printf("Error: Invalid attribute name after --skip\n");
                return -1;
            }
            ++i;
        }
        else if (!strcmp("--metadata", argv[i])) {
            options.use_metadata = true;
        }
        else if (!strcmp("-preserve_polygons", argv[i])) {
            options.preserve_polygons = true;
        }
    }
    if (argc < 3 || options.input.empty()) {
        std::cout << "argc < 3 or options.input is empty" << std::endl;
        Usage();
        return false;
    }
	return true;
}


void RGBToYUV420(const unsigned char* rgb_data, unsigned char* y_plane,
    unsigned char* u_plane, unsigned char* v_plane, int width, int height) {
    int frameSize = width * height;
    int chromaSize = frameSize / 4;

    const float R2Y = 0.2126f;
    const float G2Y = 0.7152f;
    const float B2Y = 0.0722f;
    const float R2U = -0.1146f;
    const float G2U = -0.3854f;
    const float B2U = 0.5000f;
    const float R2V = 0.5000f;
    const float G2V = -0.4542f;
    const float B2V = -0.0458f;
    std::vector<unsigned char> u_temp(width * height / 4);
    std::vector<unsigned char> v_temp(width * height / 4);


    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = (y * width + x) * 3;
            float r = rgb_data[index + 0];
            float g = rgb_data[index + 1];
            float b = rgb_data[index + 2];

            unsigned char Y = (unsigned char)((R2Y * r + G2Y * g + B2Y * b));
            unsigned char U = (unsigned char)(((R2U * r + G2U * g + B2U * b) + 128));
            unsigned char V = (unsigned char)(((R2V * r + G2V * g + B2V * b) + 128));


            y_plane[y * width + x] = Y;

            if (x % 2 == 0 && y % 2 == 0) {
                int index_uv = (y / 2 * (width / 2) + (x / 2));
                u_temp[index_uv] = U;
                v_temp[index_uv] = V;
            }
        }
    }

    for (int j = 0; j < height / 2; ++j) {
        for (int i = 0; i < width / 2; ++i) {
            int index = j * (width / 2) + i;
            u_plane[index] = u_temp[index];
            v_plane[index] = v_temp[index];
        }
    }
}

char** load_hpm_options(Options_hpm &options_hpm, int* argc_hpm) {
    std::vector<std::string> args;
    if (options_hpm.config) {
        args.push_back("--config");
        args.push_back(options_hpm.config);
    }
    else {
        std::cerr << "have no cfg!" << std::endl;
        exit(-1);
    }

    if (options_hpm.yuv) {
        args.push_back("-i");
        args.push_back(options_hpm.yuv);
    }
    else {
        std::cerr << "have no yuv path!" << std::endl;
        exit(-1);
    }

    if (options_hpm.output) {
        args.push_back("-o");
        args.push_back(options_hpm.output);
    }
    else {
        std::cerr << "have no output path!" << std::endl;
        exit(-1);
    }

    if (options_hpm.q) {
        args.push_back("-q");
        args.push_back(options_hpm.q);
    }
    else {
        std::cerr << "hpm qp error!" << std::endl;
        exit(-1);
    }

    if (options_hpm.w && options_hpm.h) {
        args.push_back("-h");
        args.push_back(options_hpm.h);
        args.push_back("-w");
        args.push_back(options_hpm.w);
    }

    *argc_hpm = args.size()+1;
    char** argv_hpm = new char* [*argc_hpm];
    for (int i = 0; i < *argc_hpm-1; ++i) {
        argv_hpm[i+1] = new char[args[i].size() + 1];
        std::strcpy(argv_hpm[i+1], args[i].c_str());
        //std::cerr << "argv_hpm[" << i << "] = " << argv_hpm[i+1] << std::endl;
    }
    return argv_hpm;
}
void encodeBitstreams(const std::string& bitstream1Path, const std::string& bitstream2Path, const std::string& outputPath) {
    std::ifstream bitstream1(bitstream1Path, std::ios::binary);
    std::vector<char> buffer1((std::istreambuf_iterator<char>(bitstream1)), std::istreambuf_iterator<char>());
    bitstream1.close();
    std::ifstream bitstream2(bitstream2Path, std::ios::binary);
    std::vector<char> buffer2((std::istreambuf_iterator<char>(bitstream2)), std::istreambuf_iterator<char>());
    bitstream2.close();
    std::ofstream outputFile(outputPath, std::ios::binary);

    uint32_t size1 = buffer1.size();
    outputFile.write(reinterpret_cast<const char*>(&size1), sizeof(size1));

   
    outputFile.write(buffer1.data(), buffer1.size());

    
    outputFile.write(buffer2.data(), buffer2.size());

    outputFile.close();
}

bool copyFile(const std::string& sourcePath, const std::string& destinationPath) {
    std::ifstream bitstream1(sourcePath, std::ios::binary);
    std::vector<char> buffer1((std::istreambuf_iterator<char>(bitstream1)), std::istreambuf_iterator<char>());
    bitstream1.close();

    std::ofstream outputFile(destinationPath, std::ios::binary);

    uint32_t size1 = buffer1.size();
    outputFile.write(reinterpret_cast<const char*>(&size1), sizeof(size1));

    outputFile.write(buffer1.data(), buffer1.size());
    outputFile.close();
    return true;
}
int main(int argc, char** argv) {
	std::cout << "avs mesh encoder version = 0.1" << std::endl;
	
	Options options;
	Options_hpm options_hpm;
    char* outputPath = {};

    if (!load_options(argc, argv, options, options_hpm, outputPath)) {
        return -1;
    }
    if (options.output.empty()) {
        options.output = removeExtension(options.input)+"_draco.bin";
    }
    
   

    encoder_draco(options);

    if (outputPath == nullptr&& !options.input.empty()) {
        size_t out_length = removeExtension(options.input).size() + strlen(".bin") + 1;
        char* out = (char*)malloc(out_length);
        if (out != nullptr) {
            strcpy(out, removeExtension(options.input).c_str());
            strcat(out, ".bin");
            outputPath = out;
        }
        
    }
    if (options_hpm.has_map) {
        if (options_hpm.input == nullptr || options_hpm.config == nullptr) {

            std::cout << "Error: Has_Map=1,but has no input png or hpm config!" << std::endl;
            return -1;

        }
        else
        {
            std::cout << "load map to rgb_data" << std::endl;
            int channels;
            int w;
            int h;
            unsigned char* rgb_data = stbi_load(options_hpm.input, &w, &h, &channels, 0);

            if (!rgb_data) {
                std::cerr << "Error: Could not load image." << std::endl;
                return 1;
            }
            std::cout << "rgb_data to yuv420" << std::endl;
            std::vector<unsigned char> y_plane(w * h);
            std::vector<unsigned char> u_plane(w * h / 4);
            std::vector<unsigned char> v_plane(w * h / 4);
            RGBToYUV420(rgb_data, y_plane.data(), u_plane.data(), v_plane.data(), w, h);

            stbi_image_free(rgb_data);
            if (options_hpm.yuv==nullptr) {
                size_t out_length = removeExtension(options.input).size() + strlen(".yuv") + 1;
                char* out = (char*)malloc(out_length);
                if (out != nullptr) {
                    strcpy(out, removeExtension(options.input).c_str());
                    strcat(out, ".yuv");
                    options_hpm.yuv = out;
                }
            }
            if (options_hpm.output == nullptr) {
                size_t out_length = removeExtension(options.input).size() + strlen("_hpm.bin") + 1;
                char* out = (char*)malloc(out_length);
                if (out != nullptr) {
                    strcpy(out, removeExtension(options.input).c_str());
                    strcat(out, "_hpm.bin");
                    options_hpm.output = out;
                }
            }
            std::ofstream file(options_hpm.yuv, std::ios::binary);
            file.write(reinterpret_cast<const char*>(y_plane.data()), y_plane.size());
            file.write(reinterpret_cast<const char*>(u_plane.data()), u_plane.size());
            file.write(reinterpret_cast<const char*>(v_plane.data()), v_plane.size());
            file.close();


            char** argv_hpm = new char* [20];
            int argc_hpm = 1;
            argv_hpm = load_hpm_options(options_hpm, &argc_hpm);
            encoder_hpm(argc_hpm, argv_hpm);
            if (outputPath != nullptr) {
                encodeBitstreams(options.output, options_hpm.output, outputPath);
            }
            
           
        }


    }
    else {
        if (outputPath!=nullptr&&copyFile(options.output, outputPath)) {
            std::cout << "File saved to:" << outputPath<< std::endl;
        }
        
    }

	return 0;
}