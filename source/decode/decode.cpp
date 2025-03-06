

#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <cinttypes>

#include "draco_decoder.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
extern "C" {
#include "hpm_decoder.h"
}

#include "draco/io/stdio_file_reader.h"
#include "draco/io/file_reader_factory.h"
#include "draco/io/stdio_file_writer.h"
#include "draco/io/file_writer_factory.h"
bool draco::StdioFileReader::registered_in_factory_ = draco::FileReaderFactory::RegisterReader(draco::StdioFileReader::Open);
bool draco::StdioFileWriter::registered_in_factory_ = draco::FileWriterFactory::RegisterWriter(draco::StdioFileWriter::Open);

Options::Options() {}
Options_hpm::Options_hpm()
    :has_map(false),
    input(nullptr),
    output_map(nullptr),
    output_yuv(nullptr)
    {}
void Usage() {
    printf("Usage: avs_mesh_decoder \n");
    printf("Main options:\n");
    printf("  -h | -?                   show help.\n");
    printf("  -i <input>               *input bitstream path.\n");
    printf("  -oMesh <output>           output mesh file path.\n");
    printf("  -oYuv <output>            output texture map yuv file path.\n");
    printf("  -oMap <output>            output texture map png file path.\n");
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
bool decodeBitstreams(const std::string& inputPath,  std::string& inputMeshBin,  char* & inputMapBin) {
    
    
    std::ifstream inputFile(inputPath, std::ios::binary);

    uint32_t size1;
    inputFile.read(reinterpret_cast<char*>(&size1), sizeof(size1));

    std::vector<char> buffer1(size1);
    inputFile.read(buffer1.data(), size1);

    std::vector<char> buffer2((std::istreambuf_iterator<char>(inputFile)), std::istreambuf_iterator<char>());
    inputFile.close();

    if (inputMeshBin.empty()) {
        inputMeshBin = removeExtension(inputPath);
        inputMeshBin += "_mesh.bin";
    }
    std::ofstream outputBitstream1(inputMeshBin, std::ios::binary);
    outputBitstream1.write(buffer1.data(), buffer1.size());
    outputBitstream1.close();


    if (buffer2.empty()) {
        return false;
    }
    else {
        if (inputMapBin==nullptr) {
            std::string input = removeExtension(inputPath);
            size_t totalSize = input.length() + std::strlen("_map.bin") + 1;
            inputMapBin = new char[totalSize];
            std::strcpy(inputMapBin, input.c_str());
            std::strcat(inputMapBin, "map.bin");
        }
        std::ofstream outputBitstream2(inputMapBin, std::ios::binary);
        outputBitstream2.write(buffer2.data(), buffer2.size());
        outputBitstream2.close();
        return true;
    }

}

char** load_hpm_options(Options_hpm &options_hpm, int* argc_hpm) {

    int i = *argc_hpm;
    char** argv_hpm = new char* [20];
    if (options_hpm.input!=nullptr) {
        argv_hpm[i] = "-i";
        argv_hpm[++i] = options_hpm.input;
        if (options_hpm.output_map == nullptr) {
            char* mappath = {};
            mappath = removeExtension(options_hpm.input);
            strcat(mappath, ".png");
            options_hpm.output_map = mappath;
        }
        if (options_hpm.output_yuv == nullptr) {
            char* yuvpath = {};
            yuvpath = removeExtension(options_hpm.input);
            strcat(yuvpath, ".yuv");
            options_hpm.output_yuv = yuvpath;

        }
    }else{
        exit(-1);
    }
    
    if (options_hpm.output_yuv != nullptr) {
        argv_hpm[++i] = "-o";
        argv_hpm[++i] = options_hpm.output_yuv;
    }
    *argc_hpm = i + 1;
   
    return argv_hpm;
}
void YUV420ToRGB(unsigned char* yuvBuffer, unsigned char* rgbBuffer, int width, int height) {
    int frameSize = width * height;
    int chromaSize = frameSize / 4;

    unsigned char* yPlane = yuvBuffer;
    unsigned char* uPlane = yuvBuffer + frameSize;
    unsigned char* vPlane = uPlane + chromaSize;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int yIndex = y * width + x;
            int uvIndex = (y / 2) * (width / 2) + (x / 2);

            int Y = yPlane[yIndex];
            int U = uPlane[uvIndex] - 128;
            int V = vPlane[uvIndex] - 128;

            int R = Y + 1.5748 * V;
            int G = Y - 0.1873 * U - 0.4681 * V;
            int B = Y + 1.8556 * U;

            R = std::min(std::max(R, 0), 255);
            G = std::min(std::max(G, 0), 255);
            B = std::min(std::max(B, 0), 255);

            rgbBuffer[3 * yIndex + 0] = R;
            rgbBuffer[3 * yIndex + 1] = G;
            rgbBuffer[3 * yIndex + 2] = B;
        }
    }
}
int main(int argc, char** argv) {
	std::cout << "avs mesh decoder version = 0.1" << std::endl;
    Options options;
    Options_hpm options_hpm;
    const int argc_check = argc - 1;
    std::string inputBin;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp("-h", argv[i]) || !strcmp("-?", argv[i])) {
            Usage();
            return 0;
        }
        else if (!strcmp("-i", argv[i]) && i < argc_check) {
            inputBin = argv[++i];
        }
        else if (!strcmp("-oMap", argv[i]) && i < argc_check) {
            options_hpm.output_map = argv[++i];
        }
        else if (!strcmp("-oYuv", argv[i]) && i < argc_check) {
            options_hpm.output_yuv = argv[++i];
        }
        else if (!strcmp("-oMesh", argv[i]) && i < argc_check) {
            options.output = argv[++i];
        }
    }
    if (inputBin.empty()) {
        Usage();
        return -1;
    }
    if (options.output.empty()) {

    }
    
    options_hpm.has_map = decodeBitstreams(inputBin, options.input, options_hpm.input);
    if (options.input.empty())
    {
        std::cout << "mesh bitstream input error" << std::endl;
        return -1;
    }
    decoder_draco(options);
    std::cout << "mesh decoded to path:" << options.output << std::endl;

    if (options_hpm.has_map) {
        if (options_hpm.input==nullptr) {
            std::cout << "Error: Has_Map=1,but has no input map bitstream!" << std::endl;
            return -1;
        }
        else
        {
            char** argv_hpm = new char* [20];
            int argc_hpm = 1;
            argv_hpm = load_hpm_options(options_hpm, &argc_hpm);
            
            int w = 0;
            int h = 0;
            decoder_hpm(argc_hpm, argv_hpm,&w,&h);
            std::cout << "texture map yuv saved to:" << options_hpm.output_yuv << std::endl;

            char* yuvFilename = options_hpm.output_yuv;
            char* pngFilename = options_hpm.output_map;

            std::ifstream yuvFile(yuvFilename, std::ios::binary);
            if (!yuvFile.is_open()) {
                std::cerr << "Error opening yuv file." << std::endl;
                return 1;
            }

            int frameSize = w * h * 3 / 2;  
            std::vector<unsigned char> yuvBuffer(frameSize);
            std::vector<unsigned char> rgbBuffer(w * h * 3);

            yuvFile.read(reinterpret_cast<char*>(yuvBuffer.data()), frameSize);
            if (!yuvFile) {
                std::cerr << "Error reading YUV file." << std::endl;
                return 1;
            }

            YUV420ToRGB(yuvBuffer.data(), rgbBuffer.data(), w, h);
            if (stbi_write_png(pngFilename, w, h, 3, rgbBuffer.data(), w * 3)) {
                std::cout << "texture map png file saved to: " << pngFilename << std::endl;
            }
            else {
                std::cerr << "Error saving PNG file." << std::endl;
            }
        }


        }

    return 0;
    }
   
