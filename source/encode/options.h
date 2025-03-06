#include <iostream>

struct Options {
	Options();
	bool is_point_cloud;
	int pos_quantization_bits;
	int tex_coords_quantization_bits;
	bool tex_coords_deleted;
	int normals_quantization_bits;
	bool normals_deleted;
	int generic_quantization_bits;
	bool generic_deleted;
	int compression_level;
	bool preserve_polygons;
	bool use_metadata;
	std::string input;
	std::string output;
};

struct Options_hpm {
	Options_hpm();
	bool has_map;
	char* q;
	char* w;
	char* h;
	char* input;
	char* output;
	char* config;
	char* yuv;
	
};
