#include <iostream>

struct Options {
	Options();
	std::string input;
	std::string output;
};

struct Options_hpm {
	Options_hpm();
	bool has_map;
	char* input;
	char* output_yuv;
	char* output_map;
};
