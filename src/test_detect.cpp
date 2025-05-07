#include "process_img.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.jpg> [output.jpg]"
                  << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = (argc > 2) ? argv[2] : "result.jpg";

    cv::Mat inputImage = cv::imread(inputFile, cv::IMREAD_GRAYSCALE);
    if (inputImage.empty()) {
        std::cerr << "Cannot open " << inputFile << std::endl;
        return 1;
    }

    SegmentResult result = detect_lines(inputImage);

    if (!cv::imwrite(outputFile, result.image)) {
        std::cerr << "Could not save result to " << outputFile << std::endl;
        return 1;
    }
    std::cout << "Saved result to " << outputFile << std::endl;

    return 0;
}
