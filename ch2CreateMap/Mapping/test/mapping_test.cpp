#include "ndt_mapping.h"

int main(int argc, char **argv)
{ 
    std::vector<cv::String> lidar_pathes;
    cv::glob("../data/rgbcloud/*.txt", lidar_pathes);
    assert( lidar_pathes.size()>=10 );

    NdtMapping mapping_processer;
    mapping_processer->CreateNdtRgbMap(lidar_pathes);

    return 0;
}