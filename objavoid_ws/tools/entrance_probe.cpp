// Legacy binary image/mask framing; JPEG bytes are ignored. Deep mask only.
#include <cstdint>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include "autorace_objavoid_v3_cpp/start_gate.hpp"
using namespace objavoid;
int main(int argc,char **argv) {
  if(argc!=3) return 2;
  const auto c=YAML::LoadFile(argv[2])["objavoid_v3_node"]["ros__parameters"];
  GroundProjection gp(argv[1],c["lane_max_range"].as<double>(),1.5,c["ground_homography"].as<std::vector<double>>());
  const auto scale=c["ground_scale"].as<std::vector<double>>();gp.set_scale(scale[0],scale[1]);
  Params p;
  uint32_t header[4];
  while(std::cin.read(reinterpret_cast<char *>(header),sizeof(header))) {
    const auto width=header[0],height=header[1],jn=header[2],mn=header[3];
    if(jn>10000000 || mn!=width*height || mn>2000000) return 3;
    std::vector<unsigned char> jpeg(jn),maskbytes(mn);
    if(!std::cin.read(reinterpret_cast<char *>(jpeg.data()),jn) ||
       !std::cin.read(reinterpret_cast<char *>(maskbytes.data()),mn)) return 4;
    const cv::Mat mask(height,width,CV_8UC1,maskbytes.data());
    const auto points=gp.yellow_points(mask);
    const auto g=eval_entrance_gate(points,false,p);
    std::cout << "{\"ok\":" << (g.ok?"true":"false") << ",\"left\":" << (g.left?"true":"false")
      << ",\"right\":" << (g.right?"true":"false") << ",\"x\":" << g.x << ",\"center\":" << g.center
      << ",\"yaw\":" << g.yaw << ",\"support\":" << g.support << ",\"points\":[";
    const size_t stride=std::max(size_t(1),points.size()/512);
    bool first=true;
    for(size_t i=0;i<points.size();i+=stride) {
      if(!first) std::cout << ',';first=false;
      std::cout << '[' << points[i].x << ',' << points[i].y << ']';
    }
    std::cout << "]}" << std::endl;
  }
}
