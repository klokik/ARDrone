#include <fstream>
#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

const int diamond_num = 25;
std::string path_prefix = "./models/aruco_diamond1234/";
// std::string path_prefix = "/tmp/";

const int marker_width = 96;
const int square_width = 128;
const int margins = 8;

std::string material_base = "abstract material ar/diamond_base\n\
{\n\
  technique\n\
  {\n\
    pass\n\
    {\n\
      ambient 0.8 0.8 0.8 1.0\n\
      diffuse 0.95 0.95 0.95 1.0\n\
      specular 0.2 0.2 0.2 1.0 12.5\n\
\n\
      texture_unit\n\
      {\n\
        filtering anisotropic\n\
        max_anisotropy 8\n\
        scale 1.0 1.0\n\
        rotate 90\n\
      }\n\
    }\n\
  }\n\
}\n\
";

std::string material_template = "material ar/diamond%d : ar/diamond_base {\n\
  technique {\n\
    pass {\n\
      texture_unit {\n\
        texture diamond_%d_%d_%d.png\n\
}}}}\n\
";

int main(int argc, char **argv)
{
  cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

  std::string material_script = material_base;

  char buf[256];

  for (int i = 0; i < diamond_num*4; i += 4)
  {
    cv::Mat diamond;
    cv::Vec4i ids{i, i+1, i+2, i+3};

    sprintf(buf, material_template.c_str(), i, i, square_width, marker_width);
    material_script.append(buf);

    cv::aruco::drawCharucoDiamond(dict, ids, square_width, marker_width, diamond, margins, 1);

    sprintf(buf, "diamond_%d_%d_%d.png", i, square_width, marker_width);
    auto img_path = path_prefix + "materials/textures/" + buf;
    std::cout << "Writing image " << img_path << std::endl;
    cv::imwrite(img_path, diamond);

    // cv::imshow("diamond", diamond);
    // if (cv::waitKey(500) == 27)
    //   break;
  }

  auto script_path = path_prefix + "materials/scripts/armark.material";
  std::cout << "Writing script " << script_path << std::endl;
  std::ofstream ofs(script_path);
  assert(ofs.is_open());

  ofs << material_script;

  ofs.flush();
  ofs.close();
}