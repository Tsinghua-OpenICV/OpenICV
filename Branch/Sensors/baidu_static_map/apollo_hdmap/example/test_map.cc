
#include "modules/map/hdmap/hdmap.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <cmath>
#include <map>
#include <vector>
#include <memory>
#include <iomanip> // for shared_ptr
#include <iostream>
#include <string>



struct Vector2i {
  int x;
  int y;
};

struct Vector2d {
  double x;
  double y;
};

struct GPS_Coord {
  double lon;
  double lat;
  double height;
  double heading;
  GPS_Coord() {
    lon = 0.0;
    lat = 0.0;
    height = 0.0;
    heading = 0.0;
  }
  bool is_same_point(GPS_Coord& newPoint) {
    return (fabs(newPoint.lon - lon) <= 1e-7) 
        && (fabs(newPoint.lat - lat) <= 1e-7);
  }
};

class Utm {
public:
  Utm();
  apollo::common::PointENU fromLatLon(double latitude, double longitude, 
      int force_zone_number, char force_zone_letter);

private:
  bool inBounds(double x, double lower, double upper); 
  double radians(double deg); 
  int zone_number_to_central_longitude(int zone_number);

private:
  double K0;
  double E ;
  double E2;
  double E3;
  double E_P2 ;

  double SQRT_E ;
  double _E ;
  double _E2;
  double _E3;
  double _E4;
  double _E5;

  double M1 ;
  double M2 ;
  double M3 ;
  double M4 ;

  double P2 ;
  double P3 ;
  double P4 ;
  double P5 ;

  double R ;
  std::string ZONE_LETTERS ;
};

Utm::Utm() {
  K0 = 0.9996;
  E = 0.00669438;
  E2 = E * E;
  E3 = E2 * E;
  E_P2 = E / (1.0 - E);

  SQRT_E = sqrt(1 - E);
  _E = (1 - SQRT_E) / (1 + SQRT_E);
  _E2 = _E * _E;
  _E3 = _E2 * _E;
  _E4 = _E3 * _E;
  _E5 = _E4 * _E;

  M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256);
  M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024);
  M3 = (15 * E2 / 256 + 45 * E3 / 1024);
  M4 = (35 * E3 / 3072);

  P2 = (3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5);
  P3 = (21. / 16 * _E2 - 55. / 32 * _E4);
  P4 = (151. / 96 * _E3 - 417. / 128 * _E5);
  P5 = (1097. / 512 * _E4);

  R = 6378137;
  ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX";
}

apollo::common::PointENU Utm::fromLatLon(double latitude, double longitude, 
    int force_zone_number=50, char force_zone_letter='S') {
  if (!inBounds(latitude, -80.0, 84.0)) {
    std::cout << "latitude out of range (must be between 80 deg S and 84 deg N)" << std::endl;
  }
  if (!inBounds(longitude, -180.0, 180.0)) {
    std::cout << "longitude out of range (must be between 180 deg W and 180 deg E)" << std::endl;
  }

  double lat_rad = radians(latitude);
  double lat_sin = sin(lat_rad);
  double lat_cos = cos(lat_rad);

  double lat_tan = lat_sin / lat_cos;
  double lat_tan2 = lat_tan * lat_tan;
  double lat_tan4 = lat_tan2 * lat_tan2;

  int zone_number = force_zone_number;
  int zone_letter = force_zone_letter;

  double lon_rad = radians(longitude);
  double central_lon = zone_number_to_central_longitude(zone_number);
  double central_lon_rad = radians(central_lon);

  double n = R / sqrt(1 - E * lat_sin * lat_sin);
  double c = E_P2 * lat_cos * lat_cos;

  double a = lat_cos * (lon_rad - central_lon_rad);
  double a2 = a * a;
  double a3 = a2 * a;
  double a4 = a3 * a;
  double a5 = a4 * a;
  double a6 = a5 * a;

  double m = R * (M1 * lat_rad -
            M2 * sin(2 * lat_rad) +
            M3 * sin(4 * lat_rad) -
            M4 * sin(6 * lat_rad));
  apollo::common::PointENU out;
  out.set_x( K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) +
      a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000 );
  out.set_y( K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) +
      a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2))) );
  return out;
}
bool Utm::inBounds(double x, double lower, double upper) {
  return (lower < x) && (x < upper);
}
double Utm::radians(double deg) {
  return (deg / 180.0 * M_PI);
}
int Utm::zone_number_to_central_longitude(int zone_number) {
  return ((zone_number - 1) * 6 - 180 + 3); 
}


bool getGPS (const std::string& dst_file, std::vector<GPS_Coord>& gps) {
  FILE *stream;
  stream = fopen (dst_file.c_str(), "rb");
  fseek(stream, 0, SEEK_END);
  long num = ftell(stream);
  fseek(stream, 0, SEEK_SET);
  num = num - ftell(stream);
  std::cout << "there are " << num << " bytes in the " << dst_file << std::endl;
  GPS_Coord *data = (GPS_Coord*)malloc(num);
  // pointers
  num = fread(data, sizeof(GPS_Coord), num/sizeof(GPS_Coord), stream);
  // gps.resize(num);
  for (long idx=0; idx<num; idx++) {
      gps.emplace_back(data[idx]);
      // STDCOUT("data " << idx << " "<< data[idx].lon << " " << data[idx].lat);
  }
  free(data);
  fclose(stream);
  return true;
}



class CVShow {
public:
  CVShow() {
    grid_width_ = 100.0;
    grid_pixel_ = 500;
    grid_x_ = 2;
    grid_y_ = 2;
    width_ = grid_pixel_ * grid_x_;
    height_ = grid_pixel_ * grid_y_;
    stride_ = grid_width_ / (double)grid_pixel_;
    roi_width_ = 1000;
    roi_height_ = 1000;
    image_ = cv::Mat::zeros(cv::Size(width_, height_), CV_8UC3);

	  cv::namedWindow("Display map", CV_WINDOW_AUTOSIZE);
  }

  ~CVShow() {}

  bool isValid (cv::Point2i& pt) {
    return ( pt.x >= 0 && pt.x < width_ && pt.y >= 0 && pt.y < height_ );
  }

  cv::Point2i mapToImage (apollo::common::PointENU& center, apollo::common::PointENU& pt) {
    cv::Point2i ipt;
    ipt.x = static_cast<int>(std::floor((pt.x() - center.x() + grid_width_) / stride_));
    ipt.y = height_ - 1 - static_cast<int>(std::floor((pt.y() - center.y() + grid_width_) / stride_));
    return ipt;
  }

  void drawPoint(apollo::common::PointENU& center, 
      std::vector<apollo::common::PointENU>& geometry, 
      int radius, const cv::Scalar& color, 
      int thickness = 1, int lineType = cv::LineTypes::LINE_8, int shift = 0) {
    for (int j=0; j<geometry.size(); j++) {
      cv::Point2i ipt = mapToImage(center, geometry[j]);
      if (isValid(ipt)) {
        circle(image_, ipt, radius, color, thickness, lineType, shift);
      }                
    }
  }

  void drawLine(apollo::common::PointENU& center,
      std::vector<apollo::common::PointENU>& geometry, const cv::Scalar& color, 
      int thickness = 1, int lineType = cv::LineTypes::LINE_8, int shift = 0) {
    std::vector<cv::Point2i> ipt; 
    for (int j=0; j<geometry.size(); j++) {
      cv::Point2i ipt_tmp = mapToImage(center, geometry[j]);
      if (isValid(ipt_tmp)) {
        ipt.emplace_back(ipt_tmp);
      }   
    }
    if (ipt.size() > 1) {
      for (int j=0; j<(ipt.size()-1); j++) {
        line(image_, ipt[j], ipt[j+1], color, thickness, lineType, shift);   
      }
    }
  }

  void drawPoly(apollo::common::PointENU& center,
      std::vector<apollo::common::PointENU>& geometry, const cv::Scalar& color, 
      int lineType = cv::LineTypes::LINE_8, int shift = 0) {
    std::vector<cv::Point2i> ipt; 
    for (int j=0; j<geometry.size(); j++) {
      cv::Point2i ipt_tmp = mapToImage(center, geometry[j]);
      if (isValid(ipt_tmp)) {
        ipt.emplace_back(ipt_tmp);
      }   
    }
    cv::fillConvexPoly(image_, ipt.data(), ipt.size(), color, lineType, shift);
  }

  void show() {
    // cv::Mat img;
    // cv::resize(image_, img, cv::Size(width_/16, height_/16));
    // s_img, None, fx=0.1, fy=0.1, interpolation=cv2.INTER_AREA)
    cv::imshow("Display map", image_);
    cv::waitKey(100);
    image_ = cv::Mat::zeros(cv::Size(width_, height_), CV_8UC3);
  }

  // void show(GPS_Coord& gps) {
  //   cv::Point2i ipt = gpsToImage(gps);
  //   if (isValid(ipt)) {
  //     cv::Mat roi;
  //     // double angle = -gps.heading;
  //     // cv::Point2f center(image_merged.cols / 2, image_merged.rows / 2);
  //     // cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1);
  //     // cv::warpAffine(image_merged, image, rot, image_merged.size());
  //     cv::Rect2i rect(std::max(0, (ipt.x - roi_width_>>1)), 
  //         std::max(0, (ipt.y - roi_height_>>1)), 
  //         std::min((ipt.x + roi_width_>>1), width_),
  //         std::min((ipt.y + roi_height_>>1), height_) );
  //     image_(rect).copyTo(roi); 
  //     cv::imshow("Display map", roi);
  //     cv::waitKey(0);
  //   }  
  // }

  void save() {
    std::string path = "../map.jpg";
    cv::imwrite(path, image_);    
  }

private:
  cv::Mat image_;

  int width_;
  int height_;
  double stride_;
  int roi_width_;
  int roi_height_;
  double grid_width_;
  int grid_pixel_;
  int grid_x_;
  int grid_y_;
};



cv::Vec3b getColor(int index) {
  cv::Vec3b color;
  switch(index) {
    case 0: // LaneMarkingColor::COLOR_NONE:
      color[0] = 245;
      color[1] = 245;
      color[2] = 245;
      return color;
    case 1: // LaneMarkingColor::COLOR_YELLOW:
      color[0] = 0;
      color[1] = 255;
      color[2] = 255;
      return color;
    case 2: // LaneMarkingColor::COLOR_WHITE:
      color[0] = 255;
      color[1] = 255;
      color[2] = 255;   
      return color;
    case 3: // LaneMarkingColor::COLOR_BLUE:
      color[0] = 255;
      color[1] = 0;
      color[2] = 0;
      return color;
    case 4: // LaneMarkingColor::COLOR_ORANGE:
      color[0] = 0;
      color[1] = 128;
      color[2] = 255;
      return color;
    case 5: // LaneMarkingColor::COLOR_WHITE_YELLOW:
      color[0] = 132;
      color[1] = 227;
      color[2] = 255;
      return color;
    case 6: // LaneMarkingColor::COLOR_YELLOW_WHITE:
      color[0] = 202;
      color[1] = 232;
      color[2] = 252;
      return color;
    case 7: // LaneMarkingColor::COLOR_YELLOW_WHITE:
      color[0] = 20;
      color[1] = 40;
      color[2] = 252;
      return color;
  }
}

#define STDCOUT(express)  std::cout << express << std::endl
#define SIZECOUT(vec) STDCOUT(#vec << " size : " << vec.size() )
#define TRY_RUN(express)  if ( 0 == (express) ) { \
  std::cout << "run " << #express << " success." << std::endl; \
} else { \
  std::cout << "run " << #express << " failed." << std::endl; \
}

int main (int argc, char** argv) {
  std::cout << "test map" << std::endl;
  if (argc < 3) {
    std::cout << "input map and gps path." << std::endl;
    return -1;
  } 

  std::string map_path(argv[1]);
  std::string gps_path(argv[2]);

  apollo::hdmap::HDMap map;
  map.LoadMapFromFile(map_path);
  std::vector<GPS_Coord> gps;
  getGPS (gps_path, gps);

  apollo::common::PointENU point;
  point.set_x(427292.62534242147);
  point.set_y(4419239.997745479);
  point.set_z(73.4581222534180);

  CVShow show;
  Utm utm;

  const cv::Scalar link_color = getColor(0);  
  const cv::Scalar curb_color = getColor(1);  
  const cv::Scalar lane_color = getColor(2);  
  const cv::Scalar topo_color = getColor(3);  
  const cv::Scalar object_color = getColor(7); 

  int stop_line_num = 0;
  int sig_num = 0;

  for (auto gps_i =0; gps_i<gps.size(); gps_i+=20) {
    auto& g = gps[gps_i];
    auto point = utm.fromLatLon(g.lat, g.lon, 50, 'S');

    std::vector<apollo::hdmap::SignalInfoConstPtr> signals;
    TRY_RUN( map.GetSignals(point, 100.0, &signals) )
    SIZECOUT(signals);
    sig_num += signals.size();

    for (auto& sig: signals) {
      STDCOUT( "signal " << sig->id().id() << " type : " << sig->signal().type() ); 
      std::vector<apollo::common::PointENU> tmp(sig->signal().boundary().point().begin(), 
          sig->signal().boundary().point().end());
      show.drawLine(point, tmp, curb_color, 1, cv::LineTypes::LINE_AA, 0);

      for (auto& sub: sig->signal().subsignal()) {
        STDCOUT( "      " << " sub type : " << sub.type() << " " 
            << std::to_string(sub.location().x()) << " " 
            << std::to_string(sub.location().y()) << " " 
            << std::to_string(sub.location().z()) ); 
      }
      STDCOUT("          stop line size : " << sig->signal().stop_line().size() );
      for (auto& curve: sig->signal().stop_line() ) {
        for (auto& seg: curve.segment()) {
          std::vector<apollo::common::PointENU> tmp(seg.line_segment().point().begin(), 
              seg.line_segment().point().end());
          show.drawLine(point, tmp, 
              object_color, 5, cv::LineTypes::LINE_AA, 0);
          STDCOUT( "stop line :"
            << std::to_string(tmp[0].x()) << " " 
            << std::to_string(tmp[0].y()) << " " 
            << std::to_string(tmp[0].z()) );

        }
      }

    }
    STDCOUT( "signal num " << sig_num ); 


#if 1 
    std::vector<apollo::hdmap::LaneInfoConstPtr> lanes;
    TRY_RUN( map.GetLanes(point, 100.0, &lanes) )
    SIZECOUT(lanes);

    for (auto& lane_ptr: lanes) {
      std::vector<apollo::common::PointENU> poly;
      if(lane_ptr->lane().has_left_boundary()) {
        if (!lane_ptr->lane().left_boundary().virtual_())
          if(lane_ptr->lane().left_boundary().has_curve()) {
            if(!lane_ptr->lane().left_boundary().curve().segment().empty()) {
              for (auto idx=0; 
                  idx<lane_ptr->lane().left_boundary().curve().segment().size(); 
                  idx++) {
                auto psize = lane_ptr->lane().left_boundary().curve().segment().Get(idx).line_segment().point().size();
                for (auto p=psize-1;  p>=0; p--) {
                  auto& pt = lane_ptr->lane().left_boundary().curve().segment().Get(idx).line_segment().point().Get(p); 
                  poly.emplace_back(pt);
                }
              }
            } else {
              STDCOUT("segment empty");
            }
          } else {
            STDCOUT("no curve");
          }
      } else {
        STDCOUT("no left_boundary");
      }
      show.drawLine(point, poly, lane_color, 1, cv::LineTypes::LINE_AA, 0);
      poly.clear();

      if(lane_ptr->lane().has_right_boundary()) {
        // if (!lane_ptr->lane().right_boundary().virtual_()) 
          if(lane_ptr->lane().right_boundary().has_curve()) {
            if(!lane_ptr->lane().right_boundary().curve().segment().empty()) {
              for (auto idx=0; 
                  idx<lane_ptr->lane().right_boundary().curve().segment().size(); 
                  idx++) {
                for (auto p=0; 
                    p<lane_ptr->lane().right_boundary().curve().segment().Get(idx).line_segment().point().size(); 
                    p++) {
                  auto& pt = lane_ptr->lane().right_boundary().curve().segment().Get(idx).line_segment().point().Get(p); 
                  poly.emplace_back(pt);
                }
              }
            } else {
              STDCOUT("segment empty");
            }
          } else {
            STDCOUT("no curve");
          }
      } else {
        STDCOUT("no right_boundary");
      }
      show.drawLine(point, poly, lane_color, 1, cv::LineTypes::LINE_AA, 0);
    }
#endif   

    std::vector<apollo::common::PointENU> ego;
    ego.emplace_back(point);
    show.drawPoint(point, ego, 3, object_color, 4, cv::LineTypes::LINE_AA, 0);

#if 1
    std::vector<apollo::hdmap::RoadInfoConstPtr> roads;
    TRY_RUN( map.GetRoads(point, 100.0, &roads) )
    SIZECOUT(roads);

    for (auto& road: roads) {
      std::vector<apollo::common::PointENU> poly;
      for (auto& sec: road->sections()) {
        for (auto& edge: sec.boundary().outer_polygon().edge()) {
          auto& curve = edge.curve();
          for (auto& seg: curve.segment()) {
            std::vector<apollo::common::PointENU> tmp(seg.line_segment().point().begin(), 
                seg.line_segment().point().end());
            show.drawLine(point, tmp, 
                object_color, 1, cv::LineTypes::LINE_AA, 0);
          }
        }
      }
    }
#endif 

#if 1
    std::vector<apollo::hdmap::JunctionInfoConstPtr> junctions;
    TRY_RUN( map.GetJunctions(point, 100.0, &junctions) )
    for (auto& jun: junctions) {
      std::vector<apollo::common::PointENU> poly;
      std::vector<apollo::common::PointENU> tmp(jun->junction().polygon().point().begin(), 
          jun->junction().polygon().point().end());
      tmp.emplace_back(jun->junction().polygon().point().Get(0));
      show.drawLine(point, tmp, 
          curb_color, 1, cv::LineTypes::LINE_AA, 0);
    }

    std::vector<apollo::hdmap::StopSignInfoConstPtr> stop_signs;
    TRY_RUN( map.GetStopSigns(point, 100.0, &stop_signs) )
    SIZECOUT(stop_signs);
    stop_line_num += stop_signs.size();

    for (auto& stop_line: stop_signs) {
      std::vector<apollo::common::PointENU> poly;
      STDCOUT("stop_line: " << stop_line->id().id());
      STDCOUT("stop_line curve size: " << stop_line->stop_sign().stop_line().size() );
      for (auto& curve: stop_line->stop_sign().stop_line()) {
        for (auto& seg: curve.segment()) {
          STDCOUT("stop_line line_segment size: " 
              << seg.line_segment().point().size() );
          std::vector<apollo::common::PointENU> tmp(seg.line_segment().point().begin(), 
              seg.line_segment().point().end());
          show.drawLine(point, tmp, 
              topo_color, 3, cv::LineTypes::LINE_AA, 0);
        }
      }
    }
#endif
    show.show();    
  }
  return 0;
}
