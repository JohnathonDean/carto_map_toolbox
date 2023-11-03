#include "carto_map/node_main.hpp"

void Run() {
  NodeMain map_node;
  map_node.Init();

  ::ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown
}

int main(int argc, char** argv) {

  FLAGS_alsologtostderr = 1;               // 日志同时输出到stderr
  FLAGS_max_log_size = 10;                 // 单个日志文件大小上限（MB）, 如果设置为0将默认为1
  FLAGS_logbufsecs = 0;
  FLAGS_stop_logging_if_full_disk = true;  // 当磁盘写满时，停止输出
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging("carto_map");

  ::ros::init(argc, argv, "carto_map");
  ::ros::start();

  Run();
  ::ros::shutdown();
}
