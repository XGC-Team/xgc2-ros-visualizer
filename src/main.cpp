#include "server/server.hpp"
#include "server/routes.hpp"
#include "layout/rviz_layout.hpp"
#include <xgc2/xrpc/unix.hpp>
#include <ros/ros.h>
#include <algorithm>
#include <exception>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
namespace {
volatile std::sig_atomic_t stop_requested=0;
void requestStop(int) { stop_requested=1; }
void requireAllocatedDirectory(const char* name) {
  const char* value=std::getenv(name);
  if(!value||!*value||value[0]!='/')
    throw std::invalid_argument(std::string(name)+" requires an explicit absolute supervisor allocation");
}
struct Options {
  std::string socket_path;
  std::size_t workers{2};
  std::string world_clock{"wall"};
  std::vector<std::string> ros_arguments;
  bool has_sim_time_remap{false};
};
std::size_t workerCount(const std::string& value) {
  if(value.empty()||value.size()>2||value.find_first_not_of("0123456789")!=std::string::npos||std::stoul(value)<1||std::stoul(value)>32)
    throw std::invalid_argument("--callback-workers must be an integer within 1..32");
  return std::stoul(value);
}
Options parseOptions(int argc,char** argv) {
  Options options;
  std::vector<std::string> selected;
  for(int i=1;i<argc;++i) {
    const std::string argument=argv[i];
    if(argument.compare(0,2,"--")==0) {
      if(std::find(selected.begin(),selected.end(),argument)!=selected.end())
        throw std::invalid_argument("startup option occurs more than once: "+argument);
      selected.push_back(argument);
      if(argument!="--socket-path"&&argument!="--callback-workers"&&argument!="--world-clock")
        throw std::invalid_argument("unknown startup option: "+argument);
      if(++i==argc)throw std::invalid_argument("startup option requires a value: "+argument);
      const std::string value=argv[i];
      if(argument=="--socket-path")options.socket_path=value;
      else if(argument=="--callback-workers")options.workers=workerCount(value);
      else options.world_clock=value;
    } else {
      // Only standard ROS remappings may follow the product options.
      if(argument.empty()||argument.find(":=")==std::string::npos||
          (argument[0]=='_'&&argument.compare(0,2,"__")!=0))
        throw std::invalid_argument("product startup uses explicit --options; only ROS remappings may follow");
      if(argument.compare(0,2,"__")==0) {
        const auto name=argument.substr(0,argument.find(":="));
        const std::vector<std::string> allowed{"__name","__ns","__master","__ip","__hostname"};
        if(std::find(allowed.begin(),allowed.end(),name)==allowed.end())
          throw std::invalid_argument("unsupported ROS startup override; log writes use allocated ROS_LOG_DIR");
      }
      if(argument.compare(0,15,"/use_sim_time:=")==0)options.has_sim_time_remap=true;
      options.ros_arguments.push_back(argument);
    }
  }
  if(options.socket_path.empty())throw std::invalid_argument("--socket-path is required");
  if(options.world_clock!="simulation"&&options.world_clock!="wall")
    throw std::invalid_argument("--world-clock must be simulation or wall");
  return options;
}
}
int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--prepare-rviz") {
      xgc2_ros_visualizer::prepareRvizArtifact(std::cin, std::cout);
      return 0;
    }
    const auto options=parseOptions(argc,argv);
    // ROS writes its cache and logs only where the supervisor allocated them.
    requireAllocatedDirectory("ROS_HOME");requireAllocatedDirectory("ROS_LOG_DIR");
    // Initialize names without writing parameters or starting the ROS graph.
    // The master watcher installs the immutable clock when the master appears.
    std::vector<std::string> arguments{argv[0]};
    arguments.insert(arguments.end(),options.ros_arguments.begin(),options.ros_arguments.end());
    if(!options.has_sim_time_remap)arguments.push_back("/use_sim_time:=~use_sim_time");
    std::vector<char*> pointers;
    for(auto& argument:arguments)pointers.push_back(&argument[0]);
    int ros_argc=static_cast<int>(pointers.size());
    ros::init(ros_argc,pointers.data(),"xgc2_ros_visualizer",ros::init_options::NoSigintHandler);
    ros::Time::init();
    std::signal(SIGINT,requestStop);std::signal(SIGTERM,requestStop);
    const std::string instance_id=xgc2::xrpc::new_instance_id();
    xgc2_ros_visualizer::Server server(options.workers,xgc2_ros_visualizer::defaultRates(),options.world_clock,&stop_requested);
    xgc2_ros_visualizer::RpcOptions rpc_options;
    rpc_options.instance_id=instance_id;
    xgc2_ros_visualizer::RpcServer transport(options.socket_path,
        [&server](const std::string& method,const std::string& path,const Json::Value& body){return xgc2_ros_visualizer::routeRpc(server,method,path,body);},
        [&server,&instance_id]{return xgc2_ros_visualizer::describeService(server,instance_id);},
        std::move(rpc_options),[&server]{
          try {server.stop();}
          catch(...) {ros::shutdown();throw;}
          ros::shutdown();
        },[&server]{server.activateRos();});
    server.startMasterWatch([&transport]{transport.notifyNativeReady();});
    transport.run(server.stopping());server.rethrowFailure();return 0;
  } catch(const std::exception& error) {std::cerr<<"visualizer server failed: "<<std::string(error.what()).substr(0,512)<<'\n';return 1;}
}
