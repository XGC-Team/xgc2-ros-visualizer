#include <zenoh.h>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
int main(int argc,char** argv) {
  if(argc<3) return 2;
  std::string config="{mode:'peer',listen:{endpoints:[]},connect:{endpoints:['tcp/127.0.0.1:"+std::string(argv[1])+"']},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}";
  z_owned_config_t c; if(zc_config_from_str(&c,config.c_str())<0) return 3;
  z_owned_session_t session; if(z_open(&session,z_move(c),nullptr)<0) return 4;
  std::vector<z_owned_publisher_t> publishers(static_cast<std::size_t>(argc-2));
  for(int i=2;i<argc;++i) {
    z_view_keyexpr_t key; if(z_view_keyexpr_from_str(&key,argv[i])<0 || z_declare_publisher(z_loan(session),&publishers[i-2],z_loan(key),nullptr)<0) return 5;
  }
  std::cout<<"READY"<<std::endl;
  std::string line;
  while(std::getline(std::cin,line)) {
    std::istringstream input(line); std::string command; std::size_t index; input>>command>>index;
    if(index>=publishers.size()) return 6;
    if(command=="status") {
      z_matching_status_t status{}; auto rc=z_publisher_get_matching_status(z_loan(publishers[index]),&status);
      std::cout<<"STATUS "<<static_cast<int>(rc)<<" "<<status.matching<<std::endl;
    } else if(command=="put") {
      std::string hex;input>>hex;std::vector<std::uint8_t> data;
      if(hex.size()%2) return 7;
      for(std::size_t i=0;i<hex.size();i+=2)data.push_back(static_cast<std::uint8_t>(std::strtoul(hex.substr(i,2).c_str(),nullptr,16)));
      z_owned_bytes_t bytes;z_bytes_copy_from_buf(&bytes,data.data(),data.size());
      std::cout<<"PUT "<<static_cast<int>(z_publisher_put(z_loan(publishers[index]),z_move(bytes),nullptr))<<std::endl;
    } else return 8;
  }
  for(auto& p:publishers) z_drop(z_move(p));
  z_drop(z_move(session));
}
