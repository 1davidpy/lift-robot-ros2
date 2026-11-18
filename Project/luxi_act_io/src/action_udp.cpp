#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <set>

using Json = nlohmann::json;
static int64_t mono() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
static uint64_t integer(const unsigned char* p, size_t n) {
  uint64_t v=0; while(n--) v=(v<<8)|*p++; return v;
}
struct Chunk {
  uint64_t id=0, observation=0;
  uint16_t count=0;
  uint32_t dt=0;
  int64_t created=0;
  std::map<uint16_t,std::array<float,16>> steps;
};
struct Session {
  uint64_t high=0;
  int64_t touched=0;
  Chunk pending, completed;
};

class ActionUdp : public rclcpp::Node {
  int fd_=-1;
  std::string allow_ip_;
  std::map<std::string,Session> sessions_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_, status_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
  uint64_t accepted_=0, rejected_=0;
  int64_t age_ns_, assembly_ns_;
  void reply(const sockaddr_in& peer, Json value) {
    value["execution"]="not_confirmed";
    value["mode"]="ingress";
    const auto body=value.dump();
    sendto(fd_, body.data(), body.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&peer), sizeof(peer));
  }
  void consume(const unsigned char* b, size_t n, const sockaddr_in& peer, int64_t now) {
    Json response={{"schema","luxi_act_ack_v1"}};
    auto reject=[&](const std::string& reason){ ++rejected_; response["status"]="rejected"; response["reason"]=reason; reply(peer,response); };
    if(n!=112 || std::memcmp(b,"LACTv1\0\0",8)!=0) { reject("bad_size_or_magic"); return; }
    const auto id=integer(b+24,8), observation=integer(b+32,8);
    const auto index=integer(b+40,2), count=integer(b+42,2), dt=integer(b+44,4);
    std::string session; const char* hex="0123456789abcdef";
    bool any=false; for(int i=8;i<24;++i){ session+=hex[b[i]>>4]; session+=hex[b[i]&15]; any=any||b[i]; }
    response["session_id"]=session; response["chunk_id"]=std::to_string(id); response["step_index"]=index;
    if(!any || !id || !count || count>60 || index>=count || dt!=33333) {reject("invalid_header");return;}
    if(observation==0 || observation>uint64_t(now) || uint64_t(now)-observation>uint64_t(age_ns_)) {
      reject("observation_stale_or_future");return;
    }
    std::array<float,16> values{};
    for(size_t i=0;i<16;++i) {
      const uint32_t bits=integer(b+48+4*i,4); std::memcpy(&values[i],&bits,4);
      if(!std::isfinite(values[i])) {reject("nonfinite_position");return;}
    }
    char address[INET_ADDRSTRLEN]{}; inet_ntop(AF_INET,&peer.sin_addr,address,sizeof(address));
    const auto key=std::string(address)+":"+std::to_string(ntohs(peer.sin_port))+":"+session;
    if(!sessions_.count(key) && sessions_.size()>=32) {reject("session_capacity");return;}
    auto& s=sessions_[key]; s.touched=now;
    if(id<s.high) {reject("old_chunk");return;}
    if(id==s.high && s.completed.id==id) {
      const auto& c=s.completed;
      if(c.observation!=observation || c.count!=count || c.dt!=dt || c.steps.at(index)!=values) {
        reject("conflicting_retransmission");return;
      }
      response["status"]="duplicate"; response["received_steps"]=c.count; reply(peer,response);return;
    }
    if(id>s.high) {s.high=id;s.pending={};s.completed={};}
    auto& c=s.pending;
    if(c.id==0) {
      c.id=id;c.observation=observation;c.count=count;c.dt=dt;c.created=now;
    }
    if(now-c.created>assembly_ns_) {reject("assembly_timeout");return;}
    if(c.observation!=observation || c.count!=count || c.dt!=dt) {reject("chunk_metadata_changed");return;}
    if(c.steps.count(index) && c.steps[index]!=values) {reject("conflicting_retransmission");return;}
    c.steps[index]=values;
    response["received_steps"]=c.steps.size();
    if(c.steps.size()!=count) {response["status"]="partial";reply(peer,response);return;}
    Json positions=Json::array(); for(size_t i=0;i<count;++i) positions.push_back(c.steps.at(i));
    Json chunk={{"schema","luxi_act_action_chunk_v1"},{"session_id",session},{"chunk_id",std::to_string(id)},
      {"observation_mono_ns",std::to_string(observation)},{"received_mono_ns",std::to_string(now)},
      {"step_interval_us",dt},{"positions",positions},{"units","rad"},
      {"joint_order","right.J1..J7,right.gripper,left.J1..J7,left.gripper"},
      {"semantics","post_compensation_position_target"},{"mode","ingress"},
      {"source_ip",address},{"execution","not_confirmed"}};
    std_msgs::msg::String msg;msg.data=chunk.dump();publisher_->publish(msg);
    ++accepted_;s.completed=c;s.pending={};
    response["status"]="accepted"; response["topic"]="/luxi/act/received_chunk";reply(peer,response);
  }
  void poll() {
    // Bound work per timer tick, keeping ROS status and shutdown responsive under load.
    for(int i=0;i<128;++i) {
      std::array<unsigned char,113> bytes{};sockaddr_in peer{};socklen_t size=sizeof(peer);
      auto n=recvfrom(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT|MSG_TRUNC,reinterpret_cast<sockaddr*>(&peer),&size);
      if(n<0) break;
      char address[INET_ADDRSTRLEN]{};inet_ntop(AF_INET,&peer.sin_addr,address,sizeof(address));
      if(!allow_ip_.empty() && allow_ip_!=address) {++rejected_;continue;}
      consume(bytes.data(),size_t(n),peer,mono());
    }
    const auto now=mono();
    for(auto it=sessions_.begin();it!=sessions_.end();) {
      if(now-it->second.touched>10000000000LL) it=sessions_.erase(it); else ++it;
    }
  }
 public:
  ActionUdp():Node("act_action_udp") {
    const auto host=declare_parameter<std::string>("host","0.0.0.0");
    const auto port=declare_parameter<int>("port",18041);
    allow_ip_=declare_parameter<std::string>("allow_ip","");
    const auto age=declare_parameter<int>("max_observation_age_ms",2000);
    const auto assembly=declare_parameter<int>("assembly_timeout_ms",500);
    if(port<1024 || port>65535 || age<1 || assembly<1) throw std::runtime_error("Invalid UDP configuration");
    age_ns_=int64_t(age)*1000000;assembly_ns_=int64_t(assembly)*1000000;
    sockaddr_in bind_address{};bind_address.sin_family=AF_INET;bind_address.sin_port=htons(port);
    if(inet_pton(AF_INET,host.c_str(),&bind_address.sin_addr)!=1) throw std::runtime_error("Numeric IPv4 host required");
    fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(fd_<0) throw std::runtime_error("UDP socket failed");
    if(bind(fd_,reinterpret_cast<sockaddr*>(&bind_address),sizeof(bind_address))!=0) {
      close(fd_);fd_=-1;throw std::runtime_error("UDP bind failed");
    }
    publisher_=create_publisher<std_msgs::msg::String>("/luxi/act/received_chunk",rclcpp::QoS(1).reliable().durability_volatile());
    status_=create_publisher<std_msgs::msg::String>("/luxi/act/ingress_status",rclcpp::QoS(1).transient_local());
    timer_=create_wall_timer(std::chrono::milliseconds(2),[this]{poll();});
    status_timer_=create_wall_timer(std::chrono::seconds(1),[this]{
      std_msgs::msg::String msg;msg.data=Json{{"mode","ingress"},{"execution","not_confirmed"},
        {"accepted_chunks",accepted_},{"rejected_packets",rejected_},{"sessions",sessions_.size()}}.dump();status_->publish(msg);
    });
    RCLCPP_INFO(get_logger(),"Receive-only ACT UDP listening on %s:%d; no motor command publisher",host.c_str(),static_cast<int>(port));
  }
  ~ActionUdp() override {if(fd_>=0)close(fd_);}
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try{auto node=std::make_shared<ActionUdp>();rclcpp::spin(node);}
  catch(const std::exception& e){fprintf(stderr,"%s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
