#include <luxi/lift_protocol.hpp>
#include <fstream>
#include <iostream>
int main(int argc, char** argv) {
  if(argc!=2) return 2;
  std::ifstream in(argv[1]); long long first, received, end; int age, ticks=0, rejected=0, invalid=0;
  while(in >> first >> received >> age >> end) {
    luxi::lift::Sample s; s.first_ns=first; s.received_ns=received; s.feedback_age_ms=age;
    s.flags=51; s.guard=16; s.fault=0;
    for(auto t=received; t<end; t+=10000000) { ++ticks; if(!s.healthy(t)) ++rejected; if(!s.valid(t)) ++invalid; }
    if(s.healthy(first+301000000)) return 3;
    s.feedback_age_ms=501; if(s.healthy(received)) return 4;
    s.feedback_age_ms=0; s.fault=1; if(s.healthy(received)) return 5;
    s.fault=0; s.flags=128; if(s.healthy(received)) return 6;
  }
  std::cout << "control_ticks=" << ticks << " watchdog_rejections=" << rejected << " strict_measurement_rejections=" << invalid << std::endl;
  return (!ticks || rejected || !invalid) ? 1 : 0;
}
