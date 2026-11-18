#include "../src/static_position_compensation.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  using Comp = luxi::StaticPositionCompensation;
  Comp c;
  Comp::Axes nominal{}, position{}, velocity{}, lower{}, upper{}, speed{};
  lower.fill(-1); upper.fill(1); speed.fill(.02);
  // A simple static load produces a constant equilibrium deflection under PD.
  // All 14 arm axes must converge in both directions; both grippers stay raw.
  Comp::Axes output{}, load{};
  for (int i=0;i<16;++i) load[i]=(i%2 ? -.06 : .06);
  for (int tick=0;tick<1200;++tick) {
    for(int i=0;i<16;++i) position[i]=output[i]-load[i];
    const auto before=c.bias;
    output=c.apply(nominal,position,velocity,lower,upper,speed,.01,true);
    for(int i=0;i<16;++i) {
      require(std::abs(c.bias[i]-before[i]) <= .000200001,"compensation exceeds requested/axis rate");
      require(output[i]>=lower[i] && output[i]<=upper[i],"compensation crossed original limits");
      if(i%8==7) require(c.bias[i]==0 && output[i]==nominal[i],"gripper compensation must remain exactly zero");
    }
  }
  for(int i=0;i<16;++i) if(i%8!=7) require(std::abs(position[i]-nominal[i])<.0031,"static load error failed to converge");
  // Invalid/stale feedback, active movement and large following error must not
  // accumulate an integral. Existing support bias remains across handover.
  const auto held=c.bias;
  for(int t=0;t<100;++t)c.apply(nominal,position,velocity,lower,upper,speed,.01,false);
  require(c.bias==held,"disabled learning changed a held compensation");
  velocity.fill(std::numeric_limits<double>::quiet_NaN());
  output=c.apply(nominal,position,velocity,lower,upper,speed,.01,true);
  require(c.bias==held,"invalid velocity changed the existing support bias");
  for(int i=0;i<16;++i)require(std::abs(output[i]-nominal[i]-held[i])<1e-12,"invalid feedback removed existing support");
  c.reset(); position.fill(-.1); velocity.fill(.1);
  for(int t=0;t<500;++t)c.apply(nominal,position,velocity,lower,upper,speed,.01,true);
  for(double v:c.bias)require(v==0,"compensation learned during motion");
  velocity.fill(0); position.fill(-.3);
  for(int t=0;t<500;++t)c.apply(nominal,position,velocity,lower,upper,speed,.01,true);
  for(double v:c.bias)require(v==0,"compensation integrated a large following error");
  // Near a limit, the integral saturates at available room and does not wind up.
  nominal.fill(.99); position.fill(.93);
  for(int t=0;t<2000;++t)output=c.apply(nominal,position,velocity,lower,upper,speed,.01,true);
  for(int i=0;i<16;++i) {
    require(output[i]<=1,"upper-limit anti-windup failed");
    if(i%8!=7)require(std::abs(c.bias[i]-.01)<1e-9,"integral exceeded remaining limit room");
  }
  c.reset(); nominal.fill(-.99);position.fill(-.93);
  for(int t=0;t<2000;++t)output=c.apply(nominal,position,velocity,lower,upper,speed,.01,true);
  for(int i=0;i<16;++i)require(output[i]>=-1,"lower-limit anti-windup failed");
  c.reset();for(double v:c.bias)require(v==0,"disable reset failed");
  std::cout<<"PASS: 14-axis signed load convergence, zero gripper bias, per-axis rate bound, motion/error freeze, anti-windup and reset\n";
}
