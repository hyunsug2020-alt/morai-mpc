#include <cassert>
#include <cmath>
#include <vector>
#include "moraimpc/controller_consistency.hpp"
using namespace moraimpc;
int main() {
    LtvParams p; p.cfg.N=12; p.cfg.Ts=.1; p.cfg.target_vel=55./3.6;
    p.curve_speed_alpha=0.; p.curve_alat_max=4.2;
    p.pred_accel_mps2=3.; p.pred_decel_mps2=2.5;
    p.cfg.curve_lookahead_m=70.; p.curve_lookahead_max_m=70.;
    assert(!p.junction_curve_profile);
    std::vector<double> x(201),y(201),h(201),k(201),s(201);
    std::vector<int> gear(201,1);
    for(int i=0;i<201;++i) {
        s[i]=.5*i; k[i]=s[i]>=45. ? .03245 : 0.;
        if(i) { h[i]=h[i-1]+.25*(k[i-1]+k[i]);
          const double yaw=.5*(h[i-1]+h[i]);x[i]=x[i-1]+.5*std::cos(yaw);y[i]=y[i-1]+.5*std::sin(yaw); }
    }
    auto run=[&](int idx,double cap,bool reverse=false,bool parking=false) {
      LTVModel model(p.cfg); LTVCost cost(p.cfg); LTVSolver solver(p.cfg); LtvPersistent state;
      LtvInput in; in.path={&x,&y,&h,&k,&s,0,200};in.gear=&gear;
      in.nearest_idx=idx;in.vehicle_s=s[idx];in.ego_x=x[idx];in.ego_y=y[idx];in.ego_yaw=h[idx];in.ego_v=48./3.6;
      in.wheel.valid=true;in.wheel.delta_rad=std::atan(3.9*k[idx]);in.wheel.reason="ok";
      in.planner_cap_fresh=true;in.planner_cap_mps=cap;in.cur_gear=reverse?-1:1;in.parking_mode=parking;
      return ltvStep(p,in,state,model,cost,solver);
    };
    const auto far=run(0,55./3.6);
    assert(std::abs(far.num.at("v_target0_mps")-55./3.6)<1e-6);
    const auto near=run(90,55./3.6);
    assert(near.num.at("v_target0_mps")<=std::sqrt(4.2/.03245)+1e-6);
    const auto cap=run(0,20./3.6);
    assert(cap.num.at("v_target0_mps")<=20./3.6+1e-6);
    const auto stop=run(0,0.);
    assert(stop.num.at("v_target0_mps")==0.);
}
