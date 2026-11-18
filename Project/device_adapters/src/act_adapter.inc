// Included inside Adapter. Explicit session-bound ACT control; never enabled at boot.
  const std::array<double,16> act_initial_{{
    .246624,-.086404,-.032235,1.724079,.021553,.531586,.052071,-.793469,
    -.113870,-.006294,.007820,1.639391,-.062753,-.311856,-.161555,-.826658}};
  std::string act_session_, act_stage_ = "idle", act_reason_ = "not_requested";
  int64_t act_heartbeat_ = 0, act_started_ = 0, act_chunk_at_ = 0, act_observation_ = 0;
  uint64_t act_chunk_id_ = 0;
  size_t act_step_ = 0;
  bool act_raw_ = false;
  std::vector<std::array<double,16>> act_steps_;
  luxi::Json act_lift_, act_lift_status_;
  int64_t act_lift_received_ = 0, act_lift_status_received_ = 0;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> act_subscriptions_;

  bool act_owned() const { return act_stage_ == "homing" || act_stage_ == "ready" || act_stage_ == "running"; }
  int64_t act_stamp(const luxi::Json& j, const char* key) const {
    const auto& value = j.at(key);
    return value.is_string() ? std::stoll(value.get<std::string>()) : value.get<int64_t>();
  }
  std::string act_environment_rejection(int64_t now, bool require_height = true,
                                        bool allow_lift_motion = false, bool measurement = false) const {
    try {
      if (!stream_console(now)) return "act_control_authority_invalid";
      if (!luxi::fresh(act_lift_received_,now,300000000)) return "act_lift_publication_stale";
      // Reuse the SDK predicates, including source scan age and brake/motion flags.
      // Do not use the cached 250 ms 'valid' bit as a running watchdog.
      luxi::lift::Sample lift;
      lift.first_ns=act_stamp(act_lift_,"receive_first_mono_ns");
      lift.received_ns=act_stamp(act_lift_,"receive_mono_ns");
      lift.feedback_age_ms=act_lift_.at("feedback_age_ms").get<uint16_t>();
      lift.flags=act_lift_.at("flags").get<uint8_t>();
      lift.fault=act_lift_.at("fault").get<uint16_t>();
      lift.guard=act_lift_.at("guard").get<uint8_t>();
      const auto transport=act_lift_.at("transport").get<std::string>();
      if(transport!="uart" && transport!="can")return "act_lift_transport_invalid";
      lift.uart=transport=="uart";
      lift.speed_rpm=act_lift_.at("speed_rpm").get<int16_t>();
      lift.current_um=act_lift_.at("current_um").get<int32_t>();
      if(lift.fault)return "act_lift_drive_fault";
      if(lift.flags & 128)return "act_lift_motion_inhibited";
      if(!(lift.flags & 1))return "act_lift_position_invalid";
      if(!lift.uart && (lift.guard & 4))return "act_lift_can_bus_off";
      bool startup_grace=false;
      // Same bounded brake-locked startup exception as lift_adapter, only for
      // this task's lift move while the arms hold their completed initial pose.
      if(allow_lift_motion && !measurement && act_stage_=="ready" &&
          luxi::fresh(act_lift_status_received_,now,300000000) &&
          act_lift_status_.value("startup_feedback_grace",false) &&
          act_lift_status_.value("enabled",false) &&
          act_lift_status_.value("external_owner",std::string())=="act-"+act_session_) {
        const auto phase=act_lift_status_.value("phase",std::string());
        startup_grace=(phase=="move_ack" || phase=="moving" || phase=="stopping") &&
            luxi::fresh(act_stamp(act_lift_status_,"receive_mono_ns"),now,300000000) &&
            lift.startup_healthy(now,act_stamp(act_lift_status_,"motion_started_mono_ns"),
                                act_lift_status_.at("startup_origin_um").get<int32_t>());
      }
      if(!lift.healthy(now) && !startup_grace)return "act_lift_feedback_unhealthy";
      if(measurement && !lift.valid(now))return "act_lift_measurement_stale";
      if(!allow_lift_motion && !lift.stationary())return "act_lift_not_stationary";
      if(require_height && std::abs(int64_t(lift.current_um)+64543)>1000)return "act_lift_height_not_ready";
      return {};
    } catch (...) { return "act_environment_feedback_malformed"; }
  }
  void act_end(const std::string& reason, bool fault = false) {
    act_stage_ = fault ? "fault" : "stopped"; act_reason_ = reason; act_steps_.clear();
    // Raw-target hold remains raw: no second compensation after ACT stops.
    if (joint_active()) finish_joint(fault ? "fault" : "stopped",reason,fault);
  }
  bool act_command(const std::string& op, const std::string& detail, std::string& message) {
    try {
      const auto j = luxi::Json::parse(detail);
      const auto session = j.at("session_id").get<std::string>();
      if(session.size()!=32 || session.find_first_not_of("0123456789abcdef")!=std::string::npos ||
          session==std::string(32,'0')) throw std::runtime_error("ACT needs a nonzero 128-bit session ID");
      const auto now = luxi::mono();
      if(op=="act_prepare") {
        if(act_owned()) {
          if(session==act_session_) {message="ACT preparation already accepted";return true;}
          throw std::runtime_error("Another ACT session owns the arms");
        }
        if(const auto rejection=act_environment_rejection(now,false,false,true); !rejection.empty()) throw std::runtime_error(rejection);
        if(!manual_feedback_rejection(now).empty()) throw std::runtime_error("ACT requires all 16 fresh, stationary, in-limit joints");
        if(!joint_command("joint_move",luxi::Json{{"request_id","act-home-"+session},
            {"positions",act_initial_},{"unit","rad"},{"speed_rad_s",.15}}.dump(),message)) return false;
        if(joint_phase_!="preparing") throw std::runtime_error("Consumed ACT session ID; start with a new session");
        act_session_=session;act_stage_="homing";act_reason_="moving_to_training_initial_pose";
        act_heartbeat_=now;act_started_=act_chunk_at_=act_observation_=0;act_chunk_id_=0;act_steps_.clear();
        message="ACT homing accepted; heartbeat lease required";return true;
      }
      if(session!=act_session_) throw std::runtime_error("ACT session does not own this operation");
      if(op=="act_stop") { if(act_owned())act_end("act_client_stop"); message="ACT stopped, retaining support target";return true; }
      if(!act_owned() || !luxi::fresh(act_heartbeat_,now,1000000000))
        throw std::runtime_error("ACT session expired/stopped; explicit new preparation required");
      if(op=="act_heartbeat") {act_heartbeat_=now;message="ACT lease renewed";return true;}
      if(op!="act_begin") throw std::runtime_error("Unknown ACT command");
      if(act_stage_=="running") {message="ACT already running";return true;}
      if(const auto rejection=act_environment_rejection(now,true,false,true); !rejection.empty()) throw std::runtime_error(rejection);
      if(act_stage_!="ready" || joint_phase_!="ready" || !enabled_ ||
          !manual_feedback_rejection(now).empty()) throw std::runtime_error("ACT initial pose is not ready");
      for(size_t i=0;i<16;++i) if(std::abs(state_.position[i]-act_initial_[i])>.03)
        throw std::runtime_error("ACT measured initial pose error exceeds 0.03 rad");
      // Preserve the last hardware support target at handover, then accept already-compensated actions.
      nominal_sent_=sent_;compensation_.reset();act_raw_=true;
      act_stage_="running";act_reason_="waiting_for_first_action";act_started_=act_heartbeat_=now;
      act_chunk_at_=0;act_observation_=0;act_chunk_id_=0;act_step_=0;act_steps_.clear();
      joint_phase_="act";joint_reason_="act_waiting_for_first_action";joint_speed_=.2;
      manual_phase_="moving";manual_reason_="act_owned";
      command_.position=nominal_sent_;command_.phase="act_hold";command_.source_mono_ns=now;
      message="ACT execution armed for this session; all 60 steps of each fresh block, 30 Hz";return true;
    } catch(const std::exception& e) {message=e.what();return false;}
  }
  void act_watchdog(int64_t now) {
    if(!act_owned())return;
    if(!luxi::fresh(act_heartbeat_,now,1000000000)) {act_end("act_heartbeat_expired");return;}
    if(const auto rejection=act_environment_rejection(now,act_stage_=="running",act_stage_=="ready"); !rejection.empty()) {act_end(rejection);return;}
    if(!manual_feedback_fresh(now)) {act_end("act_joint_feedback_invalid",true);return;}
    if(act_stage_=="homing" && joint_phase_=="ready") {
      act_stage_="ready";act_reason_="initial_pose_reached";
    }
    if(act_stage_=="running" && !luxi::fresh(act_chunk_at_ ? act_chunk_at_ : act_started_,now,act_chunk_at_ ? 3000000000LL : 1000000000LL))
      act_end("act_action_timeout");
  }
  void act_accept(const std::string& text) {
    if(act_stage_!="running" || joint_phase_!="act")return;
    auto j=luxi::Json::parse(text,nullptr,false);
    if(!j.is_object() || !j.contains("session_id") || j["session_id"]!=act_session_)return;
    try {
      const auto now=luxi::mono();
      if(const auto rejection=act_environment_rejection(now); !rejection.empty()) throw std::runtime_error(rejection);
      if(!luxi::fresh(act_heartbeat_,now,1000000000)) throw std::runtime_error("act_heartbeat_expired");
      if(j.at("schema")!="luxi_act_action_chunk_v1" || j.at("units")!="rad" ||
          j.at("semantics")!="post_compensation_position_target" ||
          j.at("joint_order")!="right.J1..J7,right.gripper,left.J1..J7,left.gripper" ||
          j.at("step_interval_us")!=33333) throw std::runtime_error("act_invalid_action_contract");
      const auto id=std::stoull(j.at("chunk_id").get<std::string>());
      if(id<=act_chunk_id_)return;
      const auto observation=act_stamp(j,"observation_mono_ns");
      if(observation<act_started_ || observation<=act_observation_ || !luxi::fresh(observation,now,750000000))
        throw std::runtime_error("act_action_observation_stale_or_prestart");
      const auto& rows=j.at("positions");
      if(!rows.is_array() || rows.size()!=60) throw std::runtime_error("act_expected_60x16");
      std::vector<std::array<double,16>> steps;
      for(size_t r=0;r<rows.size();++r) {
        if(!rows[r].is_array() || rows[r].size()!=16)throw std::runtime_error("act_expected_60x16");
        std::array<double,16> q{};
        for(size_t i=0;i<16;++i) {
          if(!rows[r][i].is_number())throw std::runtime_error("act_non_numeric_action");
          q[i]=rows[r][i].get<double>();
          if(!std::isfinite(q[i]))throw std::runtime_error("act_nonfinite_action");
        }
        if(r<60) {
          if(!joint_positions_in_range(q))throw std::runtime_error("act_action_outside_joint_limits");
          const auto& previous=r ? steps.back() : sent_;
          for(size_t i=0;i<16;++i)if(std::abs(q[i]-previous[i])>.25)
            throw std::runtime_error("act_action_jump_exceeds_0.25_rad");
          steps.push_back(q);
        }
      }
      act_steps_=std::move(steps);act_chunk_id_=id;act_observation_=observation;act_chunk_at_=now;act_step_=0;
      act_reason_="executing_60_steps";
    } catch(const std::exception& e) {act_end(e.what(),true);}
  }
  void update_act(int64_t now, double dt) {
    if(act_stage_!="running" || joint_phase_!="act")return;
    if(!manual_feedback_fresh(now) || !joint_positions_in_range(state_.position)) {act_end("act_feedback_invalid",true);return;}
    for(size_t i=0;i<16;++i)if(std::abs(sent_[i]-state_.position[i])>.25) {act_end("act_following_error",true);return;}
    // Consume all 60 predicted steps. Exhaustion holds the last sent target;
    // it never repeats the block during a network/inference gap.
    if(!act_steps_.empty()) {
      const auto index=static_cast<size_t>((now-act_chunk_at_)*30/1000000000LL);
      act_step_=std::min<size_t>(index,60);
      if(index<60) {
        joint_target_=act_steps_[index];
        for(size_t i=0;i<16;++i) {
          const double step=std::min(joint_speed_,cfg_.speed[i])*dt;
          nominal_sent_[i]=sent_[i]+std::clamp(joint_target_[i]-sent_[i],-step,step);
        }
      } else {
        nominal_sent_=sent_;act_reason_="block_exhausted_holding";
      }
    }
    command_.position=nominal_sent_;command_.phase="act_raw_position";command_.source_mono_ns=now;
    joint_reason_=act_reason_;
  }
  void setup_act() {
    act_subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/luxi/act/received_chunk",1,
      [this](std_msgs::msg::String::ConstSharedPtr m){act_accept(m->data);}));
    act_subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/luxi/lift/status",1,
      [this](std_msgs::msg::String::ConstSharedPtr m){auto j=luxi::Json::parse(m->data,nullptr,false);if(j.is_object()){act_lift_status_=j;act_lift_status_received_=luxi::mono();}}));
    act_subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/luxi/lift/state",1,
      [this](std_msgs::msg::String::ConstSharedPtr m){auto j=luxi::Json::parse(m->data,nullptr,false);if(j.is_object()){act_lift_=j;act_lift_received_=luxi::mono();}}));

  }
