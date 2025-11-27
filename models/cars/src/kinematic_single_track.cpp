
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <utility>

#include "base.h"
#include "macros.h"

// ============================================================
//  Model metadata
// ============================================================

#undef PARAM_LIST
#define PARAM_LIST(X) \
  PARAM_DOUBLE_ENTRY(X, v_max,             "v_max",             30.0,  0.0,   100.0) \
  PARAM_DOUBLE_ENTRY(X, steering_delay,    "steering_delay",    0.0,   0.0,   1.0) \
  PARAM_DOUBLE_ENTRY(X, drivetrain_delay,  "drivetrain_delay",  0.0,   0.0,   1.0) \
  PARAM_DOUBLE_ENTRY(X, steering_rack_ratio, "steering_rack_ratio", (1/10.0), 0.0, 1.0)

#undef SETTING_LIST
#define SETTING_LIST(SETTING, OPTION) \
  SETTING_ENUM_ENTRY(SETTING, OPTION, \
    steering_input_mode, "steering_input_mode", \
      OPTION_ENTRY(OPTION, "angle") \
      OPTION_ENTRY(OPTION, "rate")  \
  )

static constexpr double steering_wheel_angle_max = 3.5;
static constexpr double steering_wheel_rate_max = 24.0;
static constexpr double ax_max = 30.0;

#undef INPUT_LIST
#define INPUT_LIST(X) \
  INPUT_DOUBLE_ENTRY(X, steering_wheel_angle_input, "steering_wheel_angle_input", -steering_wheel_angle_max, steering_wheel_angle_max) \
  INPUT_DOUBLE_ENTRY(X, steering_wheel_rate_input,  "steering_wheel_rate_input",  -steering_wheel_rate_max, steering_wheel_rate_max) \
  INPUT_DOUBLE_ENTRY(X, ax_input,             "ax",             -ax_max, ax_max)

#undef STATE_LIST
#define STATE_LIST(X) \
  STATE_DOUBLE_ENTRY(X, ax,             "ax",             -ax_max, ax_max) \
  STATE_DOUBLE_ENTRY(X, steering_wheel_angle, "steering_wheel_angle", -steering_wheel_angle_max, steering_wheel_angle_max) \
  STATE_DOUBLE_ENTRY(X, steering_wheel_rate, "steering_wheel_rate", -steering_wheel_rate_max, steering_wheel_rate_max) \
  STATE_DOUBLE_ENTRY(X, v,    "v",     0.0,  50.0)
  

DECLARE_MODEL_METADATA()


// ============================================================
// Small helpers
// ============================================================

static inline double wrapAngle(double yaw) {
  // Wrap to (-pi, pi]
  constexpr double k_pi = 3.14159265358979323846;
  while (yaw > k_pi)  yaw -= 2.0 * k_pi;
  while (yaw <= -k_pi) yaw += 2.0 * k_pi;
  return yaw;
}

// ------------------------------------------------------------
// Generic circular buffer based delay (in samples)
// ------------------------------------------------------------

struct DelayBuffer {
  std::vector<double> data;
  std::size_t write_idx{0};

  void configure(std::size_t delay_steps) {
    // We store delay_steps+1 samples; output is "oldest" sample.
    std::size_t size = delay_steps + 1;
    if (size == 0) size = 1;
    data.assign(size, 0.0);
    write_idx = 0;
  }

  bool empty() const {
    return data.empty();
  }

  double step(double value) {
    if (data.empty()) {
      return value;
    }

    data[write_idx] = value;
    write_idx = (write_idx + 1) % data.size();

    // Oldest sample is now at write_idx
    return data[write_idx];
  }
};

// ------------------------------------------------------------
// Steering dynamics with optional rate mode + delay
// ------------------------------------------------------------

struct SteeringDynamics {
  bool use_rate{false};
  double steering_wheel_angle{0.0};
  double steering_wheel_rate{0.0};
  double front_wheel_angle{0.0};
  double wheel_to_front_ratio{1.0};
  double front_angle_min{-1.0};
  double front_angle_max{1.0};
  double wheel_angle_min{-1.0};
  double wheel_angle_max{1.0};
  DelayBuffer delay;

  void configure(double dt,
                 double delay_sec,
                 bool use_rate_input,
                 double ratio,
                 double min_front,
                 double max_front,
                 double min_wheel,
                 double max_wheel) {
    assert(ratio > 0.0 && "steering_rack_ratio must be positive");
    assert(min_front <= max_front && "front wheel limits invalid");
    assert(min_wheel <= max_wheel && "steering wheel limits invalid");
    use_rate = use_rate_input;
    wheel_to_front_ratio = ratio;
    front_angle_min = min_front;
    front_angle_max = max_front;
    wheel_angle_min = min_wheel;
    wheel_angle_max = max_wheel;
    steering_wheel_angle = 0.0;
    steering_wheel_rate = 0.0;
    front_wheel_angle = 0.0;

    std::size_t steps = 0;
    if (dt > 0.0 && delay_sec > 0.0) {
      steps = static_cast<std::size_t>(std::llround(delay_sec / dt));
    }
    delay.configure(steps);
  }

  double step(double steering_wheel_angle_input,
              double steering_wheel_rate_input,
              double dt) {
    double commanded_wheel_angle = steering_wheel_angle;

    if (use_rate) {
      commanded_wheel_angle += steering_wheel_rate_input * dt;
    } else {
      commanded_wheel_angle = steering_wheel_angle_input;
    }

    commanded_wheel_angle =
        std::clamp(commanded_wheel_angle, wheel_angle_min, wheel_angle_max);

    double target_front_angle = commanded_wheel_angle * wheel_to_front_ratio;
    double delayed_front = delay.step(target_front_angle);
    front_wheel_angle =
        std::clamp(delayed_front, front_angle_min, front_angle_max);

    double previous_wheel_angle = steering_wheel_angle;
    double filtered_wheel_angle =
        front_wheel_angle / wheel_to_front_ratio;
    steering_wheel_angle =
        std::clamp(filtered_wheel_angle, wheel_angle_min, wheel_angle_max);
    steering_wheel_rate =
        (dt > 0.0) ? (steering_wheel_angle - previous_wheel_angle) / dt : 0.0;

    return front_wheel_angle;
  }

  double frontAngle() const { return front_wheel_angle; }
  double wheelAngle() const { return steering_wheel_angle; }
  double wheelRate() const { return steering_wheel_rate; }
};

// ------------------------------------------------------------
// Drivetrain dynamics with simple delay on ax
// ------------------------------------------------------------

struct DrivetrainDynamics {
  DelayBuffer delay;

  void configure(double dt, double delay_sec) {
    std::size_t steps = 0;
    if (dt > 0.0 && delay_sec > 0.0) {
      steps = static_cast<std::size_t>(std::llround(delay_sec / dt));
    }
    delay.configure(steps);
  }

  double step(double ax_input) {
    return delay.step(ax_input);
  }
};

// ============================================================
// CarModel implementation
// ============================================================

struct CarModel {
  // Descriptor exposed via C ABI
  CarModelDescriptor desc{};

  // Value storage (sizes from macros/metadata)
  std::array<double,  P_COUNT> param_values{};
  std::array<int32_t, S_COUNT> setting_values{};
  std::array<double,  I_COUNT> input_values{};
  std::array<double,  ST_COUNT> state_values{};

  // Cached parameters (updated on reset)
  double wheelbase;
  double v_max;
  double steering_delay;
  double drivetrain_delay;
  // Continuous state
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  
  double ax_observed{0.0};

  // Actuation dynamics
  SteeringDynamics   steering_dyn;
  DrivetrainDynamics drivetrain_dyn;

  // Helper to read the steering mode setting
  bool useSteeringRate() const {
    static_assert(S_COUNT > 0, "This model requires at least 1 setting.");
    static_assert(S_steering_input_mode < S_COUNT, "Missing setting: steering_input_mode.");
    // We know we defined steering_input_mode as the only setting
    int32_t mode = setting_values[S_steering_input_mode];
    return (mode == 1); // 0: "angle", 1: "rate"
  }

  void updateCachedParams() {
    static_assert(P_COUNT > 0, "This model requires at least 1 parameter.");
    static_assert(P_wheelbase         < P_COUNT); 
    static_assert(P_v_max             < P_COUNT);
    static_assert(P_steering_delay    < P_COUNT);
    static_assert(P_drivetrain_delay  < P_COUNT);
    wheelbase        = param_values[P_wheelbase];
    v_max            = param_values[P_v_max];
    steering_delay   = param_values[P_steering_delay];
    drivetrain_delay = param_values[P_drivetrain_delay];
  }

  void writeStateToArray() {
    static_assert(ST_COUNT > 0, "This model requires at least 1 state.");
    static_assert(ST_x              < ST_COUNT);
    static_assert(ST_y              < ST_COUNT);
    static_assert(ST_yaw            < ST_COUNT);
    static_assert(ST_wheel_fl_angle < ST_COUNT);
    static_assert(ST_wheel_fr_angle < ST_COUNT);
    static_assert(ST_steering_wheel_angle < ST_COUNT);
    static_assert(ST_steering_wheel_rate < ST_COUNT);
    static_assert(ST_ax             < ST_COUNT);
    static_assert(ST_v              < ST_COUNT);
    
    state_values[ST_x]              = x;
    state_values[ST_y]              = y;
    state_values[ST_yaw]            = yaw;
    double front = steering_dyn.frontAngle();
    state_values[ST_wheel_fl_angle] = front;
    state_values[ST_wheel_fr_angle] = front;
    state_values[ST_steering_wheel_angle] = steering_dyn.wheelAngle();
    state_values[ST_steering_wheel_rate] = steering_dyn.wheelRate();
    state_values[ST_ax]             = ax_observed;
    state_values[ST_v]              = v;
  }

  std::pair<double, double> frontAngleLimits(double ratio) const {
    static_assert(ST_wheel_fl_angle < ST_COUNT);
    static_assert(ST_wheel_fr_angle < ST_COUNT);
    double min_fl = g_state_min[ST_wheel_fl_angle];
    double min_fr = g_state_min[ST_wheel_fr_angle];
    double max_fl = g_state_max[ST_wheel_fl_angle];
    double max_fr = g_state_max[ST_wheel_fr_angle];
    double min_front_state = std::max(min_fl, min_fr);
    double max_front_state = std::min(max_fl, max_fr);
    assert(min_front_state <= max_front_state && "Wheel angle limits do not overlap");

    auto [wheel_min, wheel_max] = steeringWheelAngleLimits();
    double min_front_from_wheel = std::min(wheel_min, wheel_max) * ratio;
    double max_front_from_wheel = std::max(wheel_min, wheel_max) * ratio;

    double min_front = std::max(min_front_state, min_front_from_wheel);
    double max_front = std::min(max_front_state, max_front_from_wheel);
    assert(min_front <= max_front && "Front limits invalid after applying rack ratio");
    return {min_front, max_front};
  }

  std::pair<double, double> steeringWheelAngleLimits() const {
    static_assert(ST_steering_wheel_angle < ST_COUNT);
    double min_wheel = g_state_min[ST_steering_wheel_angle];
    double max_wheel = g_state_max[ST_steering_wheel_angle];
    assert(min_wheel <= max_wheel && "Steering wheel angle limits invalid");
    return {min_wheel, max_wheel};
  }
};

// ============================================================
// C ABI functions
// ============================================================

CarModel* car_model_create(double dt) {
  auto* m = new CarModel();

  // Hook up the descriptor to our metadata + value storage
  m->desc.num_params    = P_COUNT;
  m->desc.param_names   = g_param_names;
  m->desc.param_min     = g_param_min;
  m->desc.param_max     = g_param_max;
  m->desc.param_values  = m->param_values.data();

  m->desc.num_settings  = S_COUNT;
  m->desc.setting_names = g_setting_names;
  m->desc.setting_values = S_COUNT > 0 ? m->setting_values.data() : nullptr;

  m->desc.num_setting_options           = g_num_setting_options;
  m->desc.setting_option_setting_index  = get_setting_option_indices().data();
  m->desc.setting_option_names          = g_setting_option_names;

  m->desc.num_inputs    = I_COUNT;
  m->desc.input_names   = g_input_names;
  m->desc.input_min     = g_input_min;
  m->desc.input_max     = g_input_max;
  m->desc.input_values  = I_COUNT > 0 ? m->input_values.data() : nullptr;

  m->desc.num_states    = ST_COUNT;
  m->desc.state_names   = g_state_names;
  m->desc.state_min     = g_state_min;
  m->desc.state_max     = g_state_max;
  m->desc.state_values  = ST_COUNT > 0 ? m->state_values.data() : nullptr;

  // Initialize param values with defaults
  for (std::size_t i = 0; i < P_COUNT; ++i) {
    m->param_values[i] = g_param_default[i];
  }

  // Initialize settings to option 0 by default
  for (std::size_t i = 0; i < S_COUNT; ++i) {
    m->setting_values[i] = 0;
  }

  // Zero inputs and states
  for (std::size_t i = 0; i < I_COUNT; ++i) {
    m->input_values[i] = 0.0;
  }
  for (std::size_t i = 0; i < ST_COUNT; ++i) {
    m->state_values[i] = 0.0;
  }

  // Finalize internal state via reset
  car_model_reset(m, dt);

  return m;
}

void car_model_destroy(CarModel* model) {
  delete model;
}

const CarModelDescriptor* car_model_get_descriptor(CarModel* model) {
  return &model->desc;
}

const char* car_model_get_name(void) {
  return "Kinematic Single Track";
}

void car_model_reset(CarModel* model, double dt) {
  if (!model) return;

  // Update cached params from param_values[]
  model->updateCachedParams();

  auto extractStateValue = [&](StateIndex idx) -> double {
      if (idx >= 0 && idx < ST_COUNT) {
          return model->state_values[idx];
      }
      return 0.0;
  };

  double init_x = extractStateValue(ST_x);
  double init_y = extractStateValue(ST_y);
  double init_yaw = extractStateValue(ST_yaw);

  if (!std::isfinite(init_x)) init_x = 0.0;
  if (!std::isfinite(init_y)) init_y = 0.0;
  if (!std::isfinite(init_yaw)) init_yaw = 0.0;

  // Reset physical state to requested pose
  model->x   = init_x;
  model->y   = init_y;
  model->yaw = init_yaw;
  model->v   = 0.0;
  
  model->ax_observed = 0.0;

  // Configure dynamics based on params/settings
  auto [wheel_min, wheel_max] = model->steeringWheelAngleLimits();
  double rack_ratio = model->param_values[P_steering_rack_ratio];
  bool use_rate = model->useSteeringRate();
  auto [front_min, front_max] = model->frontAngleLimits(rack_ratio);
  model->steering_dyn.configure(dt,
                                model->steering_delay,
                                use_rate,
                                rack_ratio,
                                front_min,
                                front_max,
                                wheel_min,
                                wheel_max);
  model->drivetrain_dyn.configure(dt, model->drivetrain_delay);

  // Write initial states
  model->writeStateToArray();
}

void car_model_step(CarModel* model, double dt) {
  if (!model) return;

  // Read inputs
  static_assert(I_COUNT > 0, "This model requires at least 1 input.");
  static_assert(I_steering_wheel_angle_input < I_COUNT);
  static_assert(I_steering_wheel_rate_input < I_COUNT);
  static_assert(I_ax_input              < I_COUNT);
    
  double wheel_angle_in = model->input_values[I_steering_wheel_angle_input];
  double wheel_rate_in  = model->input_values[I_steering_wheel_rate_input];
  double ax_in             = model->input_values[I_ax_input];

  // Actuator dynamics (get delayed steering angle and ax)
  double front_wheel_angle =
      model->steering_dyn.step(wheel_angle_in, wheel_rate_in, dt);
  double ax    = model->drivetrain_dyn.step(ax_in);
  model->ax_observed = ax;

  // Kinematic bicycle integration
  double x     = model->x;
  double y     = model->y;
  double yaw   = model->yaw;
  double v     = model->v;
  double L     = model->wheelbase;

  double dx    = v * std::cos(yaw);
  double dy    = v * std::sin(yaw);
  double dyaw  = (L > 0.0) ? (v / L) * std::tan(front_wheel_angle) : 0.0;
  double dv    = ax;

  x   += dt * dx;
  y   += dt * dy;
  yaw += dt * dyaw;
  v   += dt * dv;

  // Clamp velocity
  v = std::clamp(v, 0.0, model->v_max);

  // Store back
  model->x   = x;
  model->y   = y;
  model->yaw = wrapAngle(yaw);
  model->v   = v;

  // Update exportable state array
  model->writeStateToArray();
}
