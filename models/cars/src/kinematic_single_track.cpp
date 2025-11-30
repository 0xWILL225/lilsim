
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <utility>

#include "base.h"
#include "macros.hpp"

// ============================================================
//  Model metadata
// ============================================================

#undef PARAM_LIST
#define PARAM_LIST(X) \
  PARAM_DOUBLE_ENTRY(X, v_max,             "v_max",             30.0,  0.0,   100.0) \
  PARAM_DOUBLE_ENTRY(X, steering_delay,    "steering_delay",    0.0,   0.0,   1.0) \
  PARAM_DOUBLE_ENTRY(X, drivetrain_delay,  "drivetrain_delay",  0.0,   0.0,   1.0) \
  PARAM_DOUBLE_ENTRY(X, steering_rack_ratio, "steering_rack_ratio", 4.5, 0.0, 15.0)

#undef SETTING_LIST
#define SETTING_LIST(SETTING, OPTION) \
  SETTING_ENUM_ENTRY(SETTING, OPTION, \
    steering_input_mode, "steering_input_mode", \
      OPTION_ENTRY(OPTION, "angle") \
      OPTION_ENTRY(OPTION, "rate")  \
  )

static constexpr double steering_steering_wheel_angle_max = 3.0;
static constexpr double steering_wheel_rate_max = 20.0;
static constexpr double ax_max = 30.0;

#undef INPUT_LIST
#define INPUT_LIST(X) \
  INPUT_DOUBLE_ENTRY(X, steering_wheel_angle_input, "steering_wheel_angle_input", -steering_steering_wheel_angle_max, steering_steering_wheel_angle_max) \
  INPUT_DOUBLE_ENTRY(X, steering_wheel_rate_input,  "steering_wheel_rate_input",  -steering_wheel_rate_max, steering_wheel_rate_max) \
  INPUT_DOUBLE_ENTRY(X, ax_input,             "ax",             -ax_max, ax_max)

#undef STATE_LIST
#define STATE_LIST(X) \
  STATE_DOUBLE_ENTRY(X, ax,             "ax",             -ax_max, ax_max) \
  STATE_DOUBLE_ENTRY(X, steering_wheel_angle, "steering_wheel_angle", -steering_steering_wheel_angle_max, steering_steering_wheel_angle_max) \
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
  DelayBuffer delay;

  void configure(double dt,
                 double delay_sec,
                 bool use_rate_input) {
    use_rate = use_rate_input;
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
              double rack_ratio,
              double dt) {
    assert(rack_ratio > 0.0 && "steering_rack_ratio must be positive");

    double commanded_steering_wheel_angle = steering_wheel_angle;

    if (use_rate) {
      double delayed_rate = delay.step(steering_wheel_rate_input);
      commanded_steering_wheel_angle += delayed_rate * dt;
    } else {
      double delayed_angle = delay.step(steering_wheel_angle_input);
      commanded_steering_wheel_angle = delayed_angle;
    }

    const double steering_wheel_angle_min = g_state_min[ST_steering_wheel_angle];
    const double steering_wheel_angle_max = g_state_max[ST_steering_wheel_angle];

    commanded_steering_wheel_angle =
        std::clamp(commanded_steering_wheel_angle, steering_wheel_angle_min, steering_wheel_angle_max);

    const double front_wheel_angle_min = steering_wheel_angle_min / rack_ratio;
    const double front_wheel_angle_max = steering_wheel_angle_max / rack_ratio;

    assert(front_wheel_angle_min <= front_wheel_angle_max &&
           "Front wheel limits invalid after clamping.");

    front_wheel_angle = commanded_steering_wheel_angle / rack_ratio;

    double previous_wheel_angle = steering_wheel_angle;
    double filtered_steering_wheel_angle =
        front_wheel_angle * rack_ratio;
    steering_wheel_angle =
        std::clamp(filtered_steering_wheel_angle, steering_wheel_angle_min, steering_wheel_angle_max);
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
  bool use_rate = model->useSteeringRate();
  model->steering_dyn.configure(dt,
                                model->param_values[P_steering_delay],
                                use_rate);
  model->drivetrain_dyn.configure(dt, model->param_values[P_drivetrain_delay]);

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
      model->steering_dyn.step(wheel_angle_in,
                               wheel_rate_in,
                               model->param_values[P_steering_rack_ratio],
                               dt);
  double ax    = model->drivetrain_dyn.step(ax_in);
  model->ax_observed = ax;

  // Kinematic bicycle integration
  double x     = model->x;
  double y     = model->y;
  double yaw   = model->yaw;
  double v     = model->v;
  double L     = model->param_values[P_wheelbase];

  double dx    = v * std::cos(yaw);
  double dy    = v * std::sin(yaw);
  double dyaw  = (L > 0.0) ? (v / L) * std::tan(front_wheel_angle) : 0.0;
  double dv    = ax;

  x   += dt * dx;
  y   += dt * dy;
  yaw += dt * dyaw;
  v   += dt * dv;

  // Clamp velocity
  v = std::clamp(v, 0.0, model->param_values[P_v_max]);

  // Store back
  model->x   = x;
  model->y   = y;
  model->yaw = wrapAngle(yaw);
  model->v   = v;

  // Update exportable state array
  model->writeStateToArray();
}
