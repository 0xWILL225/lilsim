
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

#include "base.h"
#include "macros.h"

// ============================================================
// Model metadata
// ============================================================

#define PARAM_LIST(X) \
  PARAM_DOUBLE_ENTRY(X, wheelbase,         "wheelbase",         2.8,   0.5,   5.0) \
  PARAM_DOUBLE_ENTRY(X, v_max,             "v_max",             30.0,  0.0,   100.0) \
  PARAM_DOUBLE_ENTRY(X, dt,                "dt",                0.01,  0.0001, 0.1) \
  PARAM_DOUBLE_ENTRY(X, steering_delay,    "steering_delay",    0.0,   0.0,   1.0) \
  PARAM_DOUBLE_ENTRY(X, drivetrain_delay,  "drivetrain_delay",  0.0,   0.0,   1.0)

#define SETTING_LIST(SETTING, OPTION) \
  SETTING_ENUM_ENTRY(SETTING, OPTION, \
    steering_input_mode, "steering_input_mode", \
      OPTION_ENTRY(OPTION, "angle") \
      OPTION_ENTRY(OPTION, "rate")  \
  )

#define INPUT_LIST(X) \
  INPUT_DOUBLE_ENTRY(X, steering_angle_input, "steering_angle", -1.0, 1.0) \
  INPUT_DOUBLE_ENTRY(X, steering_rate_input,  "steering_rate",  -5.0, 5.0) \
  INPUT_DOUBLE_ENTRY(X, ax,             "ax",             -10.0, 10.0)

#define STATE_LIST(X) \
  STATE_DOUBLE_ENTRY(X, x,    "x",   -100.0, 100.0, CAR_STATE_TAG_POSE_X) \
  STATE_DOUBLE_ENTRY(X, y,    "y",   -100.0, 100.0, CAR_STATE_TAG_POSE_Y) \
  STATE_DOUBLE_ENTRY(X, yaw,  "yaw",  -3.14,  3.14, CAR_STATE_TAG_POSE_YAW) \
  STATE_DOUBLE_ENTRY(X, steering_angle, "steering_angle", -1.0, 1.0, CAR_STATE_TAG_STEER_ANGLE) \
  STATE_DOUBLE_ENTRY(X, v,    "v",     0.0,  50.0,  CAR_STATE_TAG_UNKNOWN)

DECLARE_MODEL_METADATA()


// ============================================================
// Small helpers
// ============================================================

static inline double wrapAngle(double yaw) {
  // Wrap to (-pi, pi]
  const double pi = 3.14159265358979323846;
  while (yaw > pi)  yaw -= 2.0 * pi;
  while (yaw <= -pi) yaw += 2.0 * pi;
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
  bool use_rate{false};     // true => interpret input as rate, else angle
  double current_angle{0.0};
  DelayBuffer delay;

  void configure(double dt, double delay_sec, bool use_rate_input) {
    use_rate = use_rate_input;
    current_angle = 0.0;

    std::size_t steps = 0;
    if (dt > 0.0 && delay_sec > 0.0) {
      steps = static_cast<std::size_t>(std::llround(delay_sec / dt));
    }
    delay.configure(steps);
  }

  double step(double steering_angle_input, double steering_rate_input, double dt) {
    if (use_rate) {
      current_angle += steering_rate_input * dt;
    } else {
      current_angle = steering_angle_input;
    }

    return delay.step(current_angle);
  }
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
  double wheelbase{2.8};
  double v_max{30.0};
  double dt_param{0.01};
  double steering_delay{0.0};
  double drivetrain_delay{0.0};

  // Continuous state
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};

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
    static_assert(P_dt                < P_COUNT);
    static_assert(P_steering_delay    < P_COUNT);
    static_assert(P_drivetrain_delay  < P_COUNT);

    wheelbase        = param_values[P_wheelbase];
    v_max            = param_values[P_v_max];
    dt_param         = param_values[P_dt];
    steering_delay   = param_values[P_steering_delay];
    drivetrain_delay = param_values[P_drivetrain_delay];
  }

  void writeStateToArray() {
    static_assert(ST_COUNT > 0, "This model requires at least 1 state.");
    static_assert(ST_x              < ST_COUNT);
    static_assert(ST_y              < ST_COUNT);
    static_assert(ST_yaw            < ST_COUNT);
    static_assert(ST_steering_angle < ST_COUNT);
    static_assert(ST_v              < ST_COUNT);
    
    state_values[ST_x]              = x;
    state_values[ST_y]              = y;
    state_values[ST_yaw]            = yaw;
    state_values[ST_steering_angle] = steering_dyn.current_angle;
    state_values[ST_v]              = v;
  }
};

// ============================================================
// C ABI functions
// ============================================================

CarModel* car_model_create(void) {
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
  m->desc.setting_option_setting_index  = g_setting_option_setting_index;
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
  m->desc.state_tags    = g_state_tags;
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
  car_model_reset(m);

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

void car_model_reset(CarModel* model) {
  if (!model) return;

  // Update cached params from param_values[]
  model->updateCachedParams();

  // Reset physical state
  model->x   = 0.0;
  model->y   = 0.0;
  model->yaw = 0.0;
  model->v   = 0.0;

  model->steering_dyn.current_angle = 0.0;

  // Configure dynamics based on params/settings
  bool use_rate = model->useSteeringRate();
  model->steering_dyn.configure(model->dt_param, model->steering_delay, use_rate);
  model->drivetrain_dyn.configure(model->dt_param, model->drivetrain_delay);

  // Write initial states
  model->writeStateToArray();
}

void car_model_step(CarModel* model, double dt) {
  if (!model) return;

  // For now, we trust the model's dt_param for dynamics; you can add
  // an assert/warning if dt differs too much from dt_param.
  double h = model->dt_param > 0.0 ? model->dt_param : dt;

  // Read inputs
  double steering_angle_in = 0.0;
  double steering_rate_in  = 0.0;
  double ax_in             = 0.0;

  static_assert(I_COUNT > 0, "This model requires at least 1 input.");
  static_assert(I_steering_angle_input < I_COUNT);
  static_assert(I_steering_rate_input < I_COUNT);
  static_assert(I_ax              < I_COUNT);
    
  steering_angle_in = model->input_values[I_steering_angle_input];
  steering_rate_in  = model->input_values[I_steering_rate_input];
  ax_in             = model->input_values[I_ax];

  // Actuator dynamics (get delayed steering angle and ax)
  double delta = model->steering_dyn.step(steering_angle_in, steering_rate_in, h);
  double ax    = model->drivetrain_dyn.step(ax_in);

  // Kinematic bicycle integration
  double x     = model->x;
  double y     = model->y;
  double yaw   = model->yaw;
  double v     = model->v;
  double L     = model->wheelbase;

  double dx    = v * std::cos(yaw);
  double dy    = v * std::sin(yaw);
  double dyaw  = (L > 0.0) ? (v / L) * std::tan(delta) : 0.0;
  double dv    = ax;

  x   += h * dx;
  y   += h * dy;
  yaw += h * dyaw;
  v   += h * dv;

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
