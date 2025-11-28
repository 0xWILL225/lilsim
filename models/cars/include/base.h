#pragma once

// lilsim car model C ABI base
// This header defines the generic interface between the simulator
// and car model plugins. It is C-compatible and can be implemented
// by C, C++, Rust, etc.

#include <stddef.h>   // size_t
#include <stdint.h>   // int32_t

#ifdef __cplusplus
extern "C" {
#endif

// Mandatory canonical parameter and state names
#define WHEELBASE_PARAM_NAME "wheelbase"
#define TRACK_WIDTH_PARAM_NAME "track_width"
#define X_STATE_NAME "x"
#define Y_STATE_NAME "y"
#define YAW_STATE_NAME "yaw"
#define WHEEL_FL_ANGLE_STATE_NAME "wheel_fl_angle"
#define WHEEL_FR_ANGLE_STATE_NAME "wheel_fr_angle"


// ============================================================
// CarModelDescriptor
// ============================================================
//
// All pointers in this struct are owned by the plugin and remain
// valid for the lifetime of the model instance unless documented
// otherwise.
//
// Conceptually, there is a separation between:
//
//  - Metadata (names, min, max, options) -> read-only to sim/viz
//  - Values (param_values, setting_values, input_values, state_values)
//    -> written by sim/GUI, read/written by plugin.
//
// All numeric values are double except settings (int32 indices).

typedef struct CarModelDescriptor {

  // --------------------------
  // Continuous parameters ("params")
  // --------------------------
  //
  // Example: wheelbase, mass, v_max, delays, etc.
  //
  // Size: num_params
  //
  // names[i]       : null-terminated parameter name
  // param_min[i]   : recommended minimum value
  // param_max[i]   : recommended maximum value
  // param_values[i]: current parameter value (mutable)

  size_t      num_params;
  const char* const* param_names;   // [num_params]
  double*            param_min;     // [num_params]
  double*            param_max;     // [num_params]
  double*            param_values;  // [num_params]


  // --------------------------
  // Discrete parameters ("settings")
  // --------------------------
  //
  // Example: steering mode, drivetrain mode, etc.
  //
  // Size: num_settings
  //
  // setting_names[i]    : null-terminated setting name
  // setting_values[i]   : current option index (0..N_i-1)
  //
  // Options are described by a flattened option list:
  //
  // Size: num_setting_options
  //
  // setting_option_setting_index[k] : which setting this option belongs to
  // setting_option_names[k]         : option label (for GUI dropdown)
  //
  // For each setting s in [0, num_settings):
  //   all k where setting_option_setting_index[k] == (int32_t)s
  //   form the list of options for that setting, in the given order.

  size_t      num_settings;
  const char* const* setting_names;    // [num_settings]
  int32_t*          setting_values;    // [num_settings]

  size_t      num_setting_options;
  const int32_t*     setting_option_setting_index; // [num_setting_options]
  const char* const* setting_option_names;         // [num_setting_options]


  // --------------------------
  // Inputs
  // --------------------------
  //
  // Example: steering angle/rate command, ax command, Fx commands, etc.
  //
  // Size: num_inputs
  //
  // input_names[i]   : null-terminated input name
  // input_min[i]     : recommended minimum value
  // input_max[i]     : recommended maximum value
  // input_values[i]  : current input value (written by sim/client)

  size_t      num_inputs;
  const char* const* input_names;   // [num_inputs]
  double*            input_min;     // [num_inputs]
  double*            input_max;     // [num_inputs]
  double*            input_values;  // [num_inputs]


  // --------------------------
  // Observed states
  // --------------------------
  //
  // States are what the simulator publishes into SceneDB and what
  // the Viz module visualizes.
  //
  // Size: num_states
  //
  // state_names[i]   : null-terminated state name
  // state_min[i]     : typical minimum value (for plotting / GUI scaling)
  // state_max[i]     : typical maximum value
  // state_values[i]  : current state value (written by plugin each step)

  size_t      num_states;
  const char* const* state_names;   // [num_states]
  double*            state_min;     // [num_states]
  double*            state_max;     // [num_states]
  double*            state_values;  // [num_states]

} CarModelDescriptor;


// ============================================================
// CarModel opaque type
// ============================================================
//
// The plugin implements the concrete model and manages all memory
// for the descriptor and its arrays. The simulator treats CarModel
// as an opaque handle and only calls the functions below.

typedef struct CarModel CarModel;


// ============================================================
// Plugin entry points
// ============================================================
//
// Each model plugin must export these symbols with C linkage.
// The simulator will dlopen/LoadLibrary the shared object and
// look up these functions by name (or via a small factory layer).
// dt is needed to initialize lagged dynamics, if applicable.

// Create a new model instance.
// Ownership: simulator must eventually call car_model_destroy().
CarModel* car_model_create(double dt);

// Destroy a model instance and free its resources.
void car_model_destroy(CarModel* model);

// Get a pointer to the model's descriptor.
//
// The descriptor and all arrays it points to must remain valid
// for the lifetime of the model instance (or until destroy).
//
// The simulator and viz will:
//
//  - Read metadata (names, min, max, option lists).
//  - Read/write param_values, setting_values, input_values.
//  - Read state_values after each step.
const CarModelDescriptor* car_model_get_descriptor(CarModel* model);

// Get the name of the model.
// The name is used to identify the model in the GUI.
const char* car_model_get_name(void);

// Reset internal state of the model.
//
// Parameters and settings are assumed to already be in desired
// values (param_values / setting_values). The plugin should
// reinitialize its internal state based on these values.
// dt is needed to initialize lagged dynamics, if applicable.
//
// state_values[] may be initialized by the plugin in this call.
void car_model_reset(CarModel* model, double dt);

// Advance the model by dt seconds.
//
// The simulator will:
//  - Ensure input_values[] have been written for this tick.
//  - Then call car_model_step(model, dt).
//
// The plugin must:
//  - Read inputs from input_values[] and any relevant params/settings.
//  - Update its internal state.
//  - Write updated observed states into state_values[].
void car_model_step(CarModel* model, double dt);


#ifdef __cplusplus
} // extern "C"
#endif
