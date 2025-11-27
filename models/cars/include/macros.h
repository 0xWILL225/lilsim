#pragma once

#include "base.h"   // must define CarModel, CarModelDescriptor, CarStateTag
#include <cstdint>

// ============================================================================
//  X-MACRO FRAMEWORK FOR MODEL METADATA
//
//  Usage pattern in a model .cpp:
//
//    #include "macros.h"
//
//    // 1) Define lists for this model:
//
//    #define PARAM_LIST(X)                                    \
//      PARAM_DOUBLE_ENTRY(X, wheelbase, "wheelbase",          \
//                                 2.8, 0.5, 5.0)                     \
//      PARAM_DOUBLE_ENTRY(X, v_max, "v_max",                  \
//                                 30.0, 0.0, 100.0)                  \
//      PARAM_DOUBLE_ENTRY(X, dt, "dt",                        \
//                                 0.01, 0.0001, 0.1)
//
//    #define SETTING_LIST(SETTING, OPTION)                    \
//      SETTING_ENUM_ENTRY(SETTING, OPTION,                    \
//          steering_input_mode, "steering_input_mode",               \
//          OPTION_ENTRY(OPTION, "angle")                      \
//          OPTION_ENTRY(OPTION, "rate")                       \
//      )
//
//    #define INPUT_LIST(X)                                    \
//      INPUT_DOUBLE_ENTRY(X, steering_angle, "steering_angle", -1.0, 1.0) \
//      INPUT_DOUBLE_ENTRY(X, steering_rate,  "steering_rate",  -5.0, 5.0) \
//      INPUT_DOUBLE_ENTRY(X, ax,             "ax",            -10.0, 10.0)
//
//    #define STATE_LIST(X)                                    \
//      STATE_DOUBLE_ENTRY(X, x,    "x",   -100.0, 100.0, CAR_STATE_TAG_POSE_X) \
//      STATE_DOUBLE_ENTRY(X, y,    "y",   -100.0, 100.0, CAR_STATE_TAG_POSE_Y) \
//      STATE_DOUBLE_ENTRY(X, yaw,  "yaw", -3.14,  3.14,  CAR_STATE_TAG_POSE_YAW)
//
//    // 2) Expand them into enums/arrays:
//    DECLARE_MODEL_METADATA()
//
//    // 3) Implement your logic struct using the generated enums/arrays.
//    //    You can also add inline accessors yourself, e.g.:
//    //    inline double& param_wheelbase(CarModel& m) {
//    //        return m.param_values[P_wheelbase];
//    //    }
//
//  This header *only* handles the repetitive metadata generation
//  (enums, names, min/max, tags). The actual CarModel layout and ABI
//  glue can be built on top, using these enums/arrays.
// ============================================================================


// ============================
//  Helper entry macros
// ============================
//
// These are just convenience wrappers so your lists read nicely.
// They simply forward to X / SETTING / OPTION.

// PARAM: continuous double parameter
//  X(NAME, STR, DEFAULT, MINV, MAXV)
#define PARAM_DOUBLE_ENTRY(X, NAME, STR, DEFAULT, MINV, MAXV) \
    X(NAME, STR, DEFAULT, MINV, MAXV)

// SETTING: discrete enum-like parameter
//  SETTING(NAME, STR, OPTIONS_BLOCK)
//  OPTIONS_BLOCK is a sequence of OPTION_ENTRY(OPTION, "optname")
#define SETTING_ENUM_ENTRY(SETTING, OPTION, NAME, STR, OPTIONS_BLOCK) \
    SETTING(NAME, STR, OPTIONS_BLOCK)

// One option inside a setting
//  OPTION(OPTSTR)
#define OPTION_ENTRY(OPTION, OPTSTR) \
    OPTION(OPTSTR)

// INPUT: continuous double input
//  X(NAME, STR, MINV, MAXV)
#define INPUT_DOUBLE_ENTRY(X, NAME, STR, MINV, MAXV) \
    X(NAME, STR, MINV, MAXV)

// STATE: continuous double state with semantic tag
//  X(NAME, STR, MINV, MAXV, TAG)
#define STATE_DOUBLE_ENTRY(X, NAME, STR, MINV, MAXV, TAG) \
    X(NAME, STR, MINV, MAXV, TAG)


// ============================
//  User must define these 4 in each model TU:
//
//    #define PARAM_LIST(X)              ...
//    #define SETTING_LIST(SETTING,OPTION) ...
//    #define INPUT_LIST(X)              ...
//    #define STATE_LIST(X)              ...
//
//  If a model doesn’t use one category, you can define it as empty:
//    #define PARAM_LIST(X)
// ============================


// ============================
//  Metadata generation macros
// ============================
//
//  DECLARE_MODEL_METADATA()
//    - generates:
//        enum ParamIndex, SettingIndex, InputIndex, StateIndex
//        param_names/min/max/default arrays
//        setting_names, setting_option_names, setting_option_setting_index
//        input_names/min/max arrays
//        state_names/min/max/tags arrays
//
//  Everything is at TU scope (global symbols).
// ============================

#define DECLARE_MODEL_METADATA()                                     \
                                                                            \
/* ---------- Param enums & arrays ---------- */                            \
                                                                            \
/* enum ParamIndex { P_NAME, ... , P_COUNT }; */                            \
#define INTERNAL_PARAM_ENUM(NAME, STR, DEF, MINV, MAXV) P_##NAME,    \
enum ParamIndex { PARAM_LIST(INTERNAL_PARAM_ENUM) P_COUNT };  \
#undef INTERNAL_PARAM_ENUM                                           \
                                                                            \
/* const char* param_names[] = { "name", ... }; */                          \
#define INTERNAL_PARAM_NAME(NAME, STR, DEF, MINV, MAXV) STR,         \
static const char* const g_param_names[] = {                         \
    PARAM_LIST(INTERNAL_PARAM_NAME)                           \
};                                                                          \
#undef INTERNAL_PARAM_NAME                                           \
                                                                            \
/* double param_min[] / param_max[] / param_default[]; */                   \
#define INTERNAL_PARAM_MIN(NAME, STR, DEF, MINV, MAXV) MINV,         \
static const double g_param_min[] = {                                \
    PARAM_LIST(INTERNAL_PARAM_MIN)                            \
};                                                                          \
#undef INTERNAL_PARAM_MIN                                            \
                                                                            \
#define INTERNAL_PARAM_MAX(NAME, STR, DEF, MINV, MAXV) MAXV,         \
static const double g_param_max[] = {                                \
    PARAM_LIST(INTERNAL_PARAM_MAX)                            \
};                                                                          \
#undef INTERNAL_PARAM_MAX                                            \
                                                                            \
#define INTERNAL_PARAM_DEF(NAME, STR, DEF, MINV, MAXV) DEF,          \
static const double g_param_default[] = {                            \
    PARAM_LIST(INTERNAL_PARAM_DEF)                            \
};                                                                          \
#undef INTERNAL_PARAM_DEF                                            \
                                                                            \
                                                                            \
/* ---------- Setting enums & arrays ---------- */                          \
                                                                            \
/* enum SettingIndex { S_NAME, ..., S_COUNT }; */                           \
#define INTERNAL_SETTING_ENUM(NAME, STR, OPTS) S_##NAME,             \
enum SettingIndex {                                                         \
    SETTING_LIST(INTERNAL_SETTING_ENUM, OPTION_ENTRY)  \
    S_COUNT                                                                 \
};                                                                          \
#undef INTERNAL_SETTING_ENUM                                         \
                                                                            \
/* const char* setting_names[] = { "steering_input_mode", ... }; */         \
#define INTERNAL_SETTING_NAME(NAME, STR, OPTS) STR,                  \
static const char* const g_setting_names[] = {                       \
    SETTING_LIST(INTERNAL_SETTING_NAME, OPTION_ENTRY)  \
};                                                                          \
#undef INTERNAL_SETTING_NAME                                         \
                                                                            \
/* Flatten options:                                                          \
 *   setting_option_names[] = { "angle", "rate", ... }                      \
 *   setting_option_setting_index[] = { S_steering_input_mode, ... }        \
 */                                                                         \
                                                                            \
/* First, collect all option names */                                       \
#define INTERNAL_SETTING_COLLECT_OPTIONS(NAME, STR, OPTS) OPTS       \
#define INTERNAL_OPTION_NAME(OPTSTR) OPTSTR,                         \
static const char* const g_setting_option_names[] = {                \
    SETTING_LIST(INTERNAL_SETTING_COLLECT_OPTIONS,            \
                        INTERNAL_OPTION_NAME)                        \
};                                                                          \
#undef INTERNAL_OPTION_NAME                                          \
#undef INTERNAL_SETTING_COLLECT_OPTIONS                              \
                                                                            \
/* Then, map each option back to its owning setting index */                \
                                                                            \
/* We need to re-expand OPTS, this time emitting the owning S_NAME */       \
#define INTERNAL_SETTING_OPTION_BLOCK(NAME, STR, OPTS)               \
    INTERNAL_SETTING_OPTION_EXPAND(NAME, OPTS)                       \
                                                                            \
#define INTERNAL_SETTING_OPTION_EXPAND(NAME, OPTS) OPTS              \
                                                                            \
/* When OPTION(OPTSTR) is expanded inside a given SETTING(NAME,...),        \
 *   we output the index S_NAME.                                            \
 */                                                                         \
#define INTERNAL_OPTION_OWNER(OPTSTR) S_##NAME,                      \
static const int32_t g_setting_option_setting_index[] = {            \
    SETTING_LIST(INTERNAL_SETTING_OPTION_BLOCK,               \
                        INTERNAL_OPTION_OWNER)                       \
};                                                                          \
#undef INTERNAL_OPTION_OWNER                                         \
#undef INTERNAL_SETTING_OPTION_EXPAND                                \
#undef INTERNAL_SETTING_OPTION_BLOCK                                 \
                                                                            \
/* Number of options */                                                     \
static const int32_t g_num_setting_options =                         \
    static_cast<int32_t>(                                                   \
        sizeof(g_setting_option_names) /                             \
        sizeof(g_setting_option_names[0])                            \
    );                                                                      \
                                                                            \
                                                                            \
/* ---------- Input enums & arrays ---------- */                            \
                                                                            \
/* enum InputIndex { I_NAME, ..., I_COUNT }; */                             \
#define INTERNAL_INPUT_ENUM(NAME, STR, MINV, MAXV) I_##NAME,         \
enum InputIndex {                                                           \
    INPUT_LIST(INTERNAL_INPUT_ENUM)                           \
    I_COUNT                                                                 \
};                                                                          \
#undef INTERNAL_INPUT_ENUM                                           \
                                                                            \
/* const char* input_names[] = { ... }; */                                  \
#define INTERNAL_INPUT_NAME(NAME, STR, MINV, MAXV) STR,              \
static const char* const g_input_names[] = {                         \
    INPUT_LIST(INTERNAL_INPUT_NAME)                           \
};                                                                          \
#undef INTERNAL_INPUT_NAME                                           \
                                                                            \
/* input_min / input_max */                                                 \
#define INTERNAL_INPUT_MIN(NAME, STR, MINV, MAXV) MINV,              \
static const double g_input_min[] = {                                \
    INPUT_LIST(INTERNAL_INPUT_MIN)                            \
};                                                                          \
#undef INTERNAL_INPUT_MIN                                            \
                                                                            \
#define INTERNAL_INPUT_MAX(NAME, STR, MINV, MAXV) MAXV,              \
static const double g_input_max[] = {                                \
    INPUT_LIST(INTERNAL_INPUT_MAX)                            \
};                                                                          \
#undef INTERNAL_INPUT_MAX                                            \
                                                                            \
                                                                            \
/* ---------- State enums & arrays ---------- */                            \
                                                                            \
/* enum StateIndex { ST_NAME, ..., ST_COUNT }; */                           \
#define INTERNAL_STATE_ENUM(NAME, STR, MINV, MAXV, TAG) ST_##NAME,   \
enum StateIndex {                                                           \
    STATE_LIST(INTERNAL_STATE_ENUM)                           \
    ST_COUNT                                                                \
};                                                                          \
#undef INTERNAL_STATE_ENUM                                           \
                                                                            \
/* const char* state_names[] = { ... }; */                                  \
#define INTERNAL_STATE_NAME(NAME, STR, MINV, MAXV, TAG) STR,         \
static const char* const g_state_names[] = {                         \
    STATE_LIST(INTERNAL_STATE_NAME)                           \
};                                                                          \
#undef INTERNAL_STATE_NAME                                           \
                                                                            \
/* state_min / state_max */                                                 \
#define INTERNAL_STATE_MIN(NAME, STR, MINV, MAXV, TAG) MINV,         \
static const double g_state_min[] = {                                \
    STATE_LIST(INTERNAL_STATE_MIN)                            \
};                                                                          \
#undef INTERNAL_STATE_MIN                                            \
                                                                            \
#define INTERNAL_STATE_MAX(NAME, STR, MINV, MAXV, TAG) MAXV,         \
static const double g_state_max[] = {                                \
    STATE_LIST(INTERNAL_STATE_MAX)                            \
};                                                                          \
#undef INTERNAL_STATE_MAX                                            \
                                                                            \
/* state_tags[] */                                                          \
#define INTERNAL_STATE_TAG(NAME, STR, MINV, MAXV, TAG) TAG,          \
static const CarStateTag g_state_tags[] = {                          \
    STATE_LIST(INTERNAL_STATE_TAG)                            \
};                                                                          \
#undef INTERNAL_STATE_TAG                                            \
                                                                            \
/* Done: ParamIndex/SettingIndex/InputIndex/StateIndex enums exist,         \
 * and these arrays are available:                                          \
 *   g_param_names/min/max/default                                  \
 *   g_setting_names                                                \
 *   g_setting_option_names                                         \
 *   g_setting_option_setting_index                                 \
 *   g_input_names/min/max                                          \
 *   g_state_names/min/max/tags                                     \
 */                                                                         \
/* End of DECLARE_MODEL_METADATA */                                  \
/* (No trailing semicolon on purpose) */



// ============================================================================
// NOTE:
//  This header intentionally does NOT define CarModel or the C ABI functions.
//  It only generates the enums and metadata arrays. In your model plugin you
//  can:
//
//    - Define struct CarModel { ... std::vector<double> param_values; ... }
//    - Use ParamIndex/SettingIndex/InputIndex/StateIndex for indexing.
//    - Use the g_* arrays to fill your CarModelDescriptor in
//      car_model_create().
//    - Implement your ModelLogic::reset/step using those indices.
// ============================================================================

