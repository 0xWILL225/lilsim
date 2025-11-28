#pragma once

#include "base.h"   // must define CarModel, CarModelDescriptor
#include <cstdint>
#include <cmath>
#include <vector>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
//  X-MACRO FRAMEWORK FOR MODEL METADATA
// ============================================================================

// ============================
//  Helper entry macros
// ============================

// PARAM: continuous double parameter
#define PARAM_DOUBLE_ENTRY(X, NAME, STR, DEFAULT, MINV, MAXV) \
    X(NAME, STR, DEFAULT, MINV, MAXV)

// SETTING: discrete enum-like parameter
#define SETTING_ENUM_ENTRY(SETTING, OPTION, NAME, STR, OPTIONS_BLOCK) \
    SETTING(NAME, STR, OPTIONS_BLOCK)

// One option inside a setting
#define OPTION_ENTRY(OPTION, OPTSTR) \
    OPTION(OPTSTR)

// INPUT: continuous double input
#define INPUT_DOUBLE_ENTRY(X, NAME, STR, MINV, MAXV) \
    X(NAME, STR, MINV, MAXV)

// STATE: continuous double state
#define STATE_DOUBLE_ENTRY(X, NAME, STR, MINV, MAXV) \
    X(NAME, STR, MINV, MAXV)


// ============================
//  Mandatory parameters
// ============================

#define MANDATORY_PARAM_LIST(X)                                      \
    PARAM_DOUBLE_ENTRY(X, wheelbase,   WHEELBASE_PARAM_NAME,   2.0, 0.0, 10.0)       \
    PARAM_DOUBLE_ENTRY(X, track_width, TRACK_WIDTH_PARAM_NAME, 1.4, 0.0, 10.0)

#ifndef PARAM_LIST
#define PARAM_LIST(X) /* empty */
#endif

#define FULL_PARAM_LIST(X) \
    MANDATORY_PARAM_LIST(X) \
    PARAM_LIST(X)

// ============================
//  Mandatory states
// ============================

#define MANDATORY_STATE_LIST(X)                                      \
    STATE_DOUBLE_ENTRY(X, x,    X_STATE_NAME,   -10000.0, 10000.0)       \
    STATE_DOUBLE_ENTRY(X, y,    Y_STATE_NAME,   -10000.0, 10000.0)       \
    STATE_DOUBLE_ENTRY(X, yaw,  YAW_STATE_NAME,  -M_PI,  M_PI)       \
    STATE_DOUBLE_ENTRY(X, wheel_fl_angle, WHEEL_FL_ANGLE_STATE_NAME, -M_PI/2, M_PI/2)       \
    STATE_DOUBLE_ENTRY(X, wheel_fr_angle, WHEEL_FR_ANGLE_STATE_NAME, -M_PI/2, M_PI/2)       \

#ifndef STATE_LIST
#define STATE_LIST(X) /* empty */
#endif

#define FULL_STATE_LIST(X) \
    MANDATORY_STATE_LIST(X) \
    STATE_LIST(X)


// ============================
//  Internal Macros for Expansion
// ============================

// Param helpers
#define INTERNAL_PARAM_ENUM(NAME, STR, DEF, MINV, MAXV) P_##NAME,
#define INTERNAL_PARAM_NAME(NAME, STR, DEF, MINV, MAXV) STR,
#define INTERNAL_PARAM_MIN(NAME, STR, DEF, MINV, MAXV) MINV,
#define INTERNAL_PARAM_MAX(NAME, STR, DEF, MINV, MAXV) MAXV,
#define INTERNAL_PARAM_DEF(NAME, STR, DEF, MINV, MAXV) DEF,

// Setting helpers
#define INTERNAL_SETTING_ENUM(NAME, STR, ...) S_##NAME,
#define INTERNAL_SETTING_NAME(NAME, STR, ...) STR,

// Option helpers
// For names array:
#define INTERNAL_OPTION_NAME(OPTSTR) OPTSTR,
#define INTERNAL_SETTING_COLLECT_OPTIONS(NAME, STR, ...) __VA_ARGS__

// For counts array:
#define INTERNAL_OPTION_COUNT(OPTSTR) 1 +
#define INTERNAL_SETTING_COUNT_BLOCK(NAME, STR, ...) __VA_ARGS__ 0,


// Input helpers
#define INTERNAL_INPUT_ENUM(NAME, STR, MINV, MAXV) I_##NAME,
#define INTERNAL_INPUT_NAME(NAME, STR, MINV, MAXV) STR,
#define INTERNAL_INPUT_MIN(NAME, STR, MINV, MAXV) MINV,
#define INTERNAL_INPUT_MAX(NAME, STR, MINV, MAXV) MAXV,

// State helpers
#define INTERNAL_STATE_ENUM(NAME, STR, MINV, MAXV) ST_##NAME,
#define INTERNAL_STATE_NAME(NAME, STR, MINV, MAXV) STR,
#define INTERNAL_STATE_MIN(NAME, STR, MINV, MAXV) MINV,
#define INTERNAL_STATE_MAX(NAME, STR, MINV, MAXV) MAXV,


// ============================
//  Metadata generation macro
// ============================

#define DECLARE_MODEL_METADATA() \
    /* Param Index Enum */ \
    enum ParamIndex { FULL_PARAM_LIST(INTERNAL_PARAM_ENUM) P_COUNT }; \
    \
    /* Param Names */ \
    static const char* const g_param_names[] = { \
        FULL_PARAM_LIST(INTERNAL_PARAM_NAME) \
    }; \
    \
    /* Param Min */ \
    static double g_param_min[] = { \
        FULL_PARAM_LIST(INTERNAL_PARAM_MIN) \
    }; \
    \
    /* Param Max */ \
    static double g_param_max[] = { \
        FULL_PARAM_LIST(INTERNAL_PARAM_MAX) \
    }; \
    \
    /* Param Default */ \
    static double g_param_default[] = { \
        FULL_PARAM_LIST(INTERNAL_PARAM_DEF) \
    }; \
    \
    /* Setting Index Enum */ \
    enum SettingIndex { \
        SETTING_LIST(INTERNAL_SETTING_ENUM, OPTION_ENTRY) \
        S_COUNT \
    }; \
    \
    /* Setting Names */ \
    static const char* const g_setting_names[] = { \
        SETTING_LIST(INTERNAL_SETTING_NAME, OPTION_ENTRY) \
    }; \
    \
    /* Option Names (flattened) */ \
    static const char* const g_setting_option_names[] = { \
        SETTING_LIST(INTERNAL_SETTING_COLLECT_OPTIONS, INTERNAL_OPTION_NAME) \
    }; \
    \
    /* Option Counts per Setting */ \
    static const int32_t g_setting_option_counts[] = { \
        SETTING_LIST(INTERNAL_SETTING_COUNT_BLOCK, INTERNAL_OPTION_COUNT) \
    }; \
    \
    /* Num Options */ \
    static const int32_t g_num_setting_options = \
        static_cast<int32_t>(sizeof(g_setting_option_names) / sizeof(g_setting_option_names[0])); \
    \
    /* Helper to build the setting_index map at runtime */ \
    static const std::vector<int32_t>& get_setting_option_indices() { \
        static const std::vector<int32_t> indices = [](){ \
            std::vector<int32_t> vec; \
            vec.reserve(g_num_setting_options); \
            for (int s = 0; s < S_COUNT; ++s) { \
                int count = g_setting_option_counts[s]; \
                for (int k = 0; k < count; ++k) { \
                    vec.push_back(s); \
                } \
            } \
            return vec; \
        }(); \
        return indices; \
    } \
    \
    /* Input Index Enum */ \
    enum InputIndex { \
        INPUT_LIST(INTERNAL_INPUT_ENUM) \
        I_COUNT \
    }; \
    \
    /* Input Names */ \
    static const char* const g_input_names[] = { \
        INPUT_LIST(INTERNAL_INPUT_NAME) \
    }; \
    \
    /* Input Min */ \
    static double g_input_min[] = { \
        INPUT_LIST(INTERNAL_INPUT_MIN) \
    }; \
    \
    /* Input Max */ \
    static double g_input_max[] = { \
        INPUT_LIST(INTERNAL_INPUT_MAX) \
    }; \
    \
    /* State Index Enum */ \
    enum StateIndex { \
        FULL_STATE_LIST(INTERNAL_STATE_ENUM) \
        ST_COUNT \
    }; \
    \
    /* State Names */ \
    static const char* const g_state_names[] = { \
        FULL_STATE_LIST(INTERNAL_STATE_NAME) \
    }; \
    \
    /* State Min */ \
    static double g_state_min[] = { \
        FULL_STATE_LIST(INTERNAL_STATE_MIN) \
    }; \
    \
    /* State Max */ \
    static double g_state_max[] = { \
        FULL_STATE_LIST(INTERNAL_STATE_MAX) \
    }; 
