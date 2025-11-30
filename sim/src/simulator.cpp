#include <chrono>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <unordered_set>
#include <limits>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "simulator.hpp"
#include "comm/CommServer.hpp"
#include "TrackLoader.hpp"
#include "models/cars/include/base.h"

namespace sim {

namespace {

/**
 * @brief Load and parse a metadata profile from a YAML file.
 * @param path Absolute or relative path to the YAML file.
 * @return Parsed profile structure ready for staging inside the simulator.
 * @throws YAML::Exception if the file cannot be parsed.
 */
std::shared_ptr<MetadataProfile> loadMetadataProfileFromYaml(const std::string& path) {
    auto profile = std::make_shared<MetadataProfile>();
    profile->path = path;

    YAML::Node root = YAML::LoadFile(path);

    if (auto modelNode = root["model"]; modelNode && modelNode.IsScalar()) {
        profile->declaredModel = modelNode.as<std::string>();
    }

    auto parseRange = [](const YAML::Node& node, RangeOverride& range) {
        if (auto minNode = node["min"]; minNode && minNode.IsScalar()) {
            range.min = minNode.as<double>();
        }
        if (auto maxNode = node["max"]; maxNode && maxNode.IsScalar()) {
            range.max = maxNode.as<double>();
        }
    };

    if (auto paramsNode = root["parameters"]; paramsNode && paramsNode.IsMap()) {
        for (const auto& entry : paramsNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            ParamOverride override{};
            parseRange(entry.second, override);
            if (auto defNode = entry.second["default"]; defNode && defNode.IsScalar()) {
                override.defaultValue = defNode.as<double>();
            }
            profile->paramOverrides[name] = override;
        }
    }

    if (auto inputsNode = root["inputs"]; inputsNode && inputsNode.IsMap()) {
        for (const auto& entry : inputsNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            RangeOverride override{};
            parseRange(entry.second, override);
            profile->inputOverrides[name] = override;
        }
    }

    if (auto statesNode = root["states"]; statesNode && statesNode.IsMap()) {
        for (const auto& entry : statesNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            RangeOverride override{};
            parseRange(entry.second, override);
            profile->stateOverrides[name] = override;
        }
    }

    if (auto settingsNode = root["settings"]; settingsNode && settingsNode.IsMap()) {
        for (const auto& entry : settingsNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            if (auto defNode = entry.second["default"]; defNode && defNode.IsScalar()) {
                profile->settingDefaults[name] = defNode.as<std::string>();
            }
        }
    }

    return profile;
}

} // namespace

Simulator::Simulator(scene::SceneDB& db)
  : m_db(db) {
}

Simulator::~Simulator() {
  stop();
}

void Simulator::start() {
  if (m_running.exchange(true))
    return;
  m_thread = std::thread(&Simulator::loop, this);
}

void Simulator::stop() {
  if (!m_running.exchange(false))
    return;
  if (m_thread.joinable())
    m_thread.join();
  
  // Stop communication server if running
  if (m_commServer) {
    m_commServer->stop();
  }
}

bool Simulator::isSyncClientConnected() const {
  return m_commServer && m_commServer->isSyncClientConnected();
}

void Simulator::probeConnection() {
  if (m_commServer && m_commEnabled.load(std::memory_order_relaxed) && 
      m_syncMode.load(std::memory_order_relaxed)) {
    m_commServer->probeConnection(50);  // 50ms timeout for heartbeat
  }
}

void Simulator::setCommEnable(bool enable) {
  if (enable && !m_commEnabled.load(std::memory_order_relaxed)) {
    if (!m_commServer) {
      m_commServer = std::make_unique<comm::CommServer>();
    }
    if (m_commServer->start()) {
      m_commEnabled.store(true, std::memory_order_relaxed);
      m_syncMode.store(false, std::memory_order_relaxed);
      spdlog::info("[sim] Communication enabled");
    } else {
      spdlog::error("[sim] Failed to start communication server");
    }
  } else if (!enable && m_commEnabled.load(std::memory_order_relaxed)) {
    if (m_commServer) {
      m_commServer->stop();
    }
    m_commEnabled.store(false, std::memory_order_relaxed);
    spdlog::info("[sim] Communication disabled");
  }
}

double Simulator::getDt() const {
    return common::CarDefaults::dt;
}

std::vector<Simulator::ModelInfo> Simulator::getAvailableModels() {
    std::vector<ModelInfo> models;
    std::vector<std::filesystem::path> searchPaths = {
        "./models/cars",
        "../models/cars",
        "models/cars",
        "build/debug/models/cars"
    };
    
    for (const auto& p : searchPaths) {
        if (std::filesystem::exists(p) && std::filesystem::is_directory(p)) {
            for (const auto& entry : std::filesystem::directory_iterator(p)) {
                auto ext = entry.path().extension();
                if (ext == ".so" || ext == ".dll" || ext == ".dylib") {
                     std::string path = std::filesystem::absolute(entry.path()).string();
                     std::string name = entry.path().stem().string();
                     
                     // Attempt to load to get real name
                     // This might be slow, but it's correct
                     {
                         LoadedCarModel tempLoader(path);
                         if (tempLoader.isValid()) {
                             name = tempLoader.get_name();
                         }
                     }
                     
                     models.push_back({path, name});
                }
            }
        }
    }
    return models;
}

bool Simulator::loadModel(const std::string& modelPath) {
    auto loader = std::make_unique<LoadedCarModel>(modelPath);
    if (!loader->isValid()) {
        spdlog::error("[sim] Failed to load model from {}: {}", modelPath, loader->getError());
        return false;
    }

    std::lock_guard<std::mutex> lock(m_dataMutex);
    
    double dt = getDt(); 
    CarModel* newModel = loader->create(dt);
    if (!newModel) {
        spdlog::error("[sim] Failed to create model instance");
        return false;
    }

    const auto* desc = loader->get_descriptor(newModel);
    if (desc) {
        std::lock_guard<std::mutex> inputLock(m_inputMutex);
        m_newInput.resize(desc->num_inputs, 0.0);
    }

    if (m_carModel && m_modelLoader) {
        m_modelLoader->destroy(m_carModel);
    }
    
    m_modelLoader = std::move(loader);
    m_carModel = newModel;
    m_currentModelName = m_modelLoader->get_name();
    m_modelChanged.store(true);
    
    if (desc) {
        m_state.car_input_values.assign(desc->input_values, desc->input_values + desc->num_inputs);
        m_state.car_state_values.assign(desc->state_values, desc->state_values + desc->num_states);
    }
    if (desc) {
        captureBaseMetadata(desc);
        restoreBaseMetadata(desc);
    }
    {
        std::lock_guard<std::mutex> profileLock(m_profileMutex);
        m_activeProfile.reset();
        m_pendingProfile.reset();
        m_activeProfilePath.clear();
        m_pendingProfilePath.clear();
        m_profileClearRequested = false;
    }
    updateDescriptorViewLocked(desc);
    m_pendingParamsDirty.store(true, std::memory_order_relaxed);
    m_pendingSettingsDirty.store(true, std::memory_order_relaxed);
    // m_pendingSettingsDirty.store(true, std::memory_order_relaxed);

    spdlog::info("[sim] Loaded model: {}", m_currentModelName);
    return true;
}

std::string Simulator::getCurrentModelName() const {
    auto* self = const_cast<Simulator*>(this);
    std::lock_guard<std::mutex> lock(self->m_dataMutex);
    return m_currentModelName;
}

const CarModelDescriptor* Simulator::getCurrentModelDescriptor() const {
    auto* self = const_cast<Simulator*>(this);
    std::lock_guard<std::mutex> lock(self->m_dataMutex);
    if (!self->m_modelLoader || !self->m_carModel) {
        return nullptr;
    }
    return &self->m_descriptorView;
}

void Simulator::setParam(size_t index, double value) {
    std::lock_guard<std::mutex> lock(m_paramMutex);
    m_pendingParamUpdates.push_back({index, value});
    m_pendingParamsDirty.store(true, std::memory_order_relaxed);
}

void Simulator::setSetting(size_t index, int32_t value) {
    std::lock_guard<std::mutex> lock(m_paramMutex);
    m_pendingSettingUpdates.push_back({index, value});
    m_pendingSettingsDirty.store(true, std::memory_order_relaxed);
}

/**
 * @brief Stage a YAML parameter profile to be applied on the next reset.
 * @param path Filesystem path to the YAML profile. Empty path clears the profile.
 */
void Simulator::setParamProfileFile(const std::string& path) {
    if (path.empty()) {
        clearParamProfile();
        return;
    }

    std::shared_ptr<MetadataProfile> profile;
    try {
        profile = loadMetadataProfileFromYaml(path);
    } catch (const std::exception& ex) {
        spdlog::error("[sim] Failed to load parameter profile '{}': {}", path, ex.what());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_profileMutex);
        m_pendingProfile = profile;
        m_pendingProfilePath = path;
        m_profileClearRequested = false;
    }

    m_pendingParamsDirty.store(true, std::memory_order_relaxed);
    spdlog::info("[sim] Parameter profile '{}' loaded. Press Reset to apply.", path);
}

/**
 * @brief Request that any active or pending profile be cleared on the next reset.
 */
void Simulator::clearParamProfile() {
    bool hadProfile = false;
    {
        std::lock_guard<std::mutex> lock(m_profileMutex);
        hadProfile = static_cast<bool>(m_activeProfile) ||
                     static_cast<bool>(m_pendingProfile) ||
                     m_profileClearRequested;
        m_pendingProfile.reset();
        m_pendingProfilePath.clear();
        m_profileClearRequested = true;
    }

    if (hadProfile) {
        m_pendingParamsDirty.store(true, std::memory_order_relaxed);
        m_pendingSettingsDirty.store(true, std::memory_order_relaxed);
        spdlog::info("[sim] Parameter profile clear requested. Press Reset to apply.");
    }
}

/**
 * @brief Retrieve the path of the currently active profile (if any).
 */
std::string Simulator::getActiveParamProfilePath() const {
    std::lock_guard<std::mutex> lock(m_profileMutex);
    return m_activeProfilePath;
}

/**
 * @brief Retrieve the path of the profile staged for the next reset, if any.
 */
std::string Simulator::getPendingParamProfilePath() const {
    std::lock_guard<std::mutex> lock(m_profileMutex);
    return m_pendingProfilePath;
}

/**
 * @brief If pending parameters changed, provide a snapshot (active + staged values).
 * @param out Destination vector resized to the number of parameters.
 * @return True if the snapshot was refreshed, false if nothing changed.
 */
bool Simulator::consumePendingParamSnapshot(std::vector<double>& out) {
    if (!m_pendingParamsDirty.exchange(false, std::memory_order_relaxed)) {
        return false;
    }

    std::lock_guard<std::mutex> dataLock(m_dataMutex);
    const CarModelDescriptor* desc = (m_modelLoader && m_carModel)
        ? m_modelLoader->get_descriptor(m_carModel)
        : nullptr;
    if (!desc || !desc->param_values) {
        out.clear();
        return true;
    }

    out.assign(desc->param_values, desc->param_values + desc->num_params);

    std::shared_ptr<MetadataProfile> pendingProfileCopy;
    {
        std::lock_guard<std::mutex> profileLock(m_profileMutex);
        pendingProfileCopy = m_pendingProfile;
    }

    if (pendingProfileCopy && desc->param_names) {
        for (size_t i = 0; i < desc->num_params; ++i) {
            const char* name = desc->param_names[i];
            if (!name) continue;
            auto it = pendingProfileCopy->paramOverrides.find(name);
            if (it == pendingProfileCopy->paramOverrides.end()) continue;
            if (!it->second.defaultValue) continue;
            double fallbackMin = desc->param_min ? desc->param_min[i] : *(it->second.defaultValue);
            double fallbackMax = desc->param_max ? desc->param_max[i] : *(it->second.defaultValue);
            out[i] = std::clamp(*(it->second.defaultValue), fallbackMin, fallbackMax);
        }
    }

    {
        std::lock_guard<std::mutex> paramLock(m_paramMutex);
        for (const auto& up : m_pendingParamUpdates) {
            if (up.index < out.size()) {
                out[up.index] = up.value;
            }
        }
    }

    return true;
}

/**
 * @brief If pending settings changed, provide a snapshot (active + staged values).
 * @param out Destination vector resized to the number of settings.
 * @return True if a new snapshot was written, false if nothing changed.
 */
bool Simulator::consumePendingSettingSnapshot(std::vector<int32_t>& out) {
    if (!m_pendingSettingsDirty.exchange(false, std::memory_order_relaxed)) {
        return false;
    }

    std::lock_guard<std::mutex> dataLock(m_dataMutex);
    const CarModelDescriptor* desc = (m_modelLoader && m_carModel)
        ? m_modelLoader->get_descriptor(m_carModel)
        : nullptr;
    if (!desc || !desc->setting_values) {
        out.clear();
        return true;
    }

    out.assign(desc->setting_values, desc->setting_values + desc->num_settings);

    std::shared_ptr<MetadataProfile> pendingProfileCopy;
    {
        std::lock_guard<std::mutex> profileLock(m_profileMutex);
        pendingProfileCopy = m_pendingProfile;
    }

    if (pendingProfileCopy &&
        desc->setting_names &&
        desc->setting_option_names &&
        desc->setting_option_setting_index) {
        for (const auto& [settingName, optionLabel] : pendingProfileCopy->settingDefaults) {
            int settingIndex = -1;
            for (size_t i = 0; i < desc->num_settings; ++i) {
                if (desc->setting_names[i] && settingName == desc->setting_names[i]) {
                    settingIndex = static_cast<int>(i);
                    break;
                }
            }
            if (settingIndex < 0 || settingIndex >= static_cast<int>(out.size())) {
                continue;
            }

            int localOptionIndex = 0;
            int selectedLocal = -1;
            for (size_t opt = 0; opt < desc->num_setting_options; ++opt) {
                if (desc->setting_option_setting_index[opt] != settingIndex) continue;
                const char* optName = desc->setting_option_names[opt];
                if (optName && optionLabel == optName) {
                    selectedLocal = localOptionIndex;
                    break;
                }
                ++localOptionIndex;
            }

            if (selectedLocal >= 0) {
                out[settingIndex] = selectedLocal;
            }
        }
    }

    {
        std::lock_guard<std::mutex> paramLock(m_paramMutex);
        for (const auto& up : m_pendingSettingUpdates) {
            if (up.index < out.size()) {
                out[up.index] = up.value;
            }
        }
    }

    return true;
}

/**
 * @brief Refresh the cached descriptor snapshot exposed to the GUI.
 */
void Simulator::refreshDescriptorView() {
    std::lock_guard<std::mutex> dataLock(m_dataMutex);
    const CarModelDescriptor* desc = (m_modelLoader && m_carModel)
        ? m_modelLoader->get_descriptor(m_carModel)
        : nullptr;
    updateDescriptorViewLocked(desc);
}

/**
 * @brief Capture the model's current metadata arrays to allow restoring defaults later.
 */
void Simulator::captureBaseMetadata(const CarModelDescriptor* desc) {
    auto copyArray = [](std::vector<double>& dst, const double* src, size_t count) {
        if (!src || count == 0) {
            dst.clear();
            return;
        }
        dst.assign(src, src + count);
    };

    if (!desc) {
        m_paramMinBase.clear();
        m_paramMaxBase.clear();
        m_inputMinBase.clear();
        m_inputMaxBase.clear();
        m_stateMinBase.clear();
        m_stateMaxBase.clear();
        return;
    }

    copyArray(m_paramMinBase, desc->param_min, desc->num_params);
    copyArray(m_paramMaxBase, desc->param_max, desc->num_params);
    copyArray(m_inputMinBase, desc->input_min, desc->num_inputs);
    copyArray(m_inputMaxBase, desc->input_max, desc->num_inputs);
    copyArray(m_stateMinBase, desc->state_min, desc->num_states);
    copyArray(m_stateMaxBase, desc->state_max, desc->num_states);
}

/**
 * @brief Restore metadata arrays to their captured defaults.
 */
void Simulator::restoreBaseMetadata(const CarModelDescriptor* desc) {
    auto restoreArray = [](const std::vector<double>& src, double* dst, size_t count) {
        if (!dst || src.size() != count || count == 0) {
            return;
        }
        std::memcpy(dst, src.data(), count * sizeof(double));
    };

    if (!desc) {
        return;
    }

    restoreArray(m_paramMinBase, desc->param_min, desc->num_params);
    restoreArray(m_paramMaxBase, desc->param_max, desc->num_params);
    restoreArray(m_inputMinBase, desc->input_min, desc->num_inputs);
    restoreArray(m_inputMaxBase, desc->input_max, desc->num_inputs);
    restoreArray(m_stateMinBase, desc->state_min, desc->num_states);
    restoreArray(m_stateMaxBase, desc->state_max, desc->num_states);
}

/**
 * @brief Update the cached descriptor snapshot that the GUI can safely read.
 */
void Simulator::updateDescriptorViewLocked(const CarModelDescriptor* source) {
    if (!source) {
        m_descriptorView = {};
        return;
    }

    m_descriptorView = *source;
}

/**
 * @brief Apply range overrides from a profile directly into the model descriptor.
 */
void Simulator::applyProfileMetadata(const CarModelDescriptor* desc,
                                     const MetadataProfile& profile) {
    if (!desc) {
        return;
    }

    auto applyRange = [](const auto& overrides,
                         double* minPtr,
                         double* maxPtr,
                         size_t count,
                         const char* const* names) {
        if (!minPtr || !maxPtr || !names) {
            return;
        }
        for (size_t i = 0; i < count; ++i) {
            const char* raw = names[i];
            if (!raw) continue;
            auto it = overrides.find(raw);
            if (it == overrides.end()) continue;
            if (it->second.min) {
                minPtr[i] = *(it->second.min);
            }
            if (it->second.max) {
                maxPtr[i] = *(it->second.max);
            }
        }
    };

    applyRange(profile.paramOverrides,
               desc->param_min,
               desc->param_max,
               desc->num_params,
               desc->param_names);
    applyRange(profile.inputOverrides,
               desc->input_min,
               desc->input_max,
               desc->num_inputs,
               desc->input_names);
    applyRange(profile.stateOverrides,
               desc->state_min,
               desc->state_max,
               desc->num_states,
               desc->state_names);
}

/**
 * @brief Handle logging and default-value injection when a profile becomes active.
 */
void Simulator::applyProfileRuntimeEffects(const CarModelDescriptor& desc,
                                           const MetadataProfile& profile,
                                           bool profileJustActivated) {
    if (profileJustActivated && profile.declaredModel && !profile.declaredModel->empty() &&
        !m_currentModelName.empty() && *profile.declaredModel != m_currentModelName) {
        spdlog::warn("[sim] Profile '{}' targets model '{}' but current model is '{}'.",
                     profile.path,
                     *profile.declaredModel,
                     m_currentModelName);
    }

    auto logMissing = [&](const char* category,
                          size_t count,
                          const char* const* names,
                          const auto& overrides) {
        if (!profileJustActivated || !names) return;
        std::unordered_set<std::string> present;
        present.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            const char* raw = names[i];
            if (!raw) continue;
            present.insert(raw);
            if (!overrides.contains(raw)) {
                spdlog::info("[sim] Profile '{}' does not override {} '{}'; using model defaults.",
                             profile.path,
                             category,
                             raw);
            }
        }
        for (const auto& [name, _] : overrides) {
            if (!present.contains(name)) {
                spdlog::warn("[sim] Profile '{}' references unknown {} '{}'; ignoring.",
                             profile.path,
                             category,
                             name);
            }
        }
    };

    logMissing("parameter", desc.num_params, desc.param_names, profile.paramOverrides);
    logMissing("input", desc.num_inputs, desc.input_names, profile.inputOverrides);
    logMissing("state", desc.num_states, desc.state_names, profile.stateOverrides);

    if (profileJustActivated && desc.param_values && desc.param_names) {
        for (size_t i = 0; i < desc.num_params; ++i) {
            const char* raw = desc.param_names[i];
            if (!raw) continue;
            auto it = profile.paramOverrides.find(raw);
            if (it == profile.paramOverrides.end()) continue;
            if (!it->second.defaultValue) continue;
            double fallbackMin = desc.param_min ? desc.param_min[i] : *(it->second.defaultValue);
            double fallbackMax = desc.param_max ? desc.param_max[i] : *(it->second.defaultValue);
            double minVal = it->second.min.value_or(fallbackMin);
            double maxVal = it->second.max.value_or(fallbackMax);
            double clamped = std::clamp(*(it->second.defaultValue), minVal, maxVal);
            desc.param_values[i] = clamped;
        }
    }

    if (profileJustActivated && desc.setting_values && desc.setting_names &&
        desc.setting_option_names && desc.setting_option_setting_index) {
        for (const auto& [settingName, optionLabel] : profile.settingDefaults) {
            int settingIndex = -1;
            for (size_t i = 0; i < desc.num_settings; ++i) {
                if (desc.setting_names[i] && settingName == desc.setting_names[i]) {
                    settingIndex = static_cast<int>(i);
                    break;
                }
            }
            if (settingIndex < 0) {
                spdlog::warn("[sim] Profile '{}' references unknown setting '{}'; ignoring.",
                             profile.path,
                             settingName);
                continue;
            }

            int localOptionIndex = 0;
            int selectedLocal = -1;
            for (size_t opt = 0; opt < desc.num_setting_options; ++opt) {
                if (desc.setting_option_setting_index[opt] != settingIndex) continue;
                const char* optName = desc.setting_option_names[opt];
                if (optName && optionLabel == optName) {
                    selectedLocal = localOptionIndex;
                    break;
                }
                ++localOptionIndex;
            }

            if (selectedLocal < 0) {
                spdlog::warn("[sim] Profile '{}' references unknown option '{}' for setting '{}'; ignoring.",
                             profile.path,
                             optionLabel,
                             settingName);
                continue;
            }

            desc.setting_values[settingIndex] = selectedLocal;
        }
    }
}

void Simulator::loop() {
  double dt = getDt(); 
  using clock = std::chrono::steady_clock;
  auto next = clock::now();
  uint64_t tick = 0;

  if (!m_carModel) {
      auto models = getAvailableModels();
      if (!models.empty()) {
          loadModel(models[0].path);
      } else {
          spdlog::warn("[sim] No models found!");
      }
  }

  while (m_running.load(std::memory_order_relaxed)) {
    std::unique_lock<std::mutex> lock(m_dataMutex);
    
    if (!m_carModel || !m_modelLoader) {
         lock.unlock();
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
         continue;
    }
    
    const auto* desc = m_modelLoader->get_descriptor(m_carModel);
    if (!desc) {
         lock.unlock();
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
         continue;
    }

    if (m_startPoseUpdateRequested.exchange(false, std::memory_order_relaxed)) {
      m_startPose = m_newStartPose;
    }
    
    if (m_inputUpdateRequested.exchange(false, std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> inputLock(m_inputMutex);
        if (m_newInput.size() == desc->num_inputs) {
            for(size_t i=0; i<desc->num_inputs; ++i) {
                desc->input_values[i] = m_newInput[i];
            }
        }
    }
    
    auto applyStartPoseToDescriptor = [&](const common::SE2& pose) {
        if (!desc->state_names || !desc->state_values) {
            return;
        }
        for (size_t i = 0; i < desc->num_states; ++i) {
            const char* name = desc->state_names[i];
            if (!name) continue;
            if (std::strcmp(name, X_STATE_NAME) == 0) {
                desc->state_values[i] = pose.x();
            } else if (std::strcmp(name, Y_STATE_NAME) == 0) {
                desc->state_values[i] = pose.y();
            } else if (std::strcmp(name, YAW_STATE_NAME) == 0) {
                desc->state_values[i] = pose.yaw();
            }
        }
    };

    if (m_resetRequested.exchange(false, std::memory_order_relaxed)) {
        {
            std::lock_guard<std::mutex> paramLock(m_paramMutex);
            for (const auto& up : m_pendingParamUpdates) {
                if (up.index < desc->num_params) {
                    desc->param_values[up.index] = up.value;
                }
            }
            for (const auto& up : m_pendingSettingUpdates) {
                if (up.index < desc->num_settings) {
                    desc->setting_values[up.index] = up.value;
                }
            }
            m_pendingParamUpdates.clear();
            m_pendingSettingUpdates.clear();
        }

        std::shared_ptr<MetadataProfile> profileToApply;
        bool profileJustActivated = false;
        bool profileCleared = false;
        {
            std::lock_guard<std::mutex> profileLock(m_profileMutex);
            if (m_pendingProfile) {
                m_activeProfile = m_pendingProfile;
                m_activeProfilePath = m_pendingProfilePath;
                m_pendingProfile.reset();
                m_pendingProfilePath.clear();
                m_profileClearRequested = false;
                profileJustActivated = true;
            } else if (m_profileClearRequested) {
                m_activeProfile.reset();
                m_activeProfilePath.clear();
                m_profileClearRequested = false;
                profileCleared = true;
            }
            profileToApply = m_activeProfile;
        }

        restoreBaseMetadata(desc);
        if (profileToApply) {
            applyProfileMetadata(desc, *profileToApply);
            applyProfileRuntimeEffects(*desc, *profileToApply, profileJustActivated);
            if (profileJustActivated) {
                spdlog::info("[sim] Applied parameter profile '{}' on reset.", m_activeProfilePath);
            }
        } else if (profileCleared) {
            spdlog::info("[sim] Cleared parameter profile; defaults restored on reset.");
        }

        applyStartPoseToDescriptor(m_startPose);
        m_modelLoader->reset(m_carModel, dt);

        updateDescriptorViewLocked(desc);
        m_pendingParamsDirty.store(true, std::memory_order_relaxed);
        m_pendingSettingsDirty.store(true, std::memory_order_relaxed);
        
        m_state.car_state_values.assign(desc->state_values, desc->state_values + desc->num_states);
        m_state.car_input_values.assign(desc->input_values, desc->input_values + desc->num_inputs);
        
        tick = 0;
        m_db.tick.store(tick, std::memory_order_relaxed);
        m_db.publish(m_state);
        m_paused.store(true, std::memory_order_relaxed);
        
        lock.unlock();
        next = clock::now();
        continue;
    }
    
    if (m_conesUpdateRequested.exchange(false, std::memory_order_relaxed)) {
        m_state.cones = m_newCones;
        m_db.publish(m_state);
    }
    
    if (m_paused.load(std::memory_order_relaxed)) {
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      next = clock::now(); 
      continue;
    }

    uint64_t stepsRemaining = m_stepTarget.load(std::memory_order_relaxed);
    if (stepsRemaining > 0) {
      m_stepTarget.fetch_sub(1, std::memory_order_relaxed);
      if (stepsRemaining == 1) {
        m_paused.store(true, std::memory_order_relaxed);
      }
    }

    m_modelLoader->step(m_carModel, dt);

    m_state.car_state_values.assign(desc->state_values, desc->state_values + desc->num_states);
    m_state.car_input_values.assign(desc->input_values, desc->input_values + desc->num_inputs);
    
    tick++;
    m_db.tick.store(tick, std::memory_order_relaxed);

    m_db.publish(m_state);
    
    lock.unlock();

    next += std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(dt));
    std::this_thread::sleep_until(next);
  }
}

} // namespace sim
