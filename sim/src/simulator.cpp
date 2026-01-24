#include <chrono>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <unordered_set>
#include <limits>
#include <sstream>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "simulator.hpp"
#include "Constants.hpp"
#include "comm/CommServer.hpp"
#include "TrackLoader.hpp"
#include "models/cars/include/base.h"

namespace sim {

namespace {

constexpr uint32_t kProtocolVersion = 1;
constexpr std::chrono::milliseconds kSyncTimeout{1000};

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
        profile->declared_model = modelNode.as<std::string>();
    }

    auto parseScalarDouble = [](const YAML::Node& value) -> std::optional<double> {
        if (!value || !value.IsScalar()) {
            return std::nullopt;
        }
        std::string raw = value.as<std::string>();
        std::string lower = raw;
        std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });

        auto isPositiveInf = [](const std::string& s) {
            return s == "inf" || s == "+inf" || s == "infinity" || s == "+infinity";
        };
        auto isNegativeInf = [](const std::string& s) {
            return s == "-inf" || s == "-infinity";
        };

        if (isPositiveInf(lower)) {
            return std::numeric_limits<double>::infinity();
        }
        if (isNegativeInf(lower)) {
            return -std::numeric_limits<double>::infinity();
        }

        try {
            return value.as<double>();
        } catch (const YAML::BadConversion& e) {
            spdlog::warn("[sim] Failed to parse scalar '{}' as double: {}", raw, e.what());
            return std::nullopt;
        }
    };

    auto parseRange = [&](const YAML::Node& node, RangeOverride& range) {
        if (auto minNode = node["min"]) {
            if (auto parsed = parseScalarDouble(minNode)) {
                range.min = parsed;
            }
        }
        if (auto maxNode = node["max"]) {
            if (auto parsed = parseScalarDouble(maxNode)) {
                range.max = parsed;
            }
        }
    };

    if (auto paramsNode = root["parameters"]; paramsNode && paramsNode.IsMap()) {
        for (const auto& entry : paramsNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            ParamOverride override{};
            parseRange(entry.second, override);
            if (auto defNode = entry.second["default"]) {
                if (auto parsed = parseScalarDouble(defNode)) {
                    override.defaultValue = parsed;
                }
            }
            profile->param_overrides[name] = override;
        }
    }

    if (auto inputsNode = root["inputs"]; inputsNode && inputsNode.IsMap()) {
        for (const auto& entry : inputsNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            RangeOverride override{};
            parseRange(entry.second, override);
            profile->input_overrides[name] = override;
        }
    }

    if (auto statesNode = root["states"]; statesNode && statesNode.IsMap()) {
        for (const auto& entry : statesNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            RangeOverride override{};
            parseRange(entry.second, override);
            profile->state_overrides[name] = override;
        }
    }

    if (auto settingsNode = root["settings"]; settingsNode && settingsNode.IsMap()) {
        for (const auto& entry : settingsNode) {
            if (!entry.first.IsScalar()) continue;
            std::string name = entry.first.as<std::string>();
            if (auto defNode = entry.second["default"]; defNode && defNode.IsScalar()) {
                profile->setting_defaults[name] = defNode.as<std::string>();
            }
        }
    }

    return profile;
}

} // namespace

Simulator::Simulator(scene::SceneDB& db)
  : m_db(db) {
  ensureCommServer();
  applyPendingTimingConfig(m_dt.load(std::memory_order_relaxed));
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

std::shared_ptr<zmq::context_t> Simulator::getCommContext() const {
  return m_commServer ? m_commServer->getContext() : nullptr;
}

void Simulator::probeConnection() {
  if (m_commServer && m_commEnabled.load(std::memory_order_relaxed) && 
      m_syncMode.load(std::memory_order_relaxed)) {
    m_commServer->probeConnection(50);  // 50ms timeout for heartbeat
  }
}

void Simulator::setCommEnable(bool enable) {
  if (enable) {
    ensureCommServer();
    if (m_commServer && m_commServer->isRunning()) {
      m_commEnabled.store(true, std::memory_order_relaxed);
      spdlog::info("[sim] Communication enabled");
    }
  } else {
    if (m_commServer) {
      m_commServer->stop();
    }
    m_commEnabled.store(false, std::memory_order_relaxed);
    spdlog::info("[sim] Communication disabled");
  }
}

void Simulator::ensureCommServer() {
  if (m_commServer && m_commServer->isRunning()) {
    return;
  }
  if (!m_commServer) {
    m_commServer = std::make_unique<comm::CommServer>();
  }
  if (!m_commServer->start()) {
    spdlog::error("[sim] Failed to start communication server");
    m_commEnabled.store(false, std::memory_order_relaxed);
  } else {
    m_commEnabled.store(true, std::memory_order_relaxed);
  }
}

namespace {
constexpr double kMinDt = 0.001;
constexpr double kMaxDt = 1.0;
constexpr double kMinRunSpeed = 0.1;
constexpr double kMaxRunSpeed = 10.0;
}

double Simulator::getDt() const {
    return m_dt.load(std::memory_order_relaxed);
}

double Simulator::getRequestedDt() const {
    std::lock_guard<std::mutex> lock(m_dtMutex);
    if (m_pendingDt.has_value()) {
        return *m_pendingDt;
    }
    return m_dt.load(std::memory_order_relaxed);
}

void Simulator::requestDt(double dtSeconds) {
    double clamped = std::clamp(dtSeconds, kMinDt, kMaxDt);
    {
        std::lock_guard<std::mutex> lock(m_dtMutex);
        m_pendingDt = clamped;
    }
    spdlog::info("[sim] Requested timestep change to {:.4f}s (applies on reset)", clamped);
}

void Simulator::setRunSpeed(double multiplier) {
    double clamped = std::clamp(multiplier, kMinRunSpeed, kMaxRunSpeed);
    m_runSpeed.store(clamped, std::memory_order_relaxed);
    spdlog::info("[sim] Updated simulation run speed to {:.2f}x", clamped);
}

double Simulator::getRunSpeed() const {
    return m_runSpeed.load(std::memory_order_relaxed);
}

/**
 * @brief Convert a millisecond duration into an integer number of control ticks.
 *
 * Values are clamped to at least one tick so the simulator never blocks forever waiting
 * for a reply.
 *
 * @param ms        Duration expressed in milliseconds from UI/SDK.
 * @param dtSeconds Current (or pending) timestep in seconds.
 * @return Whole number of ticks corresponding to the request.
 */
uint32_t Simulator::millisecondsToTicks(double ms, double dtSeconds) const {
    double dt = std::max(dtSeconds, kMinDt);
    double seconds = std::max(ms, 0.0) / 1000.0;
    double ticks = std::max(1.0, seconds / dt);
    return static_cast<uint32_t>(std::llround(ticks));
}

/**
 * @brief Convert a tick count to milliseconds for display purposes.
 *
 * @param ticks     Number of ticks.
 * @param dtSeconds Timestep in seconds.
 * @return Duration expressed in milliseconds.
 */
double Simulator::ticksToMilliseconds(uint32_t ticks, double dtSeconds) const {
    double dt = std::max(dtSeconds, kMinDt);
    uint32_t clampedTicks = std::max<uint32_t>(1, ticks);
    return dt * static_cast<double>(clampedTicks) * 1000.0;
}

/**
 * @brief Reset any pending sync-control requests and the cached last input.
 */
void Simulator::clearSyncControlQueue() {
    m_pendingSyncRequests.clear();
    m_lastControlInput.clear();
    m_nextControlRequestTick = 0;
}

/**
 * @brief Apply any staged control-period/delay timing once a reset occurs.
 *
 * Incoming UI/SDK values are stored as pending ticks and converted exactly once when a
 * new timestep becomes active.
 *
 * @param dtSeconds Active timestep after reset.
 */
void Simulator::applyPendingTimingConfig(double dtSeconds) {
    uint32_t periodTicks = m_controlPeriodTicks.load(std::memory_order_relaxed);
    uint32_t delayTicks = m_controlDelayTicks.load(std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(m_controlTimingMutex);
      if (m_pendingControlPeriodTicks.has_value()) {
          periodTicks = *m_pendingControlPeriodTicks;
      }
      if (m_pendingControlDelayTicks.has_value()) {
          delayTicks = *m_pendingControlDelayTicks;
      }
      m_pendingControlPeriodTicks.reset();
      m_pendingControlDelayTicks.reset();
    }

    periodTicks = std::max<uint32_t>(1, periodTicks);
    delayTicks = std::max<uint32_t>(1, delayTicks);
    if (delayTicks > periodTicks) {
        spdlog::warn("[sim] Control delay {} ticks exceeded control period {} ticks; clamping delay to period.",
                     delayTicks, periodTicks);
        delayTicks = periodTicks;
    }

    m_controlPeriodTicks.store(periodTicks, std::memory_order_relaxed);
    m_controlDelayTicks.store(delayTicks, std::memory_order_relaxed);

    clearSyncControlQueue();

    double msPeriod = ticksToMilliseconds(periodTicks, dtSeconds);
    double msDelay = ticksToMilliseconds(delayTicks, dtSeconds);
    spdlog::info("[sim] Applied control period {} ticks (~{:.2f} ms) and delay {} ticks (~{:.2f} ms).",
                 periodTicks, msPeriod, delayTicks, msDelay);
}

/**
 * @brief Stage a new control period, specified in milliseconds, to apply on reset.
 *
 * @param periodMs UI-facing duration in milliseconds.
 */
void Simulator::requestControlPeriodMs(double periodMs) {
    double dtSeconds = getRequestedDt();
    uint32_t clampedTicks = millisecondsToTicks(periodMs, dtSeconds);
    {
        std::lock_guard<std::mutex> lock(m_controlTimingMutex);
        uint32_t effectiveDelayTicks = m_pendingControlDelayTicks.has_value()
            ? *m_pendingControlDelayTicks
            : m_controlDelayTicks.load(std::memory_order_relaxed);
        if (effectiveDelayTicks > clampedTicks) {
            spdlog::warn("[sim] Requested control period {} ticks is below pending delay {} ticks; clamping delay to match.",
                         clampedTicks, effectiveDelayTicks);
            m_pendingControlDelayTicks = clampedTicks;
        }
        m_pendingControlPeriodTicks = clampedTicks;
    }
    spdlog::info("[sim] Requested control period change to {} ticks (~{:.2f} ms, applies on reset)",
                 clampedTicks, ticksToMilliseconds(clampedTicks, dtSeconds));
}

/**
 * @brief Current control period converted to milliseconds for UI consumption.
 */
double Simulator::getControlPeriodMilliseconds() const {
    return ticksToMilliseconds(m_controlPeriodTicks.load(std::memory_order_relaxed), getDt());
}

/**
 * @brief Pending control period converted to milliseconds.
 */
double Simulator::getRequestedControlPeriodMilliseconds() const {
    std::lock_guard<std::mutex> lock(m_controlTimingMutex);
    uint32_t ticks = m_pendingControlPeriodTicks.has_value()
        ? *m_pendingControlPeriodTicks
        : m_controlPeriodTicks.load(std::memory_order_relaxed);
    return ticksToMilliseconds(ticks, getRequestedDt());
}

/**
 * @brief Stage a new control delay, specified in milliseconds, to apply on reset.
 *
 * @param delayMs UI-facing duration in milliseconds.
 */
void Simulator::requestControlDelayMs(double delayMs) {
    double dtSeconds = getRequestedDt();
    uint32_t clampedTicks = millisecondsToTicks(delayMs, dtSeconds);
    {
        std::lock_guard<std::mutex> lock(m_controlTimingMutex);
        uint32_t effectivePeriodTicks = m_pendingControlPeriodTicks.has_value()
            ? *m_pendingControlPeriodTicks
            : m_controlPeriodTicks.load(std::memory_order_relaxed);
        if (clampedTicks > effectivePeriodTicks) {
            spdlog::warn("[sim] Requested control delay {} ticks exceeds pending period {} ticks; extending period to match.",
                         clampedTicks, effectivePeriodTicks);
            m_pendingControlPeriodTicks = clampedTicks;
        }
        m_pendingControlDelayTicks = clampedTicks;
    }
    spdlog::info("[sim] Requested control delay change to {} ticks (~{:.2f} ms, applies on reset)",
                 clampedTicks, ticksToMilliseconds(clampedTicks, dtSeconds));
}

/**
 * @brief Current control delay converted to milliseconds for UI consumption.
 */
double Simulator::getControlDelayMilliseconds() const {
    return ticksToMilliseconds(m_controlDelayTicks.load(std::memory_order_relaxed), getDt());
}

/**
 * @brief Pending control delay converted to milliseconds.
 */
double Simulator::getRequestedControlDelayMilliseconds() const {
    std::lock_guard<std::mutex> lock(m_controlTimingMutex);
    uint32_t ticks = m_pendingControlDelayTicks.has_value()
        ? *m_pendingControlDelayTicks
        : m_controlDelayTicks.load(std::memory_order_relaxed);
    return ticksToMilliseconds(ticks, getRequestedDt());
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
        std::lock_guard<std::mutex> profileLock(m_profile_mutex);
        m_activeProfile.reset();
        m_pendingProfile.reset();
        m_activeProfilePath.clear();
        m_pendingProfilePath.clear();
        m_profileClearRequested = false;
    }
    updateDescriptorViewLocked(desc);
    m_pendingParamsDirty.store(true, std::memory_order_relaxed);
    m_pendingSettingsDirty.store(true, std::memory_order_relaxed);
    broadcastMetadata(desc, "model load");

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
    stageParamUpdate(index, value);
}

void Simulator::setSetting(size_t index, uint32_t value) {
    stageSettingUpdate(index, value);
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
        std::lock_guard<std::mutex> lock(m_profile_mutex);
        m_pendingProfile = profile;
        m_pendingProfilePath = path;
        m_profileClearRequested = false;
    }

    m_pendingParamsDirty.store(true, std::memory_order_relaxed);
    m_pendingSettingsDirty.store(true, std::memory_order_relaxed);
    spdlog::info("[sim] Parameter profile '{}' loaded. Press Reset to apply.", path);
}

/**
 * @brief Request that any active or pending profile be cleared on the next reset.
 */
void Simulator::clearParamProfile() {
    bool hadProfile = false;
    {
        std::lock_guard<std::mutex> lock(m_profile_mutex);
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
    std::lock_guard<std::mutex> lock(m_profile_mutex);
    return m_activeProfilePath;
}

/**
 * @brief Retrieve the path of the profile staged for the next reset, if any.
 */
std::string Simulator::getPendingParamProfilePath() const {
    std::lock_guard<std::mutex> lock(m_profile_mutex);
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

    std::shared_ptr<MetadataProfile> pending_profile_copy;
    {
        std::lock_guard<std::mutex> profileLock(m_profile_mutex);
        pending_profile_copy = m_pendingProfile;
    }

    if (pending_profile_copy && desc->param_names) {
        for (size_t i = 0; i < desc->num_params; ++i) {
            const char* name = desc->param_names[i];
            if (!name) continue;
            auto it = pending_profile_copy->param_overrides.find(name);
            if (it == pending_profile_copy->param_overrides.end()) continue;
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
bool Simulator::consumePendingSettingSnapshot(std::vector<uint32_t>& out) {
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

    std::shared_ptr<MetadataProfile> pending_profile_copy;
    {
        std::lock_guard<std::mutex> profileLock(m_profile_mutex);
        pending_profile_copy = m_pendingProfile;
    }

    if (pending_profile_copy &&
        desc->setting_names &&
        desc->setting_option_names &&
        desc->setting_option_setting_index) {
        for (const auto& [setting_name, option_label] : pending_profile_copy->setting_defaults) {
            size_t setting_index = common::kNullIndex;
            for (size_t i = 0; i < desc->num_settings; ++i) {
                if (desc->setting_names[i] && setting_name == desc->setting_names[i]) {
                    setting_index = i;
                    break;
                }
            }
            if (setting_index == common::kNullIndex || setting_index >= out.size()) {
                continue;
            }

            int localOptionIndex = 0;
            int selectedLocal = -1;
            for (size_t opt = 0; opt < desc->num_setting_options; ++opt) {
                if (desc->setting_option_setting_index[opt] != setting_index) continue;
                const char* optName = desc->setting_option_names[opt];
                if (optName && option_label == optName) {
                    selectedLocal = localOptionIndex;
                    break;
                }
                ++localOptionIndex;
            }

            if (selectedLocal >= 0) {
                out[setting_index] = selectedLocal;
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

void Simulator::stageParamUpdate(size_t index, double value) {
    std::lock_guard<std::mutex> lock(m_paramMutex);
    m_pendingParamUpdates.push_back({index, value});
    m_pendingParamsDirty.store(true, std::memory_order_relaxed);
}

void Simulator::stageSettingUpdate(size_t index, uint32_t value) {
    std::lock_guard<std::mutex> lock(m_paramMutex);
    m_pendingSettingUpdates.push_back({index, value});
    m_pendingSettingsDirty.store(true, std::memory_order_relaxed);
}

void Simulator::publishStateUpdate(const CarModelDescriptor* desc,
                                   const scene::Scene& snapshot,
                                   uint64_t tick,
                                   double simTime) {
    if (!desc || !m_commServer || !m_commServer->isRunning()) {
        return;
    }

    lilsim::StateUpdate update;
    auto* sceneMsg = update.mutable_scene();
    auto* header = sceneMsg->mutable_header();
    header->set_tick(tick);
    header->set_sim_time(simTime);
    header->set_version(kProtocolVersion);
    sceneMsg->set_metadata_version(m_metadata_version.load(std::memory_order_relaxed));

    auto assignValues = [](const std::vector<double>& source,
                           const double* fallback,
                           size_t count,
                           google::protobuf::RepeatedField<double>* dest) {
        if (source.size() == count) {
            dest->Assign(source.begin(), source.end());
        } else if (fallback && count > 0) {
            dest->Resize(static_cast<int>(count), 0.0);
            for (size_t i = 0; i < count; ++i) {
                (*dest)[static_cast<int>(i)] = fallback[i];
            }
        }
    };

    assignValues(snapshot.car_state_values, desc->state_values, desc->num_states, sceneMsg->mutable_state_values());
    assignValues(snapshot.car_input_values, desc->input_values, desc->num_inputs, sceneMsg->mutable_input_values());

    auto* params = sceneMsg->mutable_param_values();
    if (desc->param_values && desc->num_params > 0) {
        params->Resize(static_cast<int>(desc->num_params), 0.0);
        for (size_t i = 0; i < desc->num_params; ++i) {
            (*params)[static_cast<int>(i)] = desc->param_values[i];
        }
    }

    auto* settings = sceneMsg->mutable_setting_values();
    if (desc->setting_values && desc->num_settings > 0) {
        settings->Resize(static_cast<int>(desc->num_settings), 0);
        for (size_t i = 0; i < desc->num_settings; ++i) {
            (*settings)[static_cast<int>(i)] = desc->setting_values[i];
        }
    }

    m_commServer->publishState(update);
}

void Simulator::broadcastMetadata(const CarModelDescriptor* desc, const char* reason) {
    if (!desc) {
        return;
    }
    lilsim::ModelMetadata metadata = buildModelMetadata(desc);
    {
        std::lock_guard<std::mutex> lock(m_metadataMutex);
        m_cachedMetadata = metadata;
        m_metadataDirty = false;
    }
    if (m_commServer && m_commServer->isRunning()) {
        m_commServer->publishMetadata(metadata);
    }
    if (reason) {
        spdlog::info("[sim] Broadcast metadata v{} after {}", metadata.schema_version(), reason);
    }
}

lilsim::ModelMetadata Simulator::buildModelMetadata(const CarModelDescriptor* desc) {
    lilsim::ModelMetadata metadata;
    if (!desc) {
        return metadata;
    }

    double simTime = m_db.tick.load(std::memory_order_relaxed) * getDt();
    auto* header = metadata.mutable_header();
    header->set_version(kProtocolVersion);
    header->set_tick(m_db.tick.load(std::memory_order_relaxed));
    header->set_sim_time(simTime);

    metadata.set_model_name(m_currentModelName);
    uint64_t version = m_metadata_version.fetch_add(1, std::memory_order_relaxed) + 1;
    metadata.set_schema_version(version);

    auto* params = metadata.mutable_params();
    for (size_t i = 0; i < desc->num_params; ++i) {
        auto* meta = params->Add();
        meta->set_name(desc->param_names ? desc->param_names[i] : "");
        meta->set_default_value(desc->param_values ? desc->param_values[i] : 0.0);
        if (desc->param_min && desc->param_max) {
            meta->mutable_limits()->set_min(desc->param_min[i]);
            meta->mutable_limits()->set_max(desc->param_max[i]);
        }
    }

    auto* settings = metadata.mutable_settings();
    for (size_t i = 0; i < desc->num_settings; ++i) {
        auto* meta = settings->Add();
        meta->set_name(desc->setting_names ? desc->setting_names[i] : "");
        meta->set_default_index(desc->setting_values ? desc->setting_values[i] : 0);
        if (desc->setting_option_names && desc->setting_option_setting_index) {
            for (size_t opt = 0; opt < desc->num_setting_options; ++opt) {
                if (desc->setting_option_setting_index[opt] == i) {
                    meta->add_options(desc->setting_option_names[opt]
                                          ? desc->setting_option_names[opt]
                                          : "");
                }
            }
        }
    }

    auto* inputs = metadata.mutable_inputs();
    for (size_t i = 0; i < desc->num_inputs; ++i) {
        auto* meta = inputs->Add();
        meta->set_name(desc->input_names ? desc->input_names[i] : "");
        if (desc->input_min && desc->input_max) {
            meta->mutable_limits()->set_min(desc->input_min[i]);
            meta->mutable_limits()->set_max(desc->input_max[i]);
        }
    }

    auto* states = metadata.mutable_states();
    for (size_t i = 0; i < desc->num_states; ++i) {
        auto* meta = states->Add();
        meta->set_name(desc->state_names ? desc->state_names[i] : "");
        if (desc->state_min && desc->state_max) {
            meta->mutable_limits()->set_min(desc->state_min[i]);
            meta->mutable_limits()->set_max(desc->state_max[i]);
        }
    }

    return metadata;
}

void Simulator::handleAdminCommands(const CarModelDescriptor* descHint, double dt) {
    if (!m_commServer || !m_commServer->isRunning()) {
        return;
    }
    while (auto cmd = m_commServer->pollAdminCommand()) {
        lilsim::AdminReply reply;
        auto* header = reply.mutable_header();
        header->set_version(kProtocolVersion);
        uint64_t tick = m_db.tick.load(std::memory_order_relaxed);
        header->set_tick(tick);
        header->set_sim_time(tick * dt);

        bool success = handleAdminCommand(*cmd, descHint, reply, dt);
        reply.set_success(success);
        if (!reply.success() && reply.message().empty()) {
            reply.set_message("Command failed");
        }

        m_commServer->replyAdmin(reply);
    }
}

bool Simulator::handleAdminCommand(const lilsim::AdminCommand& cmd,
                                   const CarModelDescriptor* descHint,
                                   lilsim::AdminReply& reply,
                                   double dt) {
    (void)dt;
    switch (cmd.type()) {
        case lilsim::AdminCommandType::INIT:
        case lilsim::AdminCommandType::RESET:
            reset();
            reply.set_message("Reset requested");
            return true;
        case lilsim::AdminCommandType::PAUSE:
            pause();
            reply.set_message("Paused");
            return true;
        case lilsim::AdminCommandType::RUN:
            resume();
            reply.set_message("Running");
            return true;
        case lilsim::AdminCommandType::STEP: {
            uint64_t steps = cmd.step_count() > 0 ? cmd.step_count() : 1;
            step(steps);
            reply.set_message("Step queued");
            return true;
        }
        case lilsim::AdminCommandType::SET_PARAMS: {
            for (const auto& update : cmd.param_updates()) {
                stageParamUpdate(update.index(), update.value());
            }
            reply.set_message("Parameter overrides staged");
            return true;
        }
        case lilsim::AdminCommandType::SET_SETTINGS: {
            for (const auto& update : cmd.setting_updates()) {
                stageSettingUpdate(update.index(), update.value());
            }
            reply.set_message("Setting overrides staged");
            return true;
        }
        case lilsim::AdminCommandType::SET_CONTROL_MODE: {
            m_syncMode.store(cmd.sync_mode(), std::memory_order_relaxed);
            if (cmd.control_period_ms() > 0) {
                spdlog::warn("[sim] control_period_ms is deprecated in SET_CONTROL_MODE; use SET_SIM_CONFIG instead.");
            }
            if (cmd.has_use_external_control()) {
                m_externalControlEnabled.store(cmd.use_external_control(), std::memory_order_relaxed);
            }
            std::ostringstream oss;
            oss << (m_syncMode.load(std::memory_order_relaxed) ? "Sync" : "Async")
                << " mode, source="
                << (m_externalControlEnabled.load(std::memory_order_relaxed) ? "ZeroMQ client" : "GUI");
            reply.set_message(oss.str());
            return true;
        }
        case lilsim::AdminCommandType::SET_TRACK: {
            if (cmd.track_path().empty()) {
                reply.set_message("Track path missing");
                return false;
            }
            scene::TrackData trackData;
            if (!scene::TrackLoader::loadFromCSV(cmd.track_path(), trackData)) {
                reply.set_message("Failed to load track");
                return false;
            }
            setCones(trackData.cones);
            if (trackData.startPose.has_value()) {
                setStartPose(trackData.startPose.value());
            }
            reply.set_message("Track loaded");
            return true;
        }
        case lilsim::AdminCommandType::LOAD_PARAM_PROFILE: {
            if (cmd.param_profile_path().empty()) {
                reply.set_message("Profile path missing");
                return false;
            }
            setParamProfileFile(cmd.param_profile_path());
            reply.set_message("Profile staged");
            return true;
        }
        case lilsim::AdminCommandType::CLEAR_PARAM_PROFILE:
            clearParamProfile();
            reply.set_message("Profile cleared");
            return true;
        case lilsim::AdminCommandType::GET_METADATA: {
            const CarModelDescriptor* desc = descHint;
            if (!desc) {
                std::lock_guard<std::mutex> lock(m_dataMutex);
                desc = (m_modelLoader && m_carModel)
                         ? m_modelLoader->get_descriptor(m_carModel)
                         : nullptr;
            }
            if (!desc) {
                reply.set_message("No model loaded");
                return false;
            }
            {
                std::lock_guard<std::mutex> lock(m_metadataMutex);
                if (m_metadataDirty) {
                    auto metadata = buildModelMetadata(desc);
                    m_cachedMetadata = metadata;
                    m_metadataDirty = false;
                }
                reply.mutable_metadata()->CopyFrom(m_cachedMetadata);
            }
            reply.set_message("Metadata attached");
            return true;
        }
        case lilsim::AdminCommandType::SET_SIM_CONFIG: {
            bool applied = false;
            if (cmd.has_timestep()) {
                requestDt(cmd.timestep() / 1000.0);
                applied = true;
            }
            if (cmd.has_run_speed()) {
                setRunSpeed(cmd.run_speed());
                applied = true;
            }
            if (cmd.has_control_period_ms_staged()) {
                requestControlPeriodMs(static_cast<double>(cmd.control_period_ms_staged()));
                applied = true;
            }
            if (cmd.has_control_delay_ms_staged()) {
                requestControlDelayMs(static_cast<double>(cmd.control_delay_ms_staged()));
                applied = true;
            }
            if (!applied) {
                reply.set_message("No simulation config fields provided");
                return false;
            }
            reply.set_timestep(getRequestedDt() * 1000.0);
            reply.set_run_speed(getRunSpeed());
            reply.set_control_period_ms(getRequestedControlPeriodMilliseconds());
            reply.set_control_delay_ms(getRequestedControlDelayMilliseconds());
            reply.set_message("Simulation config updated");
            return true;
        }
        case lilsim::AdminCommandType::GET_SIM_CONFIG: {
            reply.set_timestep(getRequestedDt() * 1000.0);
            reply.set_run_speed(getRunSpeed());
            reply.set_control_period_ms(getRequestedControlPeriodMilliseconds());
            reply.set_control_delay_ms(getRequestedControlDelayMilliseconds());
            reply.set_message("Simulation config attached");
            return true;
        }
        default:
            reply.set_message("Unsupported admin command");
            return false;
    }
}

void Simulator::applyAsyncControl(const lilsim::ControlAsync& control,
                                  const CarModelDescriptor* desc) {
    if (!desc) {
        return;
    }
    if (control.metadata_version() != m_metadata_version.load(std::memory_order_relaxed)) {
        spdlog::warn("[sim] Dropping async control (metadata version mismatch)");
        return;
    }
    if (static_cast<size_t>(control.input_values_size()) != desc->num_inputs) {
        spdlog::warn("[sim] Dropping async control (input size mismatch)");
        return;
    }
    std::lock_guard<std::mutex> inputLock(m_inputMutex);
    m_newInput.resize(desc->num_inputs);
    for (size_t i = 0; i < desc->num_inputs; ++i) {
        m_newInput[i] = control.input_values(static_cast<int>(i));
    }
    m_input_update_requested.store(true, std::memory_order_relaxed);
}

bool Simulator::requestSyncControl(const CarModelDescriptor* desc,
                                   double simTime,
                                   uint64_t tick) {
    if (!desc || !m_commServer || !m_commServer->isRunning()) {
        return false;
    }
    lilsim::ControlRequest request;
    auto* header = request.mutable_header();
    header->set_tick(tick);
    header->set_sim_time(simTime);
    header->set_version(kProtocolVersion);

    auto* sceneMsg = request.mutable_scene();
    const uint64_t requestMetadataVersion = m_metadata_version.load(std::memory_order_relaxed);
    auto* sceneHeader = sceneMsg->mutable_header();
    sceneHeader->set_tick(tick);
    sceneHeader->set_sim_time(simTime);
    sceneHeader->set_version(kProtocolVersion);
    sceneMsg->set_metadata_version(requestMetadataVersion);

    if (desc->state_values && desc->num_states > 0) {
        sceneMsg->mutable_state_values()->Resize(static_cast<int>(desc->num_states), 0.0);
        for (size_t i = 0; i < desc->num_states; ++i) {
            (*sceneMsg->mutable_state_values())[static_cast<int>(i)] = desc->state_values[i];
        }
    }
    if (desc->input_values && desc->num_inputs > 0) {
        sceneMsg->mutable_input_values()->Resize(static_cast<int>(desc->num_inputs), 0.0);
        for (size_t i = 0; i < desc->num_inputs; ++i) {
            (*sceneMsg->mutable_input_values())[static_cast<int>(i)] = desc->input_values[i];
        }
    }
    if (desc->param_values && desc->num_params > 0) {
        sceneMsg->mutable_param_values()->Resize(static_cast<int>(desc->num_params), 0.0);
        for (size_t i = 0; i < desc->num_params; ++i) {
            (*sceneMsg->mutable_param_values())[static_cast<int>(i)] = desc->param_values[i];
        }
    }
    if (desc->setting_values && desc->num_settings > 0) {
        sceneMsg->mutable_setting_values()->Resize(static_cast<int>(desc->num_settings), 0);
        for (size_t i = 0; i < desc->num_settings; ++i) {
            (*sceneMsg->mutable_setting_values())[static_cast<int>(i)] = desc->setting_values[i];
        }
    }

    if (!m_commServer->sendControlRequest(request)) {
        spdlog::warn("[sim] Failed to send sync control request");
        return false;
    }

    PendingSyncRequest pending;
    pending.requestTick = tick;
    uint64_t delayTicks = std::max<uint64_t>(1, m_controlDelayTicks.load(std::memory_order_relaxed));
    pending.applyTick = tick + delayTicks;
    pending.metadataVersion = requestMetadataVersion;
    m_pendingSyncRequests.push_back(std::move(pending));
    return true;
}

void Simulator::handleSyncReply(const CarModelDescriptor* desc,
                                const lilsim::ControlReply& reply) {
    if (!desc) {
        return;
    }
    auto it = std::find_if(m_pendingSyncRequests.begin(), m_pendingSyncRequests.end(),
                           [&](const PendingSyncRequest& pending) {
                               return pending.requestTick == reply.header().tick();
                           });
    if (it == m_pendingSyncRequests.end()) {
        spdlog::warn("[sim] Received sync control reply for unknown tick {}", reply.header().tick());
        return;
    }
    if (reply.metadata_version() != it->metadataVersion) {
        spdlog::warn("[sim] Sync control reply metadata mismatch (tick {})", reply.header().tick());
        return;
    }
    if (static_cast<size_t>(reply.input_values_size()) != desc->num_inputs) {
        spdlog::warn("[sim] Sync control reply input size mismatch (tick {})", reply.header().tick());
        return;
    }
    it->replyInputs.resize(desc->num_inputs);
    for (size_t i = 0; i < desc->num_inputs; ++i) {
        it->replyInputs[i] = reply.input_values(static_cast<int>(i));
    }
    it->hasReply = true;
    it->timeoutArmed = false;
}

void Simulator::pollSyncReplies(const CarModelDescriptor* desc) {
    if (!m_commServer || !m_commServer->isRunning()) {
        return;
    }
    while (auto reply = m_commServer->pollControlReply()) {
        handleSyncReply(desc, *reply);
    }
}

void Simulator::dispatchSyncControlRequest(const CarModelDescriptor* desc,
                                           double simTime,
                                           uint64_t tick) {
    if (!desc) {
        return;
    }
    if (tick < m_nextControlRequestTick) {
        return;
    }
    if (requestSyncControl(desc, simTime, tick)) {
        uint64_t step = std::max<uint32_t>(1, m_controlPeriodTicks.load(std::memory_order_relaxed));
        m_nextControlRequestTick = tick + step;
    }
}

bool Simulator::waitForSyncReply(const CarModelDescriptor* desc,
                                 uint64_t requestTick) {
    if (!desc || !m_commServer || !m_commServer->isRunning()) {
        return false;
    }
    auto it = std::find_if(m_pendingSyncRequests.begin(), m_pendingSyncRequests.end(),
                           [&](const PendingSyncRequest& pending) {
                               return pending.requestTick == requestTick;
                           });
    if (it == m_pendingSyncRequests.end()) {
        return false;
    }
    if (!it->timeoutArmed) {
        it->timeoutArmed = true;
        it->timeoutDeadline = std::chrono::steady_clock::now() + kSyncTimeout;
    }
    while (!it->hasReply) {
        auto now = std::chrono::steady_clock::now();
        if (now >= it->timeoutDeadline) {
            spdlog::warn("[sim] Sync control request at tick {} timed out; pausing simulation.", requestTick);
            pause();
            clearSyncControlQueue();
            return false;
        }
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(it->timeoutDeadline - now);
        int waitMs = static_cast<int>(std::clamp<int64_t>(remaining.count(), 1, 25));
        lilsim::ControlReply reply;
        if (m_commServer->waitControlReply(reply, waitMs)) {
            handleSyncReply(desc, reply);
        }
    }
    return true;
}

bool Simulator::ensureSyncControlReady(const CarModelDescriptor* desc,
                                       uint64_t tick) {
    if (!desc) {
        return true;
    }
    auto applyLastInputs = [&]() {
        if (m_lastControlInput.empty() || !desc->input_values) {
            return;
        }
        size_t count = std::min(m_lastControlInput.size(), static_cast<size_t>(desc->num_inputs));
        for (size_t i = 0; i < count; ++i) {
            desc->input_values[i] = m_lastControlInput[i];
        }
    };

    if (m_pendingSyncRequests.empty()) {
        applyLastInputs();
        return true;
    }

    auto& pending = m_pendingSyncRequests.front();
    if (pending.applyTick > tick) {
        applyLastInputs();
        return true;
    }

    if (!pending.hasReply) {
        if (!waitForSyncReply(desc, pending.requestTick)) {
            return false;
        }
    }

    if (!pending.replyInputs.empty()) {
        m_lastControlInput = pending.replyInputs;
        applyLastInputs();
    }

    m_pendingSyncRequests.pop_front();
    return true;
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

    applyRange(profile.param_overrides,
               desc->param_min,
               desc->param_max,
               desc->num_params,
               desc->param_names);
    applyRange(profile.input_overrides,
               desc->input_min,
               desc->input_max,
               desc->num_inputs,
               desc->input_names);
    applyRange(profile.state_overrides,
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
    if (profileJustActivated && profile.declared_model && !profile.declared_model->empty() &&
        !m_currentModelName.empty() && *profile.declared_model != m_currentModelName) {
        spdlog::warn("[sim] Profile '{}' targets model '{}' but current model is '{}'.",
                     profile.path,
                     *profile.declared_model,
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

    logMissing("parameter", desc.num_params, desc.param_names, profile.param_overrides);
    logMissing("input", desc.num_inputs, desc.input_names, profile.input_overrides);
    logMissing("state", desc.num_states, desc.state_names, profile.state_overrides);

    if (profileJustActivated && desc.param_values && desc.param_names) {
        for (size_t i = 0; i < desc.num_params; ++i) {
            const char* raw = desc.param_names[i];
            if (!raw) continue;
            auto it = profile.param_overrides.find(raw);
            if (it == profile.param_overrides.end()) continue;
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
        for (const auto& [setting_name, option_label] : profile.setting_defaults) {
            int setting_index = -1;
            for (size_t i = 0; i < desc.num_settings; ++i) {
                if (desc.setting_names[i] && setting_name == desc.setting_names[i]) {
                    setting_index = static_cast<int>(i);
                    break;
                }
            }
            if (setting_index < 0) {
                spdlog::warn("[sim] Profile '{}' references unknown setting '{}'; ignoring.",
                             profile.path,
                             setting_name);
                continue;
            }

            int localOptionIndex = 0;
            int selectedLocal = -1;
            for (size_t opt = 0; opt < desc.num_setting_options; ++opt) {
                if (desc.setting_option_setting_index[opt] != setting_index) continue;
                const char* optName = desc.setting_option_names[opt];
                if (optName && option_label == optName) {
                    selectedLocal = localOptionIndex;
                    break;
                }
                ++localOptionIndex;
            }

            if (selectedLocal < 0) {
                spdlog::warn("[sim] Profile '{}' references unknown option '{}' for setting '{}'; ignoring.",
                             profile.path,
                             option_label,
                             setting_name);
                continue;
            }

            desc.setting_values[setting_index] = selectedLocal;
        }
    }
}

void Simulator::loop() {
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
    if (m_commServer && m_commServer->isRunning()) {
      handleAdminCommands(nullptr, getDt());
    }

    double dt = getDt();
    double runSpeed = std::max(kMinRunSpeed, getRunSpeed());

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

    if (m_start_pose_update_requested.exchange(false, std::memory_order_relaxed)) {
      m_startPose = m_newStartPose;
    }

    bool externalControlActive = m_externalControlEnabled.load(std::memory_order_relaxed);
    bool syncMode = m_syncMode.load(std::memory_order_relaxed);

    if (externalControlActive &&
        m_commServer && m_commEnabled.load(std::memory_order_relaxed) &&
        !syncMode) {
      if (auto asyncControl = m_commServer->pollAsyncControl()) {
        applyAsyncControl(*asyncControl, desc);
      }
    }
    
    if (m_input_update_requested.exchange(false, std::memory_order_relaxed)) {
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
            std::lock_guard<std::mutex> profileLock(m_profile_mutex);
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

        {
            std::lock_guard<std::mutex> dtLock(m_dtMutex);
            if (m_pendingDt.has_value()) {
                double clamped = std::clamp(*m_pendingDt, kMinDt, kMaxDt);
                m_dt.store(clamped, std::memory_order_relaxed);
                spdlog::info("[sim] Applied timestep {:.4f}s on reset.", clamped);
                m_pendingDt.reset();
            }
        }
        double dt = getDt();
        applyPendingTimingConfig(dt);

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
        broadcastMetadata(desc, "reset");
        
        m_state.car_state_values.assign(desc->state_values, desc->state_values + desc->num_states);
        m_state.car_input_values.assign(desc->input_values, desc->input_values + desc->num_inputs);
        if (desc->input_values && desc->num_inputs > 0) {
            m_lastControlInput.assign(desc->input_values, desc->input_values + desc->num_inputs);
        } else {
            m_lastControlInput.clear();
        }
        
        tick = 0;
        m_db.tick.store(tick, std::memory_order_relaxed);
        m_db.publish(m_state);
        if (m_commServer && m_commServer->isRunning()) {
            publishStateUpdate(desc, m_state, tick, 0.0);
        }
        m_paused.store(true, std::memory_order_relaxed);
        
        lock.unlock();
        next = clock::now();
        continue;
    }
    
    if (m_cones_update_requested.exchange(false, std::memory_order_relaxed)) {
        m_state.cones = m_newCones;
        m_db.publish(m_state);
    }
    
    if (m_paused.load(std::memory_order_relaxed)) {
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      next = clock::now(); 
      continue;
    }

    uint64_t stepsRemaining = m_step_target.load(std::memory_order_relaxed);
    if (stepsRemaining > 0) {
      m_step_target.fetch_sub(1, std::memory_order_relaxed);
      if (stepsRemaining == 1) {
        m_paused.store(true, std::memory_order_relaxed);
      }
    }

    bool syncReady = true;
    if (externalControlActive &&
        m_commServer && m_commEnabled.load(std::memory_order_relaxed) &&
        syncMode) {
      pollSyncReplies(desc);
      dispatchSyncControlRequest(desc, tick * dt, tick);
      syncReady = ensureSyncControlReady(desc, tick);
    } else {
      clearSyncControlQueue();
    }
    if (!syncReady) {
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      next = clock::now();
      continue;
    }

    m_modelLoader->step(m_carModel, dt);

    m_state.car_state_values.assign(desc->state_values, desc->state_values + desc->num_states);
    m_state.car_input_values.assign(desc->input_values, desc->input_values + desc->num_inputs);
    
    tick++;
    m_db.tick.store(tick, std::memory_order_relaxed);

    m_db.publish(m_state);
    if (m_commServer && m_commServer->isRunning()) {
      publishStateUpdate(desc, m_state, tick, tick * dt);
    }
    
    lock.unlock();

    next += std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(dt / runSpeed));
    std::this_thread::sleep_until(next);
  }
}

} // namespace sim
