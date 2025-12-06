#pragma once
#include <atomic>
#include <thread>
#include <optional>
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>
#include <deque>
#include <chrono>

#include "CarDefaults.hpp" 
#include "scene.hpp"
#include "SE2.hpp"
#include "ModelLoader.hpp"
#include "messages.pb.h"

namespace comm {
  class CommServer;
}

namespace zmq {
  class context_t;
}

namespace lilsim {
  class AdminCommand;
  class AdminReply;
  class ControlAsync;
  class ModelMetadata;
}

namespace sim {

/** @brief Optional range overrides parsed from YAML profiles. */
struct RangeOverride {
  std::optional<double> min;
  std::optional<double> max;
};

/** @brief Parameter override that also supports supplying a default value. */
struct ParamOverride : RangeOverride {
  std::optional<double> defaultValue;
};

/** @brief Parsed metadata profile describing per-model overrides. */
struct MetadataProfile {
  std::string path;
  std::optional<std::string> declaredModel;
  std::unordered_map<std::string, ParamOverride> paramOverrides;
  std::unordered_map<std::string, RangeOverride> inputOverrides;
  std::unordered_map<std::string, RangeOverride> stateOverrides;
  std::unordered_map<std::string, std::string> settingDefaults;
};

class Simulator {
public:
  explicit Simulator(scene::SceneDB& db);
  ~Simulator();

  void start();
  void stop();
  
  void setInput(const std::vector<double>& u) {
    m_inputUpdateRequested.store(true, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(m_inputMutex);
    m_newInput = u;
  }
  
  void pause() {
    m_paused.store(true, std::memory_order_relaxed);
  }
  void resume() {
    m_paused.store(false, std::memory_order_relaxed);
  }
  bool isPaused() const {
    return m_paused.load(std::memory_order_relaxed);
  }
  void step(uint64_t numTicks) {
    m_stepTarget.store(numTicks, std::memory_order_relaxed);
    if (numTicks > 0) {
      m_paused.store(false, std::memory_order_relaxed);
    }
  }
  
  double getDt() const;
  double getRequestedDt() const;
  void requestDt(double dtSeconds);
  void setRunSpeed(double multiplier);
  double getRunSpeed() const;
  void requestControlPeriodMs(double periodMs);
  double getControlPeriodMilliseconds() const;
  double getRequestedControlPeriodMilliseconds() const;
  void requestControlDelayMs(double delayMs);
  double getControlDelayMilliseconds() const;
  double getRequestedControlDelayMilliseconds() const;
  
  uint64_t getTicksRemaining() const {
    return m_stepTarget.load(std::memory_order_relaxed);
  }
  
  void reset() {
    m_resetRequested.store(true, std::memory_order_relaxed);
  }

  void setCones(const std::vector<scene::Cone>& cones) {
    m_conesUpdateRequested.store(true, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(m_dataMutex);
    m_newCones = cones;
  }
  void setStartPose(const common::SE2& pose) {
    m_startPoseUpdateRequested.store(true, std::memory_order_relaxed);
    m_newStartPose = pose;
  }

  // Model Management
  struct ModelInfo {
      std::string path;
      std::string name;
  };
  std::vector<ModelInfo> getAvailableModels();
  
  bool loadModel(const std::string& modelPath);
  
  // Gets the currently loaded model name (safe copy)
  std::string getCurrentModelName() const;
  
  const CarModelDescriptor* getCurrentModelDescriptor() const;

  // Parameter updates
  void setParam(size_t index, double value);
  void setSetting(size_t index, int32_t value);

  // Parameter profile (YAML) management
  void setParamProfileFile(const std::string& path);
  void clearParamProfile();
  std::string getActiveParamProfilePath() const;
  std::string getPendingParamProfilePath() const;
  bool consumePendingParamSnapshot(std::vector<double>& out);
  bool consumePendingSettingSnapshot(std::vector<int32_t>& out);
  
  bool checkAndClearModelChanged() {
    return m_modelChanged.exchange(false, std::memory_order_relaxed);
  }

  // Communication
  void setCommEnable(bool enable);
  bool isCommEnabled() const {
    return m_commEnabled.load(std::memory_order_relaxed);
  }
  void setSyncMode(bool sync) {
    m_syncMode.store(sync, std::memory_order_relaxed);
  }
  bool isSyncMode() const {
    return m_syncMode.load(std::memory_order_relaxed);
  }
  bool isExternalControlEnabled() const {
    return m_externalControlEnabled.load(std::memory_order_relaxed);
  }
  bool isSyncClientConnected() const;
  std::shared_ptr<zmq::context_t> getCommContext() const;
  uint64_t getMetadataVersion() const {
    return m_metadataVersion.load(std::memory_order_relaxed);
  }
  
private:
  void loop();
  void probeConnection();
  void ensureCommServer();
  void captureBaseMetadata(const CarModelDescriptor* desc);
  void restoreBaseMetadata(const CarModelDescriptor* desc);
  void updateDescriptorViewLocked(const CarModelDescriptor* source);
  void applyProfileMetadata(const CarModelDescriptor* desc,
                            const MetadataProfile& profile);
  void applyProfileRuntimeEffects(const CarModelDescriptor& desc,
                                  const MetadataProfile& profile,
                                  bool profileJustActivated);
  void refreshDescriptorView();
  void publishStateUpdate(const CarModelDescriptor* desc,
                          const scene::Scene& snapshot,
                          uint64_t tick,
                          double simTime);
  void broadcastMetadata(const CarModelDescriptor* desc, const char* reason);
  lilsim::ModelMetadata buildModelMetadata(const CarModelDescriptor* desc);
  void handleAdminCommands(const CarModelDescriptor* desc, double dt);
  bool handleAdminCommand(const lilsim::AdminCommand& cmd,
                          const CarModelDescriptor* desc,
                          lilsim::AdminReply& reply,
                          double dt);
  void stageParamUpdate(size_t index, double value);
  void stageSettingUpdate(size_t index, int32_t value);
  void applyAsyncControl(const lilsim::ControlAsync& control,
                         const CarModelDescriptor* desc);
  bool requestSyncControl(const CarModelDescriptor* desc,
                          double simTime,
                          uint64_t tick);
  void pollSyncReplies(const CarModelDescriptor* desc);
  bool ensureSyncControlReady(const CarModelDescriptor* desc, uint64_t tick);
  void dispatchSyncControlRequest(const CarModelDescriptor* desc,
                                  double simTime,
                                  uint64_t tick);
  void handleSyncReply(const CarModelDescriptor* desc,
                       const lilsim::ControlReply& reply);
  bool waitForSyncReply(const CarModelDescriptor* desc,
                        uint64_t requestTick);
  void clearSyncControlQueue();
  void applyPendingTimingConfig(double dtSeconds);
  uint32_t millisecondsToTicks(double ms, double dtSeconds) const;
  double ticksToMilliseconds(uint32_t ticks, double dtSeconds) const;

  scene::SceneDB& m_db;
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_paused{false};
  std::atomic<bool> m_resetRequested{false};
  std::atomic<bool> m_conesUpdateRequested{false};
  std::atomic<bool> m_startPoseUpdateRequested{false};
  std::atomic<bool> m_inputUpdateRequested{false};
  
  std::atomic<uint64_t> m_stepTarget{0}; 
  std::thread m_thread;
  scene::Scene m_state;
  
  std::mutex m_dataMutex;
  std::vector<scene::Cone> m_newCones;
  
  common::SE2 m_newStartPose{0.0, 0.0, 0.0};
  common::SE2 m_startPose{0.0, 0.0, 0.0}; 
  
  std::mutex m_inputMutex;
  std::vector<double> m_newInput;

  // Model
  std::unique_ptr<LoadedCarModel> m_modelLoader;
  CarModel* m_carModel{nullptr};
  mutable std::string m_currentModelName;
  std::atomic<bool> m_modelChanged{false};
  CarModelDescriptor m_descriptorView{};
  std::vector<double> m_paramMinBase;
  std::vector<double> m_paramMaxBase;
  std::vector<double> m_inputMinBase;
  std::vector<double> m_inputMaxBase;
  std::vector<double> m_stateMinBase;
  std::vector<double> m_stateMaxBase;
  
  struct PendingParamUpdate { size_t index; double value; };
  struct PendingSettingUpdate { size_t index; int32_t value; };
  
  std::mutex m_paramMutex; 
  std::vector<PendingParamUpdate> m_pendingParamUpdates;
  std::vector<PendingSettingUpdate> m_pendingSettingUpdates;

  mutable std::mutex m_profileMutex;
  std::shared_ptr<MetadataProfile> m_activeProfile;
  std::shared_ptr<MetadataProfile> m_pendingProfile;
  std::string m_activeProfilePath;
  std::string m_pendingProfilePath;
  bool m_profileClearRequested{false};
  std::atomic<bool> m_pendingParamsDirty{true};
  std::atomic<bool> m_pendingSettingsDirty{true};
  
  // Communication
  std::unique_ptr<comm::CommServer> m_commServer;
  std::atomic<bool> m_commEnabled{false};
  std::atomic<bool> m_syncMode{false};
  std::atomic<bool> m_externalControlEnabled{false};
  std::atomic<uint32_t> m_controlPeriodTicks{2}; 
  std::atomic<uint32_t> m_controlDelayTicks{1};
  std::optional<uint32_t> m_pendingControlPeriodTicks;
  std::optional<uint32_t> m_pendingControlDelayTicks;
  mutable std::mutex m_controlTimingMutex;
  uint64_t m_nextControlRequestTick{0};
  std::atomic<double> m_runSpeed{1.0};
  std::atomic<double> m_dt{common::CarDefaults::dt};
  mutable std::mutex m_dtMutex;
  std::optional<double> m_pendingDt;
  std::vector<double> m_lastControlInput;
  /**
   * @brief Metadata for a sync-control request that is waiting to be applied.
   */
  struct PendingSyncRequest {
    uint64_t requestTick{0};
    uint64_t applyTick{0};
    uint64_t metadataVersion{0};
    bool hasReply{false};
    std::vector<double> replyInputs;
    bool timeoutArmed{false};
    std::chrono::steady_clock::time_point timeoutDeadline;
  };
  std::deque<PendingSyncRequest> m_pendingSyncRequests;
  std::atomic<uint64_t> m_metadataVersion{0};
  mutable std::mutex m_metadataMutex;
  lilsim::ModelMetadata m_cachedMetadata;
  bool m_metadataDirty{true};
  std::vector<double> m_asyncInputValues;
  std::atomic<bool> m_asyncInputPending{false};
};

} // namespace sim
