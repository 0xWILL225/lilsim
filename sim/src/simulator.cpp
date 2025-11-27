#include <chrono>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <cstring>

#include <spdlog/spdlog.h>

#include "simulator.hpp"
#include "comm/CommServer.hpp"
#include "TrackLoader.hpp"
#include "models/cars/include/base.h"

namespace sim {

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
    if (m_modelLoader && m_carModel) {
        return m_modelLoader->get_descriptor(m_carModel);
    }
    return nullptr;
}

void Simulator::setParam(size_t index, double value) {
    std::lock_guard<std::mutex> lock(m_paramMutex);
    m_pendingParamUpdates.push_back({index, value});
}

void Simulator::setSetting(size_t index, int32_t value) {
    std::lock_guard<std::mutex> lock(m_paramMutex);
    m_pendingSettingUpdates.push_back({index, value});
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

        applyStartPoseToDescriptor(m_startPose);
        m_modelLoader->reset(m_carModel, dt);
        
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
