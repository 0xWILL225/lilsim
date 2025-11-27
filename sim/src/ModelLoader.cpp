#include "ModelLoader.hpp"
#include <dlfcn.h>
#include <iostream>

namespace sim {

LoadedCarModel::LoadedCarModel(const std::string& libraryPath) {
    // Clear existing error
    dlerror();

    m_handle = dlopen(libraryPath.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!m_handle) {
        const char* err = dlerror();
        m_error = err ? err : "Unknown dlopen error";
        m_valid = false;
        return;
    }

    // Load symbols
    auto loadSym = [&](const char* name) -> void* {
        void* sym = dlsym(m_handle, name);
        const char* err = dlerror();
        if (err) {
            m_error = err;
            return nullptr;
        }
        return sym;
    };

    m_fnCreate = (FnCreate)loadSym("car_model_create");
    m_fnDestroy = (FnDestroy)loadSym("car_model_destroy");
    m_fnGetDescriptor = (FnGetDescriptor)loadSym("car_model_get_descriptor");
    m_fnGetName = (FnGetName)loadSym("car_model_get_name");
    m_fnReset = (FnReset)loadSym("car_model_reset");
    m_fnStep = (FnStep)loadSym("car_model_step");

    if (!m_fnCreate || !m_fnDestroy || !m_fnGetDescriptor || 
        !m_fnGetName || !m_fnReset || !m_fnStep) {
        m_valid = false;
        if (m_error.empty()) m_error = "Missing one or more symbols";
        dlclose(m_handle);
        m_handle = nullptr;
    } else {
        m_valid = true;
    }
}

LoadedCarModel::~LoadedCarModel() {
    if (m_handle) {
        dlclose(m_handle);
    }
}

CarModel* LoadedCarModel::create(double dt) {
    if (m_fnCreate) return m_fnCreate(dt);
    return nullptr;
}

void LoadedCarModel::destroy(CarModel* model) {
    if (m_fnDestroy) m_fnDestroy(model);
}

const CarModelDescriptor* LoadedCarModel::get_descriptor(CarModel* model) {
    if (m_fnGetDescriptor) return m_fnGetDescriptor(model);
    return nullptr;
}

const char* LoadedCarModel::get_name() {
    if (m_fnGetName) return m_fnGetName();
    return "Unknown";
}

void LoadedCarModel::reset(CarModel* model, double dt) {
    if (m_fnReset) m_fnReset(model, dt);
}

void LoadedCarModel::step(CarModel* model, double dt) {
    if (m_fnStep) m_fnStep(model, dt);
}

} // namespace sim

