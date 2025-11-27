#pragma once

#include <string>
#include <memory>
#include <vector>
#include <functional>

// Include the C base interface relative to project root
#include "models/cars/include/base.h"

namespace sim {

class LoadedCarModel {
public:
    LoadedCarModel(const std::string& libraryPath);
    ~LoadedCarModel();

    // Disable copy
    LoadedCarModel(const LoadedCarModel&) = delete;
    LoadedCarModel& operator=(const LoadedCarModel&) = delete;

    // Disable move for simplicity
    LoadedCarModel(LoadedCarModel&&) = delete;
    LoadedCarModel& operator=(LoadedCarModel&&) = delete;

    bool isValid() const { return m_valid; }
    const std::string& getError() const { return m_error; }

    // Wrapped C functions
    CarModel* create(double dt);
    void destroy(CarModel* model);
    const CarModelDescriptor* get_descriptor(CarModel* model);
    const char* get_name();
    void reset(CarModel* model, double dt);
    void step(CarModel* model, double dt);

private:
    void* m_handle{nullptr};
    bool m_valid{false};
    std::string m_error;

    // Function pointers
    using FnCreate = CarModel* (*)(double);
    using FnDestroy = void (*)(CarModel*);
    using FnGetDescriptor = const CarModelDescriptor* (*)(CarModel*);
    using FnGetName = const char* (*)(void);
    using FnReset = void (*)(CarModel*, double);
    using FnStep = void (*)(CarModel*, double);

    FnCreate m_fnCreate{nullptr};
    FnDestroy m_fnDestroy{nullptr};
    FnGetDescriptor m_fnGetDescriptor{nullptr};
    FnGetName m_fnGetName{nullptr};
    FnReset m_fnReset{nullptr};
    FnStep m_fnStep{nullptr};
};

} // namespace sim

