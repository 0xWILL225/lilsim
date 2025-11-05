#include "MarkerSystem.hpp"
#include <algorithm>

namespace viz {

void MarkerSystem::addMarker(const std::string& ns, int id,
                              const Marker& marker, double simulation_time) {
  MarkerKey key{ns, id};
  Marker newMarker = marker;
  newMarker.creation_time = simulation_time;
  m_markers[key] = newMarker;

  // Initialize namespace visibility if not set
  if (m_namespaceVisibility.find(ns) == m_namespaceVisibility.end()) {
    m_namespaceVisibility[ns] = true;
  }
}

void MarkerSystem::deleteMarker(const std::string& ns, int id) {
  MarkerKey key{ns, id};
  m_markers.erase(key);
}

void MarkerSystem::deleteNamespace(const std::string& ns) {
  auto it = m_markers.begin();
  while (it != m_markers.end()) {
    if (it->first.ns == ns) {
      it = m_markers.erase(it);
    } else {
      ++it;
    }
  }
  m_namespaceVisibility.erase(ns);
}

void MarkerSystem::clearAll() {
  m_markers.clear();
  m_namespaceVisibility.clear();
}

void MarkerSystem::update(double simulation_time) {
  // Remove expired markers
  auto it = m_markers.begin();
  while (it != m_markers.end()) {
    const Marker& marker = it->second;
    if (marker.ttl_sec.has_value()) {
      double elapsed = simulation_time - marker.creation_time;
      if (elapsed >= marker.ttl_sec.value()) {
        it = m_markers.erase(it);
        continue;
      }
    }
    ++it;
  }
}

std::vector<std::string> MarkerSystem::getNamespaces() const {
  std::vector<std::string> namespaces;
  for (const auto& [key, marker] : m_markers) {
    if (std::find(namespaces.begin(), namespaces.end(), key.ns) ==
        namespaces.end()) {
      namespaces.push_back(key.ns);
    }
  }
  std::sort(namespaces.begin(), namespaces.end());
  return namespaces;
}

std::vector<int> MarkerSystem::getMarkerIds(const std::string& ns) const {
  std::vector<int> ids;
  for (const auto& [key, marker] : m_markers) {
    if (key.ns == ns) {
      ids.push_back(key.id);
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

bool MarkerSystem::isNamespaceVisible(const std::string& ns) const {
  auto it = m_namespaceVisibility.find(ns);
  return (it == m_namespaceVisibility.end()) ? true : it->second;
}

void MarkerSystem::setNamespaceVisible(const std::string& ns, bool visible) {
  m_namespaceVisibility[ns] = visible;
}

bool MarkerSystem::isMarkerVisible(const std::string& ns, int id) const {
  MarkerKey key{ns, id};
  auto it = m_markers.find(key);
  if (it == m_markers.end()) {
    return false;
  }
  return it->second.visible && isNamespaceVisible(ns);
}

void MarkerSystem::setMarkerVisible(const std::string& ns, int id,
                                     bool visible) {
  MarkerKey key{ns, id};
  auto it = m_markers.find(key);
  if (it != m_markers.end()) {
    it->second.visible = visible;
  }
}

} // namespace viz

