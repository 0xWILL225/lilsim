#pragma once

#include <webgpu/webgpu.h>
#include <imgui.h>
#include <string>
#include <unordered_map>
#include <filesystem>
#include <optional>
#include <array>

namespace viz {

/**
 * @brief Manages texture loading and WebGPU texture creation for sprites.
 * 
 * This is a singleton that:
 * - Loads PNG images using stb_image
 * - Creates WebGPU textures with nearest-neighbor filtering (for pixel art)
 * - Registers textures with ImGui for rendering
 * - Provides asset paths relative to the executable
 */
class TextureManager {
public:
  struct TextureData {
    WGPUTexture texture = nullptr;
    WGPUTextureView textureView = nullptr;
    ImTextureID imguiTextureID = 0;  // ImTextureID is uint64_t in WebGPU backend
    int width = 0;
    int height = 0;
  };

  // Singleton access
  static TextureManager& getInstance();

  // Delete copy/move constructors
  TextureManager(const TextureManager&) = delete;
  TextureManager& operator=(const TextureManager&) = delete;
  TextureManager(TextureManager&&) = delete;
  TextureManager& operator=(TextureManager&&) = delete;

  /**
   * @brief Initialize the texture manager with WebGPU device.
   * Must be called before loading any textures.
   */
  void initialize(WGPUDevice device, WGPUQueue queue);

  /**
   * @brief Load a texture from the assets directory.
   * Path is relative to assets/ directory.
   * 
   * @param assetPath Path relative to assets/ (e.g. "pixel_x2.png")
   * @param upscaleFactor Nearest-neighbor upscale factor for pixel art (1 = no upscaling)
   * @param transparentFill Optional RGB fill that will be written into fully transparent pixels
   * @return TextureData containing the loaded texture, or nullptr on failure
   */
  const TextureData* loadTexture(const std::string& assetPath,
                                 int upscaleFactor = 8,
                                 std::optional<std::array<uint8_t,3>> transparentFill = std::nullopt);

  /**
   * @brief Get a previously loaded texture.
   * Returns nullptr if not loaded.
   */
  const TextureData* getTexture(const std::string& assetPath) const;

  /**
   * @brief Resolve a relative asset path to an absolute path on disk.
   */
  std::filesystem::path resolveAssetPath(const std::string& assetPath) const;

  /**
   * @brief Cleanup all textures and resources.
   */
  void cleanup();

private:
  TextureManager() = default;
  ~TextureManager();

  /**
   * @brief Get the absolute path to the assets directory.
   * Computed relative to the executable location.
   */
  std::filesystem::path getAssetsDirectory() const;

  WGPUDevice m_device = nullptr;
  WGPUQueue m_queue = nullptr;
  std::unordered_map<std::string, TextureData> m_textures;
  bool m_initialized = false;
};

} // namespace viz

