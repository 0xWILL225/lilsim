#include "TextureManager.hpp"
#include <spdlog/spdlog.h>
#include <imgui_impl_wgpu.h>
#include <cstdlib>

#define STB_IMAGE_IMPLEMENTATION
#include "ImGuiFileDialog/stb/stb_image.h"

#ifdef __linux__
#include <unistd.h>
#include <linux/limits.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#include <limits.h>
#elif defined(_WIN32)
#include <windows.h>
#endif

namespace viz {

TextureManager& TextureManager::getInstance() {
  static TextureManager instance;
  return instance;
}

TextureManager::~TextureManager() {
  cleanup();
}

void TextureManager::initialize(WGPUDevice device, WGPUQueue queue) {
  m_device = device;
  m_queue = queue;
  m_initialized = true;
  spdlog::info("[TextureManager] Initialized");
}

std::filesystem::path TextureManager::getAssetsDirectory() const {
  std::filesystem::path exePath;

#ifdef __linux__
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  if (count != -1) {
    exePath = std::filesystem::path(std::string(result, count));
  }
#elif defined(__APPLE__)
  char result[PATH_MAX];
  uint32_t size = PATH_MAX;
  if (_NSGetExecutablePath(result, &size) == 0) {
    exePath = std::filesystem::path(result);
  }
#elif defined(_WIN32)
  char result[MAX_PATH];
  GetModuleFileNameA(nullptr, result, MAX_PATH);
  exePath = std::filesystem::path(result);
#endif

  if (exePath.empty()) {
    spdlog::error("[TextureManager] Failed to get executable path");
    return std::filesystem::current_path() / "assets";
  }

  // Executable is at build/debug/app/lilsim
  // Assets are at assets/
  // So we go up 3 levels: ../ ../ ../ then into assets/
  return exePath.parent_path().parent_path().parent_path().parent_path() / "assets";
}

const TextureManager::TextureData* TextureManager::loadTexture(const std::string& assetPath, int upscaleFactor) {
  if (!m_initialized) {
    spdlog::error("[TextureManager] Not initialized. Call initialize() first.");
    return nullptr;
  }

  // Check if already loaded
  auto it = m_textures.find(assetPath);
  if (it != m_textures.end()) {
    return &it->second;
  }

  // Construct full path
  std::filesystem::path fullPath = getAssetsDirectory() / assetPath;
  spdlog::info("[TextureManager] Loading texture: {}", fullPath.string());

  // Load image data using stb_image
  int originalWidth, originalHeight, channels;
  unsigned char* originalData = stbi_load(fullPath.string().c_str(), &originalWidth, &originalHeight, &channels, 4);
  
  if (!originalData) {
    spdlog::error("[TextureManager] Failed to load image: {}", fullPath.string());
    return nullptr;
  }

  spdlog::info("[TextureManager] Loaded {}x{} image with {} channels", originalWidth, originalHeight, channels);

  // Upscale using nearest-neighbor for pixel art
  int width = originalWidth * upscaleFactor;
  int height = originalHeight * upscaleFactor;
  
  unsigned char* imageData = nullptr;
  if (upscaleFactor > 1) {
    spdlog::info("[TextureManager] Upscaling {}x to {}x{} using nearest-neighbor", 
                 upscaleFactor, width, height);
    
    imageData = static_cast<unsigned char*>(std::malloc(static_cast<size_t>(width * height * 4)));
    if (!imageData) {
      spdlog::error("[TextureManager] Failed to allocate memory for upscaled texture");
      stbi_image_free(originalData);
      return nullptr;
    }
    
    // Nearest-neighbor upscaling
    for (int y = 0; y < height; ++y) {
      int srcY = y / upscaleFactor;
      for (int x = 0; x < width; ++x) {
        int srcX = x / upscaleFactor;
        int srcIdx = (srcY * originalWidth + srcX) * 4;
        int dstIdx = (y * width + x) * 4;
        
        imageData[dstIdx + 0] = originalData[srcIdx + 0];
        imageData[dstIdx + 1] = originalData[srcIdx + 1];
        imageData[dstIdx + 2] = originalData[srcIdx + 2];
        imageData[dstIdx + 3] = originalData[srcIdx + 3];
      }
    }
    
    stbi_image_free(originalData);
  } else {
    // No upscaling
    imageData = originalData;
  }

  // Create WebGPU texture
  WGPUTextureDescriptor textureDesc = {};
  textureDesc.label = WGPUStringView{assetPath.c_str(), assetPath.size()};
  textureDesc.size.width = width;
  textureDesc.size.height = height;
  textureDesc.size.depthOrArrayLayers = 1;
  textureDesc.mipLevelCount = 1;
  textureDesc.sampleCount = 1;
  textureDesc.dimension = WGPUTextureDimension_2D;
  textureDesc.format = WGPUTextureFormat_RGBA8Unorm;
  textureDesc.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;

  WGPUTexture texture = wgpuDeviceCreateTexture(m_device, &textureDesc);
  if (!texture) {
    spdlog::error("[TextureManager] Failed to create WebGPU texture");
    stbi_image_free(imageData);
    return nullptr;
  }

  // Upload image data to texture
  WGPUTexelCopyTextureInfo destination = {};
  destination.texture = texture;
  destination.mipLevel = 0;
  destination.origin = {0, 0, 0};
  destination.aspect = WGPUTextureAspect_All;

  WGPUTexelCopyBufferLayout dataLayout = {};
  dataLayout.offset = 0;
  dataLayout.bytesPerRow = static_cast<uint32_t>(4 * width);
  dataLayout.rowsPerImage = static_cast<uint32_t>(height);

  WGPUExtent3D writeSize = {};
  writeSize.width = static_cast<uint32_t>(width);
  writeSize.height = static_cast<uint32_t>(height);
  writeSize.depthOrArrayLayers = 1;

  wgpuQueueWriteTexture(m_queue, &destination, imageData, static_cast<size_t>(4 * width * height), &dataLayout, &writeSize);

  // Free image data
  if (upscaleFactor > 1) {
    std::free(imageData);
  } else {
    stbi_image_free(imageData);
  }

  // Create texture view
  WGPUTextureViewDescriptor viewDesc = {};
  viewDesc.format = WGPUTextureFormat_RGBA8Unorm;
  viewDesc.dimension = WGPUTextureViewDimension_2D;
  viewDesc.baseMipLevel = 0;
  viewDesc.mipLevelCount = 1;
  viewDesc.baseArrayLayer = 0;
  viewDesc.arrayLayerCount = 1;
  viewDesc.aspect = WGPUTextureAspect_All;

  WGPUTextureView textureView = wgpuTextureCreateView(texture, &viewDesc);
  if (!textureView) {
    spdlog::error("[TextureManager] Failed to create texture view");
    wgpuTextureRelease(texture);
    return nullptr;
  }

  // In WebGPU backend, ImTextureID is just the WGPUTextureView cast to uint64_t
  // ImGui will use its own sampler during rendering
  // We've pre-upscaled the texture so even with linear filtering it looks sharp
  ImTextureID imguiTextureID = reinterpret_cast<ImTextureID>(textureView);

  // Store texture data (dimensions are the upscaled dimensions)
  TextureData data;
  data.texture = texture;
  data.textureView = textureView;
  data.imguiTextureID = imguiTextureID;
  data.width = width;
  data.height = height;

  m_textures[assetPath] = data;

  spdlog::info("[TextureManager] Successfully loaded texture: {}", assetPath);
  return &m_textures[assetPath];
}

const TextureManager::TextureData* TextureManager::getTexture(const std::string& assetPath) const {
  auto it = m_textures.find(assetPath);
  if (it != m_textures.end()) {
    return &it->second;
  }
  return nullptr;
}

void TextureManager::cleanup() {
  spdlog::info("[TextureManager] Cleaning up {} textures", m_textures.size());
  
  for (auto& [path, data] : m_textures) {
    if (data.textureView) {
      wgpuTextureViewRelease(data.textureView);
    }
    if (data.texture) {
      wgpuTextureRelease(data.texture);
    }
  }
  
  m_textures.clear();
  m_initialized = false;
}

} // namespace viz

