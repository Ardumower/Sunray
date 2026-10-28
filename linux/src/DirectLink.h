#pragma once
#include <cstddef>
#include <string>
// Loopback-only bridge to the authenticated WebRTC helper. Commands are consumed
// exclusively on Sunray's control thread; camera callbacks never perform I/O.
namespace DirectLink {
constexpr unsigned long DRIVE_TIMEOUT_MS = 3000;
void start();
bool takeDrive(std::string& command);
bool ownsDrive();
void camera(const unsigned char* data, size_t size);
void setCloudCamera(bool enabled);
bool cloudCamera();
bool rpc(const std::string& request, std::string& response);
}
