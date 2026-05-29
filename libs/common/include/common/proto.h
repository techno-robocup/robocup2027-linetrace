// Wire protocol between the laptop control station and the robot collector.
//
//   control station --(ControlPacket, UDP ~50 Hz)--> collector
//   collector       --(preview JPEG fragments, UDP)--> control station
//
// All multi-byte fields are big-endian and packed explicitly (no struct-layout
// assumptions) so x86 laptop and ARM Pi interoperate. Header-only and
// dependency-free.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rc {

// ---- control packet (station -> robot) ----
inline constexpr uint32_t kControlMagic = 0x52435450;  // 'RCTP'
inline constexpr uint16_t kControlVersion = 1;
inline constexpr size_t kControlPacketSize = 38;

namespace flags {
inline constexpr uint16_t kRecording = 1 << 0;
inline constexpr uint16_t kEstop = 1 << 1;
inline constexpr uint16_t kHeartbeat = 1 << 2;
}  // namespace flags

struct ControlPacket {
  uint32_t seq = 0;
  uint64_t tsMs = 0;
  int16_t left = 1500;
  int16_t right = 1500;
  int16_t axes[6] = {0, 0, 0, 0, 0, 0};  // lx, ly, rx, ry, lt, rt scaled to int16
  uint16_t flagBits = 0;

  bool recording() const { return flagBits & flags::kRecording; }
  bool estop() const { return flagBits & flags::kEstop; }
};

// ---- preview fragment header (robot -> station), precedes JPEG bytes ----
inline constexpr uint32_t kPreviewMagic = 0x52505657;  // 'RPVW'
inline constexpr size_t kPreviewHeaderSize = 16;

struct PreviewFragHeader {
  uint32_t frameId = 0;
  uint16_t fragIndex = 0;
  uint16_t fragCount = 0;
  uint32_t totalSize = 0;  // full JPEG size in bytes
};

// --- big-endian byte helpers ---
namespace detail {
inline void put16(uint8_t* p, uint16_t v) {
  p[0] = uint8_t(v >> 8);
  p[1] = uint8_t(v);
}
inline void put32(uint8_t* p, uint32_t v) {
  p[0] = uint8_t(v >> 24);
  p[1] = uint8_t(v >> 16);
  p[2] = uint8_t(v >> 8);
  p[3] = uint8_t(v);
}
inline void put64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; ++i) p[i] = uint8_t(v >> (56 - 8 * i));
}
inline uint16_t get16(const uint8_t* p) { return uint16_t(p[0]) << 8 | p[1]; }
inline uint32_t get32(const uint8_t* p) {
  return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];
}
inline uint64_t get64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
  return v;
}
}  // namespace detail

// CRC-16/CCITT-FALSE over [data, data+len).
inline uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

// Serializes into buf (must be >= kControlPacketSize). Returns bytes written.
inline size_t encodeControl(const ControlPacket& c, uint8_t* buf) {
  using namespace detail;
  put32(buf + 0, kControlMagic);
  put16(buf + 4, kControlVersion);
  put16(buf + 6, c.flagBits);
  put32(buf + 8, c.seq);
  put64(buf + 12, c.tsMs);
  put16(buf + 20, uint16_t(c.left));
  put16(buf + 22, uint16_t(c.right));
  for (int i = 0; i < 6; ++i) put16(buf + 24 + 2 * i, uint16_t(c.axes[i]));
  put16(buf + 36, crc16(buf, 36));
  return kControlPacketSize;
}

// Validates magic, version and crc; fills out. Returns false on any mismatch.
inline bool decodeControl(const uint8_t* buf, size_t len, ControlPacket& out) {
  using namespace detail;
  if (len < kControlPacketSize) return false;
  if (get32(buf + 0) != kControlMagic) return false;
  if (get16(buf + 4) != kControlVersion) return false;
  if (get16(buf + 36) != crc16(buf, 36)) return false;
  out.flagBits = get16(buf + 6);
  out.seq = get32(buf + 8);
  out.tsMs = get64(buf + 12);
  out.left = int16_t(get16(buf + 20));
  out.right = int16_t(get16(buf + 22));
  for (int i = 0; i < 6; ++i) out.axes[i] = int16_t(get16(buf + 24 + 2 * i));
  return true;
}

inline size_t encodePreviewHeader(const PreviewFragHeader& h, uint8_t* buf) {
  using namespace detail;
  put32(buf + 0, kPreviewMagic);
  put32(buf + 4, h.frameId);
  put16(buf + 8, h.fragIndex);
  put16(buf + 10, h.fragCount);
  put32(buf + 12, h.totalSize);
  return kPreviewHeaderSize;
}

inline bool decodePreviewHeader(const uint8_t* buf, size_t len, PreviewFragHeader& out) {
  using namespace detail;
  if (len < kPreviewHeaderSize) return false;
  if (get32(buf + 0) != kPreviewMagic) return false;
  out.frameId = get32(buf + 4);
  out.fragIndex = get16(buf + 8);
  out.fragCount = get16(buf + 10);
  out.totalSize = get32(buf + 12);
  return true;
}

}  // namespace rc
