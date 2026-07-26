#pragma once
#include "AudioTools/CoreAudio/AudioBasic/Collections/Vector.h"
#include <algorithm>
#include <stdint.h>
#include <sys/time.h>
#include <vector>

namespace snap_arduino {

/**
 * @brief The the sys/time functions are used to represent the server time.
 * The local time will be measured with the help of the Arduino millis() method.
 * This class provides the basic functionality to translate between local and server time.
 * @author Phil Schatzmann
 * @version 0.1
 * @date 2023-10-28
 * @copyright Copyright (c) 2023
 */
class SnapTime {
public:
  static SnapTime &instance() {
    static SnapTime self;
    return self;
  }

  /// Provides the actual time as timeval
  timeval time() {
    timeval result;
    int rc = gettimeofday(&result, NULL);
    if (rc) {
      uint32_t ms = millis();
      result.tv_sec = ms / 1000;
      result.tv_usec = (ms - (result.tv_sec * 1000)) * 1000;
    }
    return result;
  }

  /// Provides the current server time in ms.
  /// The anchor always represents true server timeline at local anchor point.
  uint32_t serverMillis() {
    int64_t value = static_cast<int64_t>(toMillis(server_time)) -
                    static_cast<int64_t>(server_ms) +
                    static_cast<int64_t>(millis());
    if (value < 0) {
      value = 0;
    }
    return static_cast<uint32_t>(value);
  }

  uint32_t localMillis() { return toMillis(time()); }

  /// Provides the avg latecy in milliseconds
  int timeDifferenceClientServerMs() {
    int result = time_diff;
    assert(result == time_diff);
    return result;
  }

  uint32_t toMillis(timeval tv) { return toMillis(tv.tv_sec, tv.tv_usec); }

  inline uint32_t toMillis(uint32_t sec, uint32_t usec) {
    return sec * 1000 + (usec / 1000);
  }

  bool printLocalTime(const char *msg) {
    const timeval val = time();
    auto *tm_result = gmtime(&val.tv_sec);

    char str[80];
    strftime(str, 80, "%d-%m-%Y %H-%M-%S", tm_result);
    ESP_LOGI(TAG, "%s: Time is %s", msg, str);
    return true;
  }

  /// Record the last time difference between client and server
  void setTimeDifferenceClientServerMs(int32_t diff) {
    time_diff = diff;
    time_update_count++;
  }

  // updates the server-time anchor from absolute server timeval
  void updateServerTime(const timeval &server) {
    server_ms = millis();
    server_time = server;
    has_server_time_anchor = true;
  }

  // updates server time by server milliseconds at local "now"
  void updateServerTimeMs(uint32_t server_time_ms) {
    server_ms = millis();
    server_time.tv_sec = server_time_ms / 1000;
    server_time.tv_usec = (server_time_ms % 1000) * 1000;
    has_server_time_anchor = true;
  }

  bool hasServerTimeAnchor() const { return has_server_time_anchor; }

  /// Adds synchronization measurement and updates filtered offset.
  /// local_rx_ms: local receive timestamp in ms
  /// server_tx_ms: server timestamp (message send) in ms
  /// network_one_way_ms: estimated one-way network latency in ms
  /// Returns true when enough measurements are available for stable correction.
  bool addSyncMeasurement(uint32_t local_rx_ms, uint32_t server_tx_ms,
                          uint32_t network_one_way_ms,
                          uint32_t &filtered_server_ms,
                          int32_t &filtered_offset_ms) {
    int64_t corrected_server_at_rx = static_cast<int64_t>(server_tx_ms) +
                                     static_cast<int64_t>(network_one_way_ms);
    int64_t raw_offset = static_cast<int64_t>(local_rx_ms) - corrected_server_at_rx;

    if (raw_offset < INT32_MIN || raw_offset > INT32_MAX) {
      return false;
    }

    if (sync_offset_samples.size() >= max_sync_samples) {
      sync_offset_samples.pop_front();
    }
    sync_offset_samples.push_back(static_cast<int32_t>(raw_offset));

    filtered_offset_ms = median(sync_offset_samples);

    int64_t server_ms_now = static_cast<int64_t>(local_rx_ms) -
                            static_cast<int64_t>(filtered_offset_ms);
    if (server_ms_now < 0) {
      server_ms_now = 0;
    }
    filtered_server_ms = static_cast<uint32_t>(server_ms_now);

    if (sync_offset_samples.size() >= min_valid_sync_samples) {
      setTimeDifferenceClientServerMs(filtered_offset_ms);
      return true;
    }

    return false;
  }

  // Calculat the difference between 2 timeval
  timeval timeDifference(timeval t1, timeval t2) {
    timeval result;
    timersub(&t1, &t2, &result);
    return result;
  }

  // Calculat the difference between 2 timeval -> result in ms
  uint32_t timeDifferenceMs(timeval t1, timeval t2) {
    timeval result;
    timersub(&t1, &t2, &result);
    return toMillis(result);
  }

#ifdef ESP32
  void setupSNTPTime() {
    ESP_LOGD(TAG, "start");
    const char *ntpServer = CONFIG_SNTP_SERVER;
    const long gmtOffset_sec = 1 * 60 * 60;
    const int daylightOffset_sec = 1 * 60 * 60;
    for (int retry = 0; retry < 5; retry++) {
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      tm time;
      if (!getLocalTime(&time)) {
        continue;
      }
      SnapTime::instance().printLocalTime("SNTP");
      has_sntp_time = true;
      break;
    }
  }
#endif

protected:
  const char *TAG = "SnapTime";
  static constexpr size_t max_sync_samples = 200;
  static constexpr size_t min_valid_sync_samples = 20;

  int32_t median(Vector<int32_t> &values) {
    std::vector<int32_t> sorted;
    sorted.reserve(values.size());
    for (size_t j = 0; j < values.size(); j++) {
      sorted.push_back(values[j]);
    }
    std::sort(sorted.begin(), sorted.end());
    size_t mid = sorted.size() / 2;
    if (sorted.size() % 2 == 0) {
      return static_cast<int32_t>((static_cast<int64_t>(sorted[mid - 1]) +
                                   static_cast<int64_t>(sorted[mid])) /
                                  2);
    }
    return sorted[mid];
  }

  int32_t time_diff = 0;
  uint32_t server_ms = 0;
  uint32_t local_ms;
  uint32_t time_update_count = 0;
  timeval server_time{0, 0};
  bool has_sntp_time = false;
  bool has_server_time_anchor = false;
  Vector<int32_t> sync_offset_samples;
  
};

}
