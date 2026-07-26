#pragma once
#include "AudioTools.h"
#include "SnapLogger.h"
#include "SnapTime.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace snap_arduino {

/**
 * @brief Abstract (Common) Time Synchronization Logic which consists of the
 * startup synchronization and the local to server clock synchronization which
 * adjusts the sampling rate.
 * @author Phil Schatzmann
 * @version 0.1
 * @date 2023-10-28
 * @copyright Copyright (c) 2023
*/
class SnapTimeSync {
public:
  SnapTimeSync(int processingLag = CONFIG_PROCESSING_TIME_MS, int interval = 10) {
    setInterval(interval);
    setProcessingLag(processingLag);
  }

  /// Starts the processing
  virtual void begin(int rate) {
    (void)rate;
    update_count = 0;
    active = false;
  }

  /// Records one sync measurement using explicit client monotonic and server timestamps.
  virtual void updateSyncMeasurement(uint32_t clientMonotonicMs,
                                     uint32_t serverMillis) = 0;

  /// Records the actual playback delay (currently not used)
  virtual void updateActualDelay(int delay) {}

  /// Calculate the resampling factor: with a positive delay we play too fast
  /// and need to slow down
  virtual float getFactor() = 0;

  /// Returns true if a synchronization (update of the sampling rate) is needed.
  bool isSync() {
    bool result = active && update_count > 2 && update_count % interval == 0;
    active = false;
    return result;
  }

  /// Defines the message buffer lag
  void setMessageBufferDelay(int ms) {
    ESP_LOGI(TAG, "delay: %d", ms);
    message_buffer_delay_ms = ms;
  }

  /// Defines the lag which is substracted from the message_buffer_delay_ms. It
  /// conists of the delay added by the decoder and your selected output device
  void setProcessingLag(int lag) { this->processing_lag = lag; }

  /// Defines the interval that is used to adjust the sample rate: 10 means
  /// every 10 updates.
  void setInterval(int interval) { this->interval = interval; }

  /// Provides the effective delay to be used (message buffer lag -
  /// decoder/playback latency).
  int getStartDelay() {
    int delay = std::max(0, message_buffer_delay_ms - processing_lag);
    if (message_buffer_delay_ms - processing_lag < 0){
      LOGE("The processing lag can not be bigger then %d", message_buffer_delay_ms);
    }
    ESP_LOGD(TAG, "delay: %d", delay);
    return delay;
  }

protected:
  const char *TAG = "SnapTimeSync";
  // resampling speed
  uint64_t update_count = 0;
  int interval = 10;
  bool active = false;
  // start delay
  int processing_lag = 0;
  int message_buffer_delay_ms = 0;

};

/**
 * @brief Dynamically adjusts the effective playback sample rate based on the differences
 * of the local and server clock between the different intervals
 * @author Phil Schatzmann
 * @version 0.1
 * @date 2023-10-28
 * @copyright Copyright (c) 2023
 **/
class SnapTimeSyncDynamic : public SnapTimeSync {
public:
  SnapTimeSyncDynamic(int processingLag = CONFIG_PROCESSING_TIME_MS,
                      int interval = 10)
      : SnapTimeSync(processingLag, interval) {}

  void begin(int rate) override {
    SnapTimeSync::begin(rate);
    raw_offsets.clear();
    offset_points.clear();
    playback_errors.clear();
    smoothed_factor = 1.0f;
    clock_factor = 1.0f;
    playback_factor = 1.0f;
  }

  void updateSyncMeasurement(uint32_t clientMonotonicMs,
                             uint32_t serverMillis) override {
    update_count++;
    active = true;

    int32_t raw_offset = static_cast<int32_t>(clientMonotonicMs - serverMillis);
    if (raw_offsets.size() >= ppm_samples_window) {
      raw_offsets.pop_front();
    }
    raw_offsets.push_back(raw_offset);

    int32_t filtered_offset = median(raw_offsets);

    if (offset_points.size() >= ppm_samples_window) {
      offset_points.pop_front();
    }
    offset_points.push_back(OffsetPoint{clientMonotonicMs, filtered_offset});

    recomputeFactor();
  }

  void updateActualDelay(int delay) override {
    if (playback_errors.size() >= playback_error_window) {
      playback_errors.pop_front();
    }
    playback_errors.push_back(delay);
    recomputeFactor();
  }

  float getFactor() override { return smoothed_factor; }

  void set_ppm_threshold(float ppm) {
    ppm_threshold = std::max(0.0f, ppm);
  }

  void set_ppm_max_correction(float ppm) {
    ppm_max_correction = std::max(1.0f, ppm);
  }

  void set_ppm_samples(size_t samples) {
    ppm_samples_window = std::max(min_valid_measurements, samples);
    while (raw_offsets.size() > ppm_samples_window) {
      raw_offsets.pop_front();
    }
    while (offset_points.size() > ppm_samples_window) {
      offset_points.pop_front();
    }
  }

  void set_ppm_smoothing(float alpha) {
    if (alpha < 0.0f) {
      alpha = 0.0f;
    }
    if (alpha > 1.0f) {
      alpha = 1.0f;
    }
    ppm_smoothing_alpha = alpha;
  }

protected:
  struct OffsetPoint {
    uint32_t local_ms;
    int32_t filtered_offset_ms;
  };

  static constexpr size_t min_valid_measurements = 20;

  float ppm_threshold = 50.0f;
  float ppm_max_correction = 1000.0f;
  size_t ppm_samples_window = 20;
  float ppm_smoothing_alpha = 0.05f;
  float smoothed_factor = 1.0f;
  float clock_factor = 1.0f;
  float playback_factor = 1.0f;

  Vector<int32_t> raw_offsets;
  Vector<OffsetPoint> offset_points;
  Vector<int32_t> playback_errors;
  size_t playback_error_window = 20;

  int32_t median(Vector<int32_t> &values) {
    std::vector<int32_t> sorted;
    sorted.reserve(values.size());
    for (size_t i = 0; i < values.size(); i++) {
      sorted.push_back(values[i]);
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

  void recomputeFactor() {
    if (offset_points.size() >= min_valid_measurements &&
        raw_offsets.size() >= min_valid_measurements) {
      const OffsetPoint &oldest = offset_points[0];
      const OffsetPoint &newest = offset_points[offset_points.size() - 1];
      int32_t delta_local = static_cast<int32_t>(newest.local_ms - oldest.local_ms);
      if (delta_local > 0) {
        int32_t delta_offset = newest.filtered_offset_ms - oldest.filtered_offset_ms;
        float slope = static_cast<float>(delta_offset) / static_cast<float>(delta_local);
        float drift_ppm = slope * 1000000.0f;

        if (std::fabs(drift_ppm) < ppm_threshold) {
          drift_ppm = 0.0f;
        }

        if (drift_ppm > ppm_max_correction) {
          drift_ppm = ppm_max_correction;
        } else if (drift_ppm < -ppm_max_correction) {
          drift_ppm = -ppm_max_correction;
        }

        clock_factor = 1.0f - (drift_ppm / 1000000.0f);
      }
    }

    if (playback_errors.size() >= min_valid_measurements) {
      int32_t error_ms = median(playback_errors);
      float playback_ppm = static_cast<float>(error_ms) * 50.0f;
      if (playback_ppm > ppm_max_correction) {
        playback_ppm = ppm_max_correction;
      } else if (playback_ppm < -ppm_max_correction) {
        playback_ppm = -ppm_max_correction;
      }
      playback_factor = 1.0f + (playback_ppm / 1000000.0f);
    }

    float target_factor = clock_factor * playback_factor;
    smoothed_factor += ppm_smoothing_alpha * (target_factor - smoothed_factor);
  }
};

/**
 * @brief Dynamically adjusts the effective playback sample rate based on the differences
 * of the local and server clock since the start
 * @author Phil Schatzmann
 * @version 0.1
 * @date 2023-10-28
 * @copyright Copyright (c) 2023
 **/
class SnapTimeSyncDynamicSinceStart : public SnapTimeSync {
public:
  SnapTimeSyncDynamicSinceStart(int processingLag = CONFIG_PROCESSING_TIME_MS,
                      int interval = 10)
      : SnapTimeSync(processingLag, interval) {}

  void begin(int rate) override {
    SnapTimeSync::begin(rate);
    start_time.local_ms = 0;
    start_time.server_ms = 0;
    current_time.local_ms = 0;
    current_time.server_ms = 0;
  }

  void updateSyncMeasurement(uint32_t clientMonotonicMs,
                             uint32_t serverMillis) override {
    if (update_count == 0){
      start_time.local_ms = clientMonotonicMs;
      start_time.server_ms = serverMillis;
    }
    current_time.local_ms = clientMonotonicMs;
    current_time.server_ms = serverMillis;
    update_count++;
    active = true;
  }

  float getFactor() {
    if (update_count < 20) {
      return 1.0;
    }

    float timespan_local_ms = current_time.local_ms - start_time.local_ms;
    float timespan_server_ms = current_time.server_ms - start_time.server_ms;
    if (timespan_local_ms == 0.0 || timespan_server_ms == 0.0) {
      ESP_LOGE(TAG, "Could not determine clock differences");
      return 1.0;
    }
    // if server time span is smaller then local, local runs faster and needs to be slowed down
    float result_factor = timespan_server_ms / timespan_local_ms;    
    return result_factor;
  }
protected:
  SnapTimePoints start_time;
  SnapTimePoints current_time;
};


/**
 * @brief Uses predefined fixed factor
 * @author Phil Schatzmann
 * @version 0.1
 * @date 2023-10-28
 * @copyright Copyright (c) 2023
 **/
class SnapTimeSyncFixed : public SnapTimeSync {
public:
  SnapTimeSyncFixed(int processingLag = CONFIG_PROCESSING_TIME_MS, float factor = 1.0,
                    int interval = 10)
      : SnapTimeSync(processingLag, interval) {
    resample_factor = factor;
  }

  void updateSyncMeasurement(uint32_t clientMonotonicMs,
                             uint32_t serverMillis) override {
    (void)clientMonotonicMs;
    (void)serverMillis;
  }

  float getFactor() { return resample_factor; }

protected:
  float resample_factor;
};

}
