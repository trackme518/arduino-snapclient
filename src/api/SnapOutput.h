#pragma once

#include <algorithm>
#include <stdint.h>
#include <sys/time.h>

#include "Arduino.h"  // for ESP.getPsramSize()
#include "AudioTools.h"
#include "SnapCommon.h"
#include "SnapConfig.h"
#include "SnapLogger.h"
#include "SnapTime.h"
#include "SnapTimeSync.h"

namespace snap_arduino {

class SnapProcessor;

/**
 * @brief Simple Output Class which uses the AudioTools to build an output chain
 * with volume control and a resampler
 * @author Phil Schatzmann
 * @version 0.1
 * @date 2023-10-28
 * @copyright Copyright (c) 2023
 **/

class SnapOutput : public AudioInfoSupport {

 public:
  SnapOutput() = default;

  /// Starts the processing which is also starting the the dsp_i2s_task_handler
  /// task
  virtual bool begin() {
    ESP_LOGI(TAG, "begin");
    is_sync_started = false;
    return audioBegin();
  }

  /// Writes audio data to the queue
  virtual size_t write(const uint8_t *data, size_t size) {
    ESP_LOGD(TAG, "%zu", size);
    // only start to proces data after we received codec header
    if (!is_audio_begin_called) {
      ESP_LOGI(TAG, "not started");
      return 0;
    }

    time_last_audio_activity = millis();

    if (!synchronizePlayback()) {
      return size;
    }

    size_t result = audioWrite(data, size);
    return result;
  }

  /// Provides info about the audio data
  virtual bool writeHeader(SnapAudioHeader &header) {
    this->header = header;
    updateChunkDurationEstimate(header);
    return true;
  }

  /// Ends the processing and releases the memory
  virtual void end(void) { resetQueueModel(); }

  /// Adjust the volume
  void setVolume(float vol) {
    this->vol = vol;
    ESP_LOGI(TAG, "Volume: %f", this->vol);
    vol_stream.setVolume(this->vol * vol_factor);
  }

  /// provides the actual volume
  float volume() { return vol; }

  /// mute / unmute
  void setMute(bool mute) {
    is_mute = mute;
    setVolume(mute ? 0.0f : vol);
    audioWriteSilence();
  }

  /// checks if volume is mute
  bool isMute() { return is_mute; }

  /// Adjust volume by factor e.g. 1.5
  void setVolumeFactor(float fact) { vol_factor = fact; }

  /// Defines the audio output chain to the final output
  void setOutput(AudioOutput &output) {
    this->out = &output;  // final output
    counted_output.setTarget(output, *this);
    resample.setOutput(counted_output);
    vol_stream.setStream(resample);  // adjust volume
    decoder_stream.setStream(&vol_stream);  // decode to pcm

    // synchronized audio information
    AudioInfo info = output.audioInfo();
    resample.begin(info, info);
    vol_stream.setAudioInfo(info);
    decoder_stream.setAudioInfo(info);
  }

  AudioOutput &getOutput() { return *out; }

  /// Defines the decoder class
  void setDecoder(AudioDecoder &dec) { decoder_stream.setDecoder(&dec); }

  AudioDecoder &getDecoder() { return decoder_stream.decoder(); }

  /// setup of all audio objects
  void setAudioInfo(AudioInfo info) {
    ESP_LOGI(TAG, "sample_rate: %d, channels: %d, bits: %d", info.sample_rate,
             info.channels, info.bits_per_sample);
    audio_info = info;
    if (is_audio_begin_called) {
      vol_stream.setAudioInfo(info);
      out->setAudioInfo(info);
    }
  }

  AudioInfo audioInfo() { return audio_info; }

  /// Defines the time synchronization logic
  void setSnapTimeSync(SnapTimeSync &timeSync) { p_snap_time_sync = &timeSync; }

  SnapTimeSync &snapTimeSync() { return *p_snap_time_sync; }

  bool isStarted() { return is_audio_begin_called; }

  void markAudioActivity() { time_last_audio_activity = millis(); }

  void setRealtimeOutputDelayLimitEnabled(bool enabled) {
    limit_realtime_output_delay = enabled;
  }

  int getDelayMsForHeader(const SnapAudioHeader &audio_header,
                          bool apply_realtime_limit = true) {
    return calculateDelayMs(audio_header, apply_realtime_limit);
  }

  int getDelayErrorMsForHeader(const SnapAudioHeader &audio_header,
                               bool apply_realtime_limit = true) {
    float output_delay_ms = estimatedOutputDelayMs();
    int delay_ms = getDelayMsForHeader(audio_header, apply_realtime_limit);
    return static_cast<int>(output_delay_ms - static_cast<float>(delay_ms));
  }

  bool isHeaderReadyForPlayback(const SnapAudioHeader &audio_header,
                                bool apply_realtime_limit = true) {
    return getDelayErrorMsForHeader(audio_header, apply_realtime_limit) >=
           -hard_sync_early_ms;
  }

  bool isHeaderTooLate(const SnapAudioHeader &audio_header,
                       bool apply_realtime_limit = true) {
    return getDelayErrorMsForHeader(audio_header, apply_realtime_limit) >
           hard_sync_late_ms;
  }

  // writes the audio data to the decoder
  size_t audioWrite(const void *src, size_t size) {
    ESP_LOGD(TAG, "audioWrite: %zu", size);
    time_last_write = millis();
    size_t result = decoder_stream.write((const uint8_t *)src, size);
    if (result != size){
      ESP_LOGW(TAG, "Could not write all data %zu -> %zu", size, result);
    }

    return result;
  }

  /// start to play audio only in valid server time: return false if to be
  /// ignored - update playback speed
  bool synchronizePlayback() {
    bool result = true;
    assert(p_snap_time_sync!=nullptr);

    SnapTimeSync &ts = *p_snap_time_sync;

    // calculate how long we need to wait to playback the audio
    auto delay_ms = getDelayMs();
    float output_delay_ms = estimatedOutputDelayMs();
    int delay_error_ms =
        static_cast<int>(output_delay_ms - static_cast<float>(delay_ms));

    if (!is_sync_started) {
      ts.begin(audio_info.sample_rate);

      // start audio when first package in the future becomes valid
      result = synchronizeOnStart(delay_ms, output_delay_ms);
    } else {
      if (delay_error_ms > hard_sync_late_ms) {
        ESP_LOGW(TAG, "audio data late: output %.1f ms, target %d ms, error %d ms",
                 output_delay_ms, delay_ms, delay_error_ms);
        return false;
      }

      if (delay_error_ms < -hard_sync_early_ms) {
        writeSilenceMs(std::min<uint32_t>(
            static_cast<uint32_t>(-delay_error_ms), max_sync_silence_step_ms));
        output_delay_ms = estimatedOutputDelayMs();
        delay_error_ms =
            static_cast<int>(output_delay_ms - static_cast<float>(delay_ms));
      }

      // provide the actual delay to the synch
      ts.updateActualDelay(delay_error_ms);

      if (ts.isSync()) {
        // update speed
        float current_factor = playbackFactor();
        float new_factor = p_snap_time_sync->getFactor();
        if (new_factor != current_factor) {
          setPlaybackFactor(new_factor);
        }
      }
    }
    return result;
  }

  uint64_t getLastWriteTime() {
    return time_last_write;
  }

  /// checks if the audio is still playing
  bool isActive(uint16_t timeout=1000){
    return (time_last_audio_activity + timeout) >= millis();
  }

 protected:
  class CountingAudioOutput : public AudioOutput {
   public:
    void setTarget(AudioOutput &target, SnapOutput &owner) {
      p_target = &target;
      p_owner = &owner;
    }

    size_t write(const uint8_t *data, size_t len) override {
      if (p_target == nullptr) {
        return 0;
      }
      size_t written = p_target->write(data, len);
      if (p_owner != nullptr && written > 0) {
        p_owner->addQueuedPcmBytes(written);
      }
      return written;
    }

    size_t write(uint8_t ch) override {
      return AudioOutput::write(ch);
    }

    int availableForWrite() override {
      return p_target == nullptr ? 0 : p_target->availableForWrite();
    }

    void flush() PRINT_FLUSH_OVERRIDE {
      AudioOutput::flush();
      if (p_target != nullptr) {
        p_target->flush();
      }
    }

    void setAudioInfo(AudioInfo newInfo) override {
      AudioOutput::setAudioInfo(newInfo);
      if (p_target != nullptr) {
        p_target->setAudioInfo(newInfo);
      }
    }

    AudioInfo audioInfo() override {
      return p_target == nullptr ? AudioOutput::audioInfo() : p_target->audioInfo();
    }

    bool begin() override {
      return p_target == nullptr ? false : p_target->begin();
    }

    void end() override {
      if (p_target != nullptr) {
        p_target->end();
      }
    }

    operator bool() override {
      return p_target != nullptr && static_cast<bool>(*p_target);
    }

   private:
    AudioOutput *p_target = nullptr;
    SnapOutput *p_owner = nullptr;
  };

  const char *TAG = "SnapOutput";
  AudioOutput *out = nullptr;
  CountingAudioOutput counted_output;
  AudioInfo audio_info;
  EncodedAudioStream decoder_stream;
  VolumeStream vol_stream;
  ResampleStream resample;
  float vol = 1.0;         // volume in the range 0.0 - 1.0
    float vol_factor = 1.0;  //
  bool is_mute = false;
  SnapAudioHeader header;
  SnapTime &snap_time = SnapTime::instance();
  SnapTimeSync *p_snap_time_sync = nullptr;
  bool is_sync_started = false;
  bool is_audio_begin_called = false;
  bool limit_realtime_output_delay = true;
  uint64_t time_last_write = 0;
  uint64_t time_last_audio_activity = 0;

  /// setup of all audio objects
  bool audioBegin() {
    if (out == nullptr) {
      ESP_LOGI(TAG, "out is null");
      return false;
    }
    if (p_snap_time_sync==nullptr){
      ESP_LOGI(TAG, "p_snap_time_sync is null");
      return false;
    }

    // determine default audio info from output 
    audio_info = out->audioInfo();

    // open volume control: allow amplification
    auto vol_cfg = vol_stream.defaultConfig();
    vol_cfg.copyFrom(audio_info);
    vol_cfg.allow_boost = true;
    vol_stream.begin(vol_cfg);
    vol_stream.setVolume(vol * vol_factor);

    // open final output
    out->setAudioInfo(audio_info);
    out->begin();

    // open decoder
    auto dec_cfg = decoder_stream.defaultConfig();
    dec_cfg.copyFrom(audio_info);
    decoder_stream.begin(dec_cfg);
    decoder_stream.addNotifyAudioChange(*this);

    // open resampler
    auto res_cfg = resample.defaultConfig();
    res_cfg.step_size = p_snap_time_sync->getFactor();
    res_cfg.copyFrom(audio_info);
    resample.begin(res_cfg);

    ESP_LOGD(TAG, "end");
    is_audio_begin_called = true;
    resetQueueModel();
    return true;
  }

  /// to speed up or slow down playback
  void setPlaybackFactor(float fact) { resample.setStepSize(fact); }

  /// determine actual playback speed
  float playbackFactor() { return resample.getStepSize(); }

  void audioWriteSilence() {
    writeSilenceMs(mute_silence_ms);
  }

  void audioEnd() {
    ESP_LOGD(TAG, "audioEnd");
    if (out == nullptr) return;
    out->end();
  }


  bool synchronizeOnStart(int delay_ms, float output_delay_ms) {
    bool result = true;
    int delay_error_ms =
        static_cast<int>(output_delay_ms - static_cast<float>(delay_ms));
    if (delay_error_ms > hard_sync_late_ms) {
      // ignore the data and report it as processed
      ESP_LOGW(TAG, "audio data expired: output %.1f ms, target %d ms",
               output_delay_ms, delay_ms);
      result = false;
    } else if (delay_ms > 100000) {
      ESP_LOGW(TAG, "invalid delay: %d ms", delay_ms);
      result = false;
    } else {
      if (delay_error_ms < 0) {
        ESP_LOGI(TAG, "priming output with %d ms silence", -delay_error_ms);
        writeSilenceMs(std::min<uint32_t>(
            static_cast<uint32_t>(-delay_error_ms), max_sync_silence_step_ms));
        output_delay_ms = estimatedOutputDelayMs();
        delay_error_ms =
            static_cast<int>(output_delay_ms - static_cast<float>(delay_ms));
      }

      assert(p_snap_time_sync!=nullptr);
      setPlaybackFactor(p_snap_time_sync->getFactor());
      is_sync_started = true;
      result = true;
    }
    return result;
  }

  void resetQueueModel() {
    queued_audio_ms = 0.0f;
    queue_update_ms = millis();
    is_queue_model_started = true;
    have_last_chunk_time = false;
    current_chunk_duration_ms = default_chunk_duration_ms;
  }

  void updateQueuedAudioDelay() {
    uint32_t now = millis();
    if (!is_queue_model_started) {
      queue_update_ms = now;
      is_queue_model_started = true;
      return;
    }
    uint32_t elapsed = now - queue_update_ms;
    queue_update_ms = now;
    queued_audio_ms -= static_cast<float>(elapsed);
    if (queued_audio_ms < 0.0f) {
      queued_audio_ms = 0.0f;
    }
  }

  float estimatedOutputDelayMs() {
    updateQueuedAudioDelay();
    return queued_audio_ms;
  }

  void addQueuedAudioMs(float ms) {
    if (ms <= 0.0f) {
      return;
    }
    updateQueuedAudioDelay();
    queued_audio_ms += ms;
    float max_delay = static_cast<float>(p_snap_time_sync->getStartDelay() + 1000);
    if (queued_audio_ms > max_delay) {
      queued_audio_ms = max_delay;
    }
  }

  void addQueuedPcmBytes(size_t bytes) {
    float bytes_per_ms = bytesPerMs();
    if (bytes_per_ms <= 0.0f || bytes == 0) {
      return;
    }
    addQueuedAudioMs(static_cast<float>(bytes) / bytes_per_ms);
  }

  float bytesPerMs() const {
    int bytes_per_sample = audio_info.bits_per_sample / 8;
    if (bytes_per_sample <= 0 || audio_info.sample_rate <= 0 ||
        audio_info.channels <= 0) {
      return 0.0f;
    }
    return (static_cast<float>(audio_info.sample_rate) *
            static_cast<float>(audio_info.channels) *
            static_cast<float>(bytes_per_sample)) /
           1000.0f;
  }

  void writeSilenceMs(uint32_t ms) {
    if (out == nullptr || ms == 0) {
      return;
    }
    float bytes_per_ms = bytesPerMs();
    if (bytes_per_ms <= 0.0f) {
      return;
    }
    uint32_t bytes = static_cast<uint32_t>(bytes_per_ms * static_cast<float>(ms));
    int frame_size = audio_info.channels * (audio_info.bits_per_sample / 8);
    if (frame_size <= 0) {
      return;
    }
    bytes = (bytes / static_cast<uint32_t>(frame_size)) *
            static_cast<uint32_t>(frame_size);
    uint8_t silence[512] = {0};
    while (bytes > 0) {
      size_t to_write = bytes > 512 ? 512 : bytes;
      to_write = (to_write / static_cast<size_t>(frame_size)) *
                 static_cast<size_t>(frame_size);
      if (to_write == 0) {
        break;
      }
      size_t written = out->write(silence, to_write);
      if (written == 0) {
        break;
      }
      addQueuedAudioMs(static_cast<float>(written) / bytes_per_ms);
      bytes -= written;
    }
  }

  void updateChunkDurationEstimate(SnapAudioHeader &h) {
    uint32_t msg_time = snap_time.toMillis(h.sec, h.usec);
    if (have_last_chunk_time) {
      int32_t delta = static_cast<int32_t>(msg_time - last_chunk_time_ms);
      if (delta > 0 && delta <= 200) {
        current_chunk_duration_ms = static_cast<float>(delta);
      } else {
        current_chunk_duration_ms = default_chunk_duration_ms;
      }
    } else {
      current_chunk_duration_ms = default_chunk_duration_ms;
    }
    last_chunk_time_ms = msg_time;
    have_last_chunk_time = true;
  }

  /// Calculate the delay in ms
  int getDelayMs() {
    return calculateDelayMs(header, limit_realtime_output_delay);
  }

  int calculateDelayMs(const SnapAudioHeader &audio_header,
                       bool apply_realtime_limit) {
    assert(p_snap_time_sync!=nullptr);

    // If no time anchor exists yet, keep buffering with configured start delay.
    if (!snap_time.hasServerTimeAnchor()) {
      return p_snap_time_sync->getStartDelay();
    }

    uint32_t msg_time = snap_time.toMillis(audio_header.sec, audio_header.usec);
    uint32_t server_time = snap_time.serverMillis();

    // Signed subtraction on wrapped 32-bit server timeline.
    int32_t diff_ms = static_cast<int32_t>(msg_time - server_time);
    int32_t delay_ms = diff_ms + p_snap_time_sync->getStartDelay();
    if (apply_realtime_limit && delay_ms > max_realtime_output_delay_ms) {
      delay_ms = max_realtime_output_delay_ms;
    }
    return static_cast<int>(delay_ms);
  }

  static constexpr int hard_sync_late_ms = 50;
  static constexpr int hard_sync_early_ms = 5;
  static constexpr uint32_t max_sync_silence_step_ms = 20;
  static constexpr int max_realtime_output_delay_ms = 40;
  static constexpr uint32_t mute_silence_ms = 250;
  static constexpr float default_chunk_duration_ms = 20.0f;
  float queued_audio_ms = 0.0f;
  uint32_t queue_update_ms = 0;
  bool is_queue_model_started = false;
  bool have_last_chunk_time = false;
  uint32_t last_chunk_time_ms = 0;
  float current_chunk_duration_ms = default_chunk_duration_ms;
};

}
