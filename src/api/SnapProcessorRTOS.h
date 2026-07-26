#pragma once
#include "SnapOutput.h"
#if defined(AUDIOTOOLS_MAJOR_VERSION) 
#  include "AudioTools/AudioLibs/Concurrency.h"
#else
#  include "AudioLibs/Concurrency.h"
#endif

namespace snap_arduino {

/**
 * @brief Processor for which the encoded output is buffered in a queue in order to
 * prevent any buffer underruns. A RTOS task feeds the output from the queue.
 * @author Phil Schatzmann
 * @version 0.1
 * @date 2024-02-26
 * @copyright Copyright (c) 2023
 */
class SnapProcessorRTOS : public SnapProcessor {
 public:
  /// Default constructor
  SnapProcessorRTOS(SnapOutput &output, int buffer_size,
                    int activationAtPercent = 75)
      : SnapProcessor(output) {
    init_rtos(buffer_size, activationAtPercent);
  }
  /// Default constructor
  SnapProcessorRTOS(int buffer_size, int activationAtPercent = 75)
      : SnapProcessor() {
    init_rtos(buffer_size, activationAtPercent);
  }

  bool begin() override {
    // regular begin logic
    bool result = SnapProcessor::begin();
    // allocate buffer, so that we could use psram
    chunk_queue.resize(RTOS_MAX_QUEUE_ENTRY_COUNT);
    chunk_queue.setReadMaxWait(0);
    chunk_queue.setWriteMaxWait(5);
    buffer.resize(buffer_size);
    buffer.setReadMaxWait(0);
    buffer.setWriteMaxWait(5);
    p_snap_output->setRealtimeOutputDelayLimitEnabled(false);
    task_started = false;
    return result;
  }

  void end(void) override {
    task.suspend();
    task_started = false;
    chunk_queue.clear();
    buffer.reset();
    SnapProcessor::end();
  }

 protected:
  struct QueuedChunk {
    SnapAudioHeader header;
    size_t size = 0;
  };

  const char *TAG = "SnapProcessorRTOS";
  audio_tools::Task task{"output", RTOS_STACK_SIZE, RTOS_TASK_PRIORITY, -1};
  audio_tools::QueueRTOS<QueuedChunk> chunk_queue{0};
  audio_tools::BufferRTOS<uint8_t> buffer{0}; // size defined in constructor
  bool task_started = false;
  int active_percent;
  int buffer_size;
  static SnapProcessorRTOS *self;

  /// store parameters provided by constructor
  void init_rtos(int bufferSize, int activationAtPercent) {
    self = this;
    active_percent = activationAtPercent;
    buffer_size = bufferSize;
  }

  /// Writes the encoded audio data to a queue
  size_t writeAudio(const uint8_t *data, size_t size) override {

    if (size > buffer.size()){
      ESP_LOGE(TAG, "The buffer is too small. Use a multiple of %d", size);
      stop();
    }
    
    ESP_LOGD(TAG, "size: %zu / buffer %d", size, buffer.available());
    if (!p_snap_output->isStarted() || size == 0) {
      ESP_LOGW(TAG, "not started");
      return 0;
    }

    p_snap_output->markAudioActivity();

    if (buffer.availableForWrite() < static_cast<int>(size)) {
      ESP_LOGW(TAG, "encoded buffer full: need %zu, free %d", size,
               buffer.availableForWrite());
      return 0;
    }

    size_t size_written = buffer.writeArray(data, size);
    if (size_written != size) {
      ESP_LOGE(TAG, "buffer-overflow");
      buffer.reset();
      chunk_queue.clear();
      return 0;
    }

    QueuedChunk chunk;
    chunk.header = current_audio_header;
    chunk.size = size;
    if (!chunk_queue.enqueue(chunk)) {
      ESP_LOGW(TAG, "chunk_queue full");
      buffer.reset();
      chunk_queue.clear();
      return 0;
    }

    ESP_LOGD(TAG, "buffer %d - %d vs limit %d", size, buffer.available(),
             bufferTaskActivationLimit());
    if (!task_started && buffer.available() >= bufferTaskActivationLimit()) {
      ESP_LOGI(TAG, "===> starting output task");
      task_started = true;
      task.begin(task_copy);
    }

    return size_written;
  }

  /// Determines the buffer fill limit at which we start to process the data
  int bufferTaskActivationLimit() {
    return static_cast<float>(active_percent) / 100.0 * buffer.size();
  }

  /// Copy the buffered data to the output
  void copy() {
    QueuedChunk chunk;
    if (!chunk_queue.peek(chunk)) {
      delay(1);
      return;
    }

    if (p_snap_output->isHeaderTooLate(chunk.header, false)) {
      if (chunk_queue.dequeue(chunk)) {
        discardPayload(chunk.size);
        ESP_LOGW(TAG, "dropped late chunk: %zu bytes", chunk.size);
      }
      delay(1);
      return;
    }

    if (!p_snap_output->isHeaderReadyForPlayback(chunk.header, false)) {
      delay(1);
      return;
    }

    if (!chunk_queue.dequeue(chunk)) {
      delay(1);
      return;
    }

    uint8_t data[chunk.size];
    int read = buffer.readArray(data, chunk.size);
    if (read != static_cast<int>(chunk.size)) {
      ESP_LOGE(TAG, "readArray failed %d -> %d", static_cast<int>(chunk.size),
               read);
      buffer.reset();
      chunk_queue.clear();
      delay(1);
      return;
    }

    p_snap_output->writeHeader(chunk.header);
    int written = p_snap_output->write(data, chunk.size);
    if (written != static_cast<int>(chunk.size)) {
      ESP_LOGW(TAG, "write %d of %d", written, static_cast<int>(chunk.size));
    }
    delay(1);
  }

  void discardPayload(size_t size) {
    uint8_t discard[256];
    while (size > 0) {
      size_t to_read = size > sizeof(discard) ? sizeof(discard) : size;
      int read = buffer.readArray(discard, to_read);
      if (read <= 0) {
        buffer.reset();
        chunk_queue.clear();
        return;
      }
      size -= static_cast<size_t>(read);
    }
  }

  /// static method for rtos task: make sure we constantly output audio
  static void task_copy() {
    while (self != nullptr) self->copy();
  }
};

SnapProcessorRTOS *SnapProcessorRTOS::self = nullptr;

}
