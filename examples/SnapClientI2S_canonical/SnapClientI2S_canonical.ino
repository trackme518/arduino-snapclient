/**
 * @brief SnapClient with Opus decoder: I2S OUtput on an ESP32
 * @author Phil Schatzmann
 * @copyright GPLv3
 */

#include "AudioTools.h"
#include "SnapClient.h"
#include "api/SnapProcessorRTOS.h"
#include "AudioTools/AudioCodecs/CodecOpus.h"

#define ARDUINO_LOOP_STACK_SIZE (10 * 1024)
#define SNAP_ENCODED_BUFFER_BYTES (48 * 1024)
#define SNAP_ENCODED_BUFFER_ACTIVATION_PERCENT 0

OpusAudioDecoder opus;
I2SStream out;
WiFiClient wifi;
SnapTimeSyncDynamic synch(0, 10);
SnapProcessorRTOS snapProcessorRTOS(SNAP_ENCODED_BUFFER_BYTES,
                                    SNAP_ENCODED_BUFFER_ACTIVATION_PERCENT);
SnapClient client(wifi, out, opus);

void setup() {
  Serial.begin(115200);

  // login to wifi -> Define values in SnapConfig.h or replace them here
  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  // print ip address
  Serial.println();
  Serial.println(WiFi.localIP());

  // setup I2S to define custom pins
  auto cfg = out.defaultConfig(TX_MODE);
  AudioInfo audio_format_i2s(48000, 2, 16);
  cfg.copyFrom(audio_format_i2s);
  cfg.pin_bck = 14;
  cfg.pin_ws = 15;
  cfg.pin_data = 22;
  cfg.buffer_size = 512;
  cfg.buffer_count = 4;
  out.begin(cfg);

  synch.set_ppm_threshold(50);
  synch.set_ppm_max_correction(1000);
  synch.set_ppm_samples(20);
  synch.set_ppm_smoothing(0.05f);

  // Use timestamped RTOS buffering: encoded Opus chunks are queued with their
  // Snapcast timestamps and scheduled against the server-provided bufferMs.
  client.setSnapProcessor(snapProcessorRTOS);
  client.snapProcessor().setFastLoop(true);

  // Define CONFIG_SNAPCAST_SERVER_HOST in SnapConfig.h or here
  // client.setServerIP(IPAddress(192,168,1,38));

  // start snap client
  client.begin(synch);
}

void loop() {
  client.doLoop();
}
