/*
 * ESPectre - Wi-Fi CSI Movement Detection for ESP32-S3
 *
 * Uses Channel State Information (CSI) from Wi-Fi packets to detect movement.
 * Extracts 8 mathematical features and combines them with configurable weights
 * to distinguish between static environment and human movement.
 * 
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * License: GPLv3
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_timer.h"

// Module headers
#include "calibration.h"
#include "nvs_storage.h"
#include "filters.h"
#include "csi_processor.h"
#include "detection_engine.h"
#include "statistics.h"
#include "config_manager.h"
#include "wifi_manager.h"
#include "mqtt_handler.h"
#include "mqtt_commands.h"
#include "traffic_generator.h"

// Configuration - can be overridden via menuconfig
#define WIFI_SSID           CONFIG_WIFI_SSID
#define WIFI_PASSWORD       CONFIG_WIFI_PASSWORD
#define MQTT_BROKER_URI     CONFIG_MQTT_BROKER_URI
#define MQTT_TOPIC          CONFIG_MQTT_TOPIC
#define MQTT_USERNAME       CONFIG_MQTT_USERNAME
#define MQTT_PASSWORD       CONFIG_MQTT_PASSWORD

// Default threshold
#define DEFAULT_THRESHOLD   0.40f

// Logging intervals
#define LOG_CSI_VALUES_INTERVAL 1
#define STATS_LOG_INTERVAL  100

// Buffer sizes
#define STATS_BUFFER_SIZE   100

// Publishing configuration
#define PUBLISH_INTERVAL    1.0f
#define CONFIDENCE_THRESHOLD 0.5f

static const char *TAG = "ESPectre";

// Global MQTT response topic
static const char *g_response_topic = NULL;

// Global state structure
static struct {
    stats_buffer_t stats_buffer;
    
    float detection_score;
    float threshold_high;
    float threshold_low;
    
    detection_state_t state;
    uint8_t consecutive_detections;
    int64_t last_detection_time;
    float confidence;
    
    uint32_t packets_received;
    uint32_t packets_processed;
    _Atomic uint32_t packets_dropped;
    
    csi_features_t current_features;
    
    // Module instances
    wifi_manager_state_t wifi_state;
    mqtt_handler_state_t mqtt_state;
    runtime_config_t config;
    
} g_state = {0};

// Mutex to protect g_state from concurrent access
static SemaphoreHandle_t g_state_mutex = NULL;

// Filter module instances
static butterworth_filter_t g_butterworth = {0};
static filter_buffer_t g_filter_buffer = {0};
static adaptive_normalizer_t g_normalizer = {0};

// Adaptive normalizer reset tracking
static int64_t g_last_movement_time = 0;
static uint32_t g_normalizer_reset_count = 0;

// MQTT command context
static mqtt_cmd_context_t g_mqtt_cmd_context = {0};

// MQTT command callback
static void mqtt_command_callback(const char *data, int data_len) {
    mqtt_commands_process(data, data_len, &g_mqtt_cmd_context, 
                         &g_state.mqtt_state, MQTT_TOPIC "/response");
}

static int64_t get_timestamp_ms(void) {
    return esp_timer_get_time() / 1000;
}

static int64_t get_timestamp_sec(void) {
    return esp_timer_get_time() / 1000000;
}

// Helper function to format progress bar with threshold marker (UTF-8 version)
static void format_progress_bar(char *buffer, size_t size, float score, float threshold) {
    const int bar_width = 20;
    int filled = (int)(score * bar_width);
    int threshold_pos = (int)(threshold * bar_width);
    int percent = (int)(score * 100);
    
    // Clamp values
    if (filled < 0) filled = 0;
    if (filled > bar_width) filled = bar_width;
    if (threshold_pos < 0) threshold_pos = 0;
    if (threshold_pos > bar_width) threshold_pos = bar_width;
    
    // Build bar directly in output buffer
    int pos = 0;
    pos += snprintf(buffer + pos, size - pos, "[");
    
    for (int i = 0; i < bar_width; i++) {
        if (i == threshold_pos) {
            pos += snprintf(buffer + pos, size - pos, "|");  // Threshold marker
        } else if (i < filled) {
            pos += snprintf(buffer + pos, size - pos, "█");  // Filled block (UTF-8)
        } else {
            pos += snprintf(buffer + pos, size - pos, "░");  // Empty block (UTF-8)
        }
    }
    
    snprintf(buffer + pos, size - pos, "] %d%%", percent);
}

static void csi_callback(void *ctx __attribute__((unused)), wifi_csi_info_t *data) {
    int8_t *csi_data = data->buf;
    size_t csi_len = data->len;
    
    if (csi_len < 10) {
        return;
    }
    
    // Capture CSI data during calibration for debugging
    if (calibration_is_active()) {
        calibration_capture_csi(csi_data, csi_len);
    }
    
    // Protect g_state modifications with mutex (50ms timeout)
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_state.packets_received++;
        
        // Extract features from CSI data
        csi_extract_features(csi_data, csi_len, &g_state.current_features);
        
        // Feed features to calibration system if active
        static uint32_t calib_update_count = 0;
        
        if (calibration_is_active()) {
            calib_update_count++;
            if (calib_update_count % 100 == 1) {
                ESP_LOGI(TAG, "🔬 Calibration active: %u updates sent", calib_update_count);
            }
            
            feature_array_t feat_array;
            feat_array.features[0] = g_state.current_features.variance;
            feat_array.features[1] = g_state.current_features.skewness;
            feat_array.features[2] = g_state.current_features.kurtosis;
            feat_array.features[3] = g_state.current_features.entropy;
            feat_array.features[4] = g_state.current_features.iqr;
            feat_array.features[5] = g_state.current_features.spatial_variance;
            feat_array.features[6] = g_state.current_features.spatial_correlation;
            feat_array.features[7] = g_state.current_features.spatial_gradient;
            
            calibration_update(&feat_array);
        }
        
        // Calculate detection score
        detection_config_t det_config = {
            .threshold_high = g_state.threshold_high,
            .threshold_low = g_state.threshold_low,
            .debounce_count = g_state.config.debounce_count,
            .persistence_timeout = g_state.config.persistence_timeout,
            .feature_weights = g_state.config.feature_weights  // Pass pointer to array
        };
        
        // Calculate detection score - use calibrated method if available
        float detection_score;
        uint8_t num_selected = calibration_get_num_selected();
        
        if (num_selected > 0) {
            // Use calibrated features and weights
            const uint8_t *selected_features = calibration_get_selected_features();
            const float *weights = calibration_get_weights();
            detection_score = detection_calculate_score_calibrated(&g_state.current_features,
                                                                   selected_features,
                                                                   weights,
                                                                   num_selected);
        } else {
            // Fallback to default scoring
            detection_score = detection_calculate_score(&g_state.current_features, &det_config);
        }
        
        // Add to stats buffer
        stats_buffer_add(&g_state.stats_buffer, detection_score);
        
        // Update detection state
        detection_engine_state_t engine_state = {
            .current_state = g_state.state,
            .consecutive_detections = g_state.consecutive_detections,
            .last_detection_time = g_state.last_detection_time,
            .last_score = detection_score,
            .confidence = g_state.confidence
        };
        
        detection_update_state(&engine_state, detection_score, &det_config, get_timestamp_sec());
        
        // Update g_state from engine_state
        g_state.state = engine_state.current_state;
        g_state.consecutive_detections = engine_state.consecutive_detections;
        g_state.last_detection_time = engine_state.last_detection_time;
        g_state.detection_score = detection_score;
        g_state.confidence = engine_state.confidence;
        
        g_state.packets_processed++;
        
        xSemaphoreGive(g_state_mutex);
    } else {
        // Safely increment dropped packet counter
        uint32_t dropped = atomic_fetch_add(&g_state.packets_dropped, 1) + 1;
        
        if ((dropped % 100 == 0) || (dropped == 1)) {
            ESP_LOGW(TAG, "CSI callback: %lu packets dropped due to mutex timeout", 
                     (unsigned long)dropped);
        }
    }
}

static void csi_init(void) {
#if CONFIG_IDF_TARGET_ESP32C6
    // ESP32-C6 uses simplified wifi_csi_acquire_config_t structure
    // It doesn't support the advanced LTF configuration fields
    wifi_csi_acquire_config_t csi_config = {
        .enable = 1,
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_LOGI(TAG, "CSI initialized and enabled (ESP32-C6 simplified mode)");
#else
    // ESP32 and ESP32-S3 use wifi_csi_config_t with advanced LTF fields
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = false,
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_LOGI(TAG, "CSI initialized and enabled (ESP32/ESP32-S3 advanced mode)");
#endif
    
    // Common CSI setup for all targets
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

static void mqtt_publish_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t publish_period = pdMS_TO_TICKS((uint32_t)(PUBLISH_INTERVAL * 1000));
    
    int64_t last_csi_log_time = 0;
    
    // Smart publishing configuration
    mqtt_publish_config_t pub_config = {
        .enabled = g_state.config.smart_publishing_enabled,
        .delta_threshold = 0.05f,
        .max_interval_sec = 5.0f,
        .confidence_threshold = CONFIDENCE_THRESHOLD
    };
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, publish_period);
        
        // Read g_state values with mutex protection
        float detection_score, threshold_high, confidence;
        detection_state_t state;
        uint32_t packets_received, packets_processed;
        
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            detection_score = g_state.detection_score;
            threshold_high = g_state.threshold_high;
            confidence = g_state.confidence;
            state = g_state.state;
            packets_received = g_state.packets_received;
            packets_processed = g_state.packets_processed;
            xSemaphoreGive(g_state_mutex);
        } else {
            ESP_LOGW(TAG, "MQTT publish task: Failed to acquire mutex, skipping publish cycle");
            continue;
        }
        
        // Check calibration and advance phases
        static calibration_phase_t last_phase = CALIB_IDLE;
        calibration_phase_t current_phase = calibration_get_phase();
        
        // Publish calibration status on phase change
        if (current_phase != last_phase) {
            uint32_t samples = calibration_get_samples_collected();
            
            // Get phase target samples and traffic rate from calibration system
            calibration_state_t calib_state;
            calibration_get_results(&calib_state);
            uint32_t phase_target = calib_state.phase_target_samples;
            uint32_t traffic_rate = calib_state.traffic_rate;
            
            mqtt_publish_calibration_status(&g_state.mqtt_state, 
                                          (uint8_t)current_phase,
                                          phase_target,
                                          samples,
                                          traffic_rate,
                                          g_response_topic);
        }
        
        // Detect calibration completion (when phase becomes ANALYZING)
        if (current_phase == CALIB_ANALYZING && last_phase != CALIB_ANALYZING) {
            ESP_LOGI(TAG, "🎉 Calibration analysis complete, applying results...");
            
            if (calibration_get_num_selected() > 0) {
                float optimal_threshold = calibration_get_threshold();
                
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    float old_threshold = g_state.threshold_high;
                    g_state.threshold_high = optimal_threshold;
                    g_state.threshold_low = optimal_threshold * g_state.config.hysteresis_ratio;
                    xSemaphoreGive(g_state_mutex);
                    
                    ESP_LOGI(TAG, "🎯 Applied calibration results: threshold %.4f -> %.4f",
                             old_threshold, optimal_threshold);
                }
                
                // Apply optimized weights (protected by mutex for thread-safety)
                const float *weights = calibration_get_weights();
                const uint8_t *features = calibration_get_selected_features();
                uint8_t num_selected = calibration_get_num_selected();
                
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Reset all weights to 0
                    memset(g_state.config.feature_weights, 0, sizeof(g_state.config.feature_weights));
                    
                    // Map ALL calibrated features to the weights array
                    for (uint8_t i = 0; i < num_selected; i++) {
                        uint8_t feat_idx = features[i];
                        float weight = weights[i];
                        
                        // Map feature index directly to array (supports all 8 features)
                        if (feat_idx < 8) {
                            g_state.config.feature_weights[feat_idx] = weight;
                        } else {
                            ESP_LOGW(TAG, "Invalid feature index %d (max 7)", feat_idx);
                        }
                    }
                    
                    xSemaphoreGive(g_state_mutex);
                    
                    ESP_LOGI(TAG, "✅ Calibration applied: all %d feature weights updated", num_selected);
                } else {
                    ESP_LOGW(TAG, "Failed to acquire mutex for weight update");
                }
                
                // Save to NVS
                calibration_state_t calib_state;
                calibration_get_results(&calib_state);
                nvs_calibration_data_t nvs_calib;
                nvs_calib.version = NVS_CALIBRATION_VERSION;
                nvs_calib.num_selected = calib_state.num_selected;
                memcpy(nvs_calib.selected_features, calib_state.selected_features, 
                       sizeof(nvs_calib.selected_features));
                memcpy(nvs_calib.optimized_weights, calib_state.optimized_weights, 
                       sizeof(nvs_calib.optimized_weights));
                nvs_calib.optimal_threshold = calib_state.optimal_threshold;
                memcpy(nvs_calib.feature_min, calib_state.feature_min,
                       sizeof(nvs_calib.feature_min));
                memcpy(nvs_calib.feature_max, calib_state.feature_max,
                       sizeof(nvs_calib.feature_max));
                
                esp_err_t calib_err = nvs_save_calibration(&nvs_calib);
                if (calib_err == ESP_OK) {
                    ESP_LOGI(TAG, "💾 Calibration data saved to NVS");
                } else {
                    ESP_LOGE(TAG, "❌ Failed to save calibration to NVS: %s", esp_err_to_name(calib_err));
                }
                
                esp_err_t config_err = config_save_to_nvs(&g_state.config, g_state.threshold_high, g_state.threshold_low);
                if (config_err == ESP_OK) {
                    ESP_LOGI(TAG, "💾 Configuration saved to NVS");
                } else {
                    ESP_LOGE(TAG, "❌ Failed to save configuration to NVS: %s", esp_err_to_name(config_err));
                }
                
                // Acknowledge completion to reset calibration phase to IDLE
                calibration_acknowledge_completion();
            }
        }
        
        // Detect when calibration fully completes (ANALYZING → IDLE)
        if (current_phase == CALIB_IDLE && last_phase == CALIB_ANALYZING) {
            // Send calibration complete recap
            calibration_state_t calib_state;
            calibration_get_results(&calib_state);
            mqtt_publish_calibration_complete(&g_state.mqtt_state, &calib_state, g_response_topic);
        }
        
        last_phase = current_phase;
        calibration_check_completion();
        
        // Adaptive normalizer auto-reset logic
        if (g_state.config.adaptive_normalizer_enabled && 
            g_state.config.adaptive_normalizer_reset_timeout_sec > 0) {
            
            int64_t current_time = get_timestamp_sec();
            
            // Update last movement time
            if (state != STATE_IDLE) {
                g_last_movement_time = current_time;
            }
            
            // Check if we should reset the normalizer
            if (state == STATE_IDLE && g_last_movement_time > 0) {
                int64_t idle_duration = current_time - g_last_movement_time;
                
                if (idle_duration >= (int64_t)g_state.config.adaptive_normalizer_reset_timeout_sec) {
                    // Reset the normalizer
                    adaptive_normalizer_init(&g_normalizer, g_state.config.adaptive_normalizer_alpha);
                    g_normalizer_reset_count++;
                    g_last_movement_time = current_time;
                    
                    ESP_LOGI(TAG, "🔄 Adaptive normalizer reset after %lld sec of IDLE (reset #%u)",
                             (long long)idle_duration, (unsigned int)g_normalizer_reset_count);
                }
            }
        }
        
        // CSI logging with progress bar (skip during calibration to avoid confusion)
        int64_t now = get_timestamp_sec();
        if (g_state.config.csi_logs_enabled && 
            !calibration_is_active() && 
            (now - last_csi_log_time >= LOG_CSI_VALUES_INTERVAL)) {
            const char *state_names[] = {"IDLE", "DETECTED"};
            const char *state_str = (state < 2) ? state_names[state] : "UNKNOWN";
            
            // Format progress bar with threshold marker (larger buffer to avoid truncation)
            char progress_bar[256];
            format_progress_bar(progress_bar, sizeof(progress_bar), detection_score, threshold_high);
            
            // Calculate packet delta (packets processed in last second)
            static uint32_t last_packets_processed = 0;
            uint32_t packet_delta = packets_processed - last_packets_processed;
            last_packets_processed = packets_processed;
            
            ESP_LOGI(TAG, "📊 %s | pkts:%lu mvmt:%.4f thr:%.4f | %s",
                     progress_bar, (unsigned long)packet_delta, 
                     detection_score, threshold_high, state_str);
            last_csi_log_time = now;
        }
        
        // Smart publishing
        if (confidence >= pub_config.confidence_threshold) {
            int64_t current_time = get_timestamp_ms();
            
            if (mqtt_should_publish(&g_state.mqtt_state, detection_score, state, 
                                   &pub_config, current_time)) {
                // Prepare detection result
                detection_result_t result = {
                    .score = detection_score,
                    .confidence = confidence,
                    .state = state,
                    .timestamp = get_timestamp_sec()
                };
                
                mqtt_publish_detection(&g_state.mqtt_state, &result, MQTT_TOPIC);
                mqtt_update_publish_state(&g_state.mqtt_state, detection_score, state, current_time);
            }
        }
        
        // Statistics logging
        if (packets_received > 0 && packets_received % STATS_LOG_INTERVAL == 0) {
            float success_rate = ((float)packets_processed / packets_received) * 100.0f;
            uint32_t packets_dropped_stat = atomic_load(&g_state.packets_dropped);
            ESP_LOGD(TAG, "Stats: %lu packets received, %lu processed (%.1f%% success), %lu dropped",
                     (unsigned long)packets_received, (unsigned long)packets_processed, 
                     success_rate, (unsigned long)packets_dropped_stat);
            
            // Only log smart publishing stats if the feature is enabled
            if (g_state.config.smart_publishing_enabled) {
                uint32_t published, skipped;
                mqtt_get_publish_stats(&g_state.mqtt_state, &published, &skipped);
                if (published + skipped > 0) {
                    float reduction = (skipped * 100.0f) / (published + skipped);
                    ESP_LOGD(TAG, "Smart Publishing: %lu published, %lu skipped (%.1f%% reduction)",
                             (unsigned long)published, (unsigned long)skipped, reduction);
                }
            }
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "System starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create mutex for g_state protection
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create g_state mutex");
        return;
    }
    
    memset(&g_state, 0, sizeof(g_state));
    
    // Initialize configuration with defaults
    config_init_defaults(&g_state.config);
    
    // Initialize statistics buffer
    if (stats_buffer_init(&g_state.stats_buffer, STATS_BUFFER_SIZE) != 0) {
        ESP_LOGE(TAG, "Failed to initialize statistics buffer");
        vSemaphoreDelete(g_state_mutex);
        return;
    }
    
    // Set default thresholds
    g_state.threshold_high = DEFAULT_THRESHOLD;
    g_state.threshold_low = DEFAULT_THRESHOLD * g_state.config.hysteresis_ratio;
    
    // Initialize filter modules
    filter_buffer_init(&g_filter_buffer);
    adaptive_normalizer_init(&g_normalizer, g_state.config.adaptive_normalizer_alpha);
    
    // Initialize adaptive normalizer reset tracking
    g_last_movement_time = get_timestamp_sec();
    g_normalizer_reset_count = 0;
    
    // Initialize calibration system
    calibration_init();
    
    // Initialize NVS storage
    nvs_storage_init();
    
    // Load saved configuration if exists
    if (config_exists_in_nvs()) {
        nvs_config_data_t nvs_cfg;
        if (nvs_load_control_params(&nvs_cfg) == ESP_OK) {
            // Load config parameters (pass nvs_cfg to avoid duplicate NVS read)
            config_load_from_nvs(&g_state.config, &nvs_cfg);
            
            // Load thresholds from NVS
            g_state.threshold_high = nvs_cfg.threshold_high;
            g_state.threshold_low = nvs_cfg.threshold_low;
            
            ESP_LOGI(TAG, "💾 Loaded saved configuration from NVS");
            ESP_LOGI(TAG, "🎯 Loaded thresholds: high=%.4f, low=%.4f",
                     g_state.threshold_high, g_state.threshold_low);
        }
    }
    
    // Load saved calibration if exists
    if (nvs_has_calibration()) {
        ESP_LOGI(TAG, "🔍 Calibration data found in NVS, loading...");
        nvs_calibration_data_t nvs_calib;
        if (nvs_load_calibration(&nvs_calib) == ESP_OK) {
            ESP_LOGI(TAG, "📥 NVS calibration loaded: %d features, threshold=%.4f",
                     nvs_calib.num_selected, nvs_calib.optimal_threshold);
            
            calibration_state_t calib_state = {0};
            calib_state.num_selected = nvs_calib.num_selected;
            calib_state.optimal_threshold = nvs_calib.optimal_threshold;
            memcpy(calib_state.selected_features, nvs_calib.selected_features, 
                   sizeof(calib_state.selected_features));
            memcpy(calib_state.optimized_weights, nvs_calib.optimized_weights, 
                   sizeof(calib_state.optimized_weights));
            memcpy(calib_state.feature_min, nvs_calib.feature_min,
                   sizeof(calib_state.feature_min));
            memcpy(calib_state.feature_max, nvs_calib.feature_max,
                   sizeof(calib_state.feature_max));
            
            // Apply to calibration system
            calibration_apply_saved(&calib_state);
            
            // Update thresholds
            g_state.threshold_high = nvs_calib.optimal_threshold;
            g_state.threshold_low = nvs_calib.optimal_threshold * g_state.config.hysteresis_ratio;
            ESP_LOGI(TAG, "🎯 Thresholds updated: high=%.4f, low=%.4f",
                     g_state.threshold_high, g_state.threshold_low);
            
            // Apply loaded weights to config
            const uint8_t *features = calib_state.selected_features;
            const float *weights = calib_state.optimized_weights;
            
            // Feature names mapping (only implemented features)
            const char *feature_names[] = {
                "variance", "skewness", "kurtosis", "entropy", "iqr",
                "spatial_variance", "spatial_correlation", "spatial_gradient"
            };
            
            // Reset all weights to 0
            memset(g_state.config.feature_weights, 0, sizeof(g_state.config.feature_weights));
            
            // Apply calibrated weights to array (supports ALL 8 features)
            ESP_LOGI(TAG, "⚖️  Applying calibrated weights:");
            for (uint8_t i = 0; i < calib_state.num_selected; i++) {
                uint8_t feat_idx = features[i];
                float weight = weights[i];
                
                const char *feat_name = (feat_idx < 8) ? feature_names[feat_idx] : "invalid";
                
                // Map feature index directly to array (supports all 8 features)
                if (feat_idx < 8) {
                    g_state.config.feature_weights[feat_idx] = weight;
                    ESP_LOGI(TAG, "  Feature[%d] %s: weight=%.4f", feat_idx, feat_name, weight);
                } else {
                    ESP_LOGW(TAG, "  Invalid feature index: %d", feat_idx);
                }
            }
            
            ESP_LOGI(TAG, "✅ All %d calibrated feature weights applied to array", calib_state.num_selected);
            
            ESP_LOGI(TAG, "💾 Saved calibration successfully applied");
        } else {
            ESP_LOGE(TAG, "❌ Failed to load calibration from NVS");
        }
    } else {
        ESP_LOGI(TAG, "ℹ️  No saved calibration found in NVS, using defaults");
    }
    
    // Initialize WiFi
    wifi_credentials_t wifi_creds = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD
    };
    
    if (wifi_manager_init(&g_state.wifi_state, &wifi_creds) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi");
        return;
    }
    
    wifi_manager_wait_connected();
    
    // Initialize MQTT
    mqtt_config_t mqtt_cfg = {
        .broker_uri = MQTT_BROKER_URI,
        .username = MQTT_USERNAME,
        .password = MQTT_PASSWORD,
        .base_topic = MQTT_TOPIC,
        .cmd_topic = MQTT_TOPIC "/cmd",
        .response_topic = MQTT_TOPIC "/response"
    };
    
    // Store response topic globally for use in mqtt_publish_task
    g_response_topic = mqtt_cfg.response_topic;
    
    if (mqtt_handler_init(&g_state.mqtt_state, &mqtt_cfg) != 0) {
        ESP_LOGE(TAG, "Failed to initialize MQTT handler");
        return;
    }
    
    if (mqtt_handler_start(&g_state.mqtt_state) != 0) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        return;
    }
    
    // Setup MQTT command context
    g_mqtt_cmd_context.config = &g_state.config;
    g_mqtt_cmd_context.threshold_high = &g_state.threshold_high;
    g_mqtt_cmd_context.threshold_low = &g_state.threshold_low;
    g_mqtt_cmd_context.stats_buffer = &g_state.stats_buffer;
    g_mqtt_cmd_context.current_features = &g_state.current_features;
    g_mqtt_cmd_context.current_state = &g_state.state;
    g_mqtt_cmd_context.butterworth = &g_butterworth;
    g_mqtt_cmd_context.filter_buffer = &g_filter_buffer;
    g_mqtt_cmd_context.normalizer = &g_normalizer;
    g_mqtt_cmd_context.mqtt_base_topic = mqtt_cfg.base_topic;
    g_mqtt_cmd_context.mqtt_cmd_topic = mqtt_cfg.cmd_topic;
    g_mqtt_cmd_context.mqtt_response_topic = mqtt_cfg.response_topic;
    
    // Initialize MQTT commands
    if (mqtt_commands_init(&g_state.mqtt_state, &g_mqtt_cmd_context) != 0) {
        ESP_LOGE(TAG, "Failed to initialize MQTT commands");
        return;
    }
    
    // Set command callback
    mqtt_handler_set_command_callback(mqtt_command_callback);
    
    ESP_LOGI(TAG, "MQTT client started with command support");
    
    // Initialize traffic generator
    traffic_generator_init();
    
    // Start traffic generator with configured rate (if > 0)
    // Can be changed later via MQTT command
    if (g_state.config.traffic_generator_rate > 0) {
        if (traffic_generator_start(g_state.config.traffic_generator_rate)) {
            ESP_LOGI(TAG, "✅ Traffic generator started (%u pps)", 
                     (unsigned int)g_state.config.traffic_generator_rate);
        } else {
            ESP_LOGW(TAG, "⚠️  Failed to start traffic generator");
        }
    }
    
    // Initialize CSI
    csi_init();
    
    // Start MQTT publish task
    xTaskCreate(mqtt_publish_task, "mqtt_pub", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                   🛜  E S P e c t r e 👻                   ║");
    ESP_LOGI(TAG, "║                                                           ║");
    ESP_LOGI(TAG, "║                Wi-Fi motion detection system              ║");
    ESP_LOGI(TAG, "║          based on Channel State Information (CSI)         ║");
    ESP_LOGI(TAG, "║                                                           ║");
    ESP_LOGI(TAG, "╠═══════════════════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║                                                           ║");
    ESP_LOGI(TAG, "║             System Ready - Monitoring Active              ║");
    ESP_LOGI(TAG, "║               Detecting the invisible... 👁️                ║");
    ESP_LOGI(TAG, "║                                                           ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
}
