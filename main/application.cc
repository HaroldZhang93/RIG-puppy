#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "assets/lang_config.h"
#include "mcp_server.h"

#include <cstring>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>

#include <esp_flash.h>
#include "xgo.h"
#define TAG "Application"


static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "audio_testing",
    "fatal_error",
    "invalid_state"
};

Application::Application() {
    event_group_ = xEventGroupCreate();

#if CONFIG_USE_DEVICE_AEC && CONFIG_USE_SERVER_AEC
#error "CONFIG_USE_DEVICE_AEC and CONFIG_USE_SERVER_AEC cannot be enabled at the same time"
#elif CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void* arg) {
            Application* app = (Application*)arg;
            app->OnClockTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    if (xgo_task_handle_ != nullptr) {
        vTaskDelete(xgo_task_handle_);
    }
    if (xgo_rx_task_handle_ != nullptr) {
        vTaskDelete(xgo_rx_task_handle_);
    }
    if (debug_cmd_task_handle_ != nullptr) {
        vTaskDelete(debug_cmd_task_handle_);
    }
    vEventGroupDelete(event_group_);
}

void Application::CheckNewVersion(Ota& ota) {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int retry_delay = 10; // 初始重试延迟为10秒

    auto& board = Board::GetInstance();
    while (true) {
        SetDeviceState(kDeviceStateActivating);
        auto display = board.GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        if (!ota.CheckVersion()) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char buffer[256];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay, ota.GetCheckVersionUrl().c_str());
            Alert(Lang::Strings::ERROR, buffer, "sad", Lang::Sounds::OGG_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay, retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (device_state_ == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2; // 每次重试后延迟时间翻倍
            continue;
        }
        retry_count = 0;
        retry_delay = 10; // 重置重试延迟时间

        if (ota.HasNewVersion()) {
            Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "happy", Lang::Sounds::OGG_UPGRADE);
            vTaskDelay(pdMS_TO_TICKS(3000));
            SetDeviceState(kDeviceStateUpgrading);
            
            display->SetIcon(FONT_AWESOME_DOWNLOAD);
            std::string message = std::string(Lang::Strings::NEW_VERSION) + ota.GetFirmwareVersion();
            display->SetChatMessage("system", message.c_str());

            board.SetPowerSaveMode(false);
            audio_service_.Stop();
            vTaskDelay(pdMS_TO_TICKS(1000));

            bool upgrade_success = ota.StartUpgrade([display](int progress, size_t speed) {
                std::thread([display, progress, speed]() {
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                    display->SetChatMessage("system", buffer);
                }).detach();
            });

            if (!upgrade_success) {
                // Upgrade failed, restart audio service and continue running
                ESP_LOGE(TAG, "Firmware upgrade failed, restarting audio service and continuing operation...");
                audio_service_.Start(); // Restart audio service
                board.SetPowerSaveMode(true); // Restore power save mode
                Alert(Lang::Strings::ERROR, Lang::Strings::UPGRADE_FAILED, "sad", Lang::Sounds::OGG_EXCLAMATION);
                vTaskDelay(pdMS_TO_TICKS(3000));
                // Continue to normal operation (don't break, just fall through)
            } else {
                // Upgrade success, reboot immediately
                ESP_LOGI(TAG, "Firmware upgrade successful, rebooting...");
                display->SetChatMessage("system", "Upgrade successful, rebooting...");
                vTaskDelay(pdMS_TO_TICKS(1000)); // Brief pause to show message
                Reboot();
                return; // This line will never be reached after reboot
            }
        }

        // No new version, mark the current version as valid
        ota.MarkCurrentVersionValid();
        if (!ota.HasActivationCode() && !ota.HasActivationChallenge()) {
            xEventGroupSetBits(event_group_, MAIN_EVENT_CHECK_NEW_VERSION_DONE);
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Activation code is shown to the user and waiting for the user to input
        if (ota.HasActivationCode()) {
            ShowActivationCode(ota.GetActivationCode(), ota.GetActivationMessage());
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota.Activate();
            if (err == ESP_OK) {
                xEventGroupSetBits(event_group_, MAIN_EVENT_CHECK_NEW_VERSION_DONE);
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (device_state_ == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::ShowActivationCode(const std::string& code, const std::string& message) {
    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{{
        digit_sound{'0', Lang::Sounds::OGG_0},
        digit_sound{'1', Lang::Sounds::OGG_1}, 
        digit_sound{'2', Lang::Sounds::OGG_2},
        digit_sound{'3', Lang::Sounds::OGG_3},
        digit_sound{'4', Lang::Sounds::OGG_4},
        digit_sound{'5', Lang::Sounds::OGG_5},
        digit_sound{'6', Lang::Sounds::OGG_6},
        digit_sound{'7', Lang::Sounds::OGG_7},
        digit_sound{'8', Lang::Sounds::OGG_8},
        digit_sound{'9', Lang::Sounds::OGG_9}
    }};

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish

    std::string access = Lang::Strings::ACCESS_VIA_BROWSER;
    if (!access.empty() && access[0] == ',') {
        access.erase(0, 1);  // 删除开头逗号
    }
    std::string full_message = access + message;
    Alert(Lang::Strings::ACTIVATION, full_message.c_str(), "", Lang::Sounds::OGG_ACTIVATION);

    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
            [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            audio_service_.PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char* status, const char* message, const char* emotion, const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert %s: %s [%s]", status, message, emotion);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        audio_service_.PlaySound(sound);
    }
}

void Application::DismissAlert() {
    if (device_state_ == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::ToggleChatState() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    } else if (device_state_ == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        Schedule([this]() {
            protocol_->CloseAudioChannel();
        });
    }
}

void Application::StartListening() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }
    
    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(kListeningModeManualStop);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
            SetListeningMode(kListeningModeManualStop);
        });
    }
}

void Application::StopListening() {
    if (device_state_ == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    const std::array<int, 3> valid_states = {
        kDeviceStateListening,
        kDeviceStateSpeaking,
        kDeviceStateIdle,
    };
    // If not valid, do nothing
    if (std::find(valid_states.begin(), valid_states.end(), device_state_) == valid_states.end()) {
        return;
    }

    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

void Application::Start() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    /* Setup the display */
    auto display = board.GetDisplay();

    /* Setup the audio service */
    auto codec = board.GetAudioCodec();
    audio_service_.Initialize(codec);
    audio_service_.Start();

    AudioServiceCallbacks callbacks;
    callbacks.on_send_queue_available = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_SEND_AUDIO);
    };
    callbacks.on_wake_word_detected = [this](const std::string& wake_word) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_WAKE_WORD_DETECTED);
    };
    callbacks.on_vad_change = [this](bool speaking) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_VAD_CHANGE);
    };
    audio_service_.SetCallbacks(callbacks);

    esp_timer_start_periodic(clock_timer_handle_, 1000000);

#ifdef CONFIG_BOARD_TYPE_LULU_ESP32S3
    xTaskCreatePinnedToCore([](void* arg) {
        Application* app = (Application*)arg;
        app->XgoTask();
        vTaskDelete(NULL);
    }, "xgo_task", 4096, this, 4, &xgo_task_handle_, 0);

    xTaskCreatePinnedToCore([](void* arg) {
        Application* app = (Application*)arg;
        app->XgoRxTask();
        vTaskDelete(NULL);
    }, "xgo_rx_task", 4096, this, 2, &xgo_rx_task_handle_, 1);
#endif
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->DebugCommandTask();
        vTaskDelete(NULL);
    }, "dbg_cmd_task", 6144, this, 2, &debug_cmd_task_handle_);

    /* Wait for the network to be ready */
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);

    // Check for new firmware version or get the MQTT broker address
    Ota ota;
    CheckNewVersion(ota);

    // Initialize the protocol
    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    // Add MCP common tools before initializing the protocol
    McpServer::GetInstance().AddCommonTools();

    if (ota.HasMqttConfig()) {
        protocol_ = std::make_unique<MqttProtocol>();
    } else if (ota.HasWebsocketConfig()) {
        protocol_ = std::make_unique<WebsocketProtocol>();
    } else {
        ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    }

    protocol_->OnNetworkError([this](const std::string& message) {
        last_error_message_ = message;
        xEventGroupSetBits(event_group_, MAIN_EVENT_ERROR);
    });
    protocol_->OnIncomingAudio([this](std::unique_ptr<AudioStreamPacket> packet) {
        if (device_state_ == kDeviceStateSpeaking) {
            audio_service_.PushPacketToDecodeQueue(std::move(packet));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "Server sample rate %d does not match device output sample rate %d, resampling may cause distortion",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                        SetDeviceState(kDeviceStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (device_state_ == kDeviceStateSpeaking) {
                        if (listening_mode_ == kListeningModeManualStop) {
                            SetDeviceState(kDeviceStateIdle);
                        } else {
                            SetDeviceState(kDeviceStateListening);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([this, display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "mcp") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                McpServer::GetInstance().ParseMessage(payload);
            }
        } else if (strcmp(type->valuestring, "system") == 0) {
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    // Do a reboot if user requests a OTA update
                    Schedule([this]() {
                        Reboot();
                    });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring, Lang::Sounds::OGG_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
#if CONFIG_RECEIVE_CUSTOM_MESSAGE
        } else if (strcmp(type->valuestring, "custom") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            ESP_LOGI(TAG, "Received custom message: %s", cJSON_PrintUnformatted(root));
            if (cJSON_IsObject(payload)) {
                Schedule([this, display, payload_str = std::string(cJSON_PrintUnformatted(payload))]() {
                    display->SetChatMessage("system", payload_str.c_str());
                });
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
#endif
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });
    bool protocol_started = protocol_->Start();

    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota.HasServerTime();
    if (protocol_started) {
        std::string message = std::string(Lang::Strings::VERSION) + ota.GetCurrentVersion();
        display->ShowNotification(message.c_str());
        display->SetChatMessage("system", "");
        // Play the success sound to indicate the device is ready
        audio_service_.PlaySound(Lang::Sounds::OGG_SUCCESS);
    }

    // Print heap stats
    SystemInfo::PrintHeapStats();
}

void Application::OnClockTimer() {
    clock_ticks_++;

    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar();

    // Print the debug info every 10 seconds
    if (clock_ticks_ % 10 == 0) {
        // SystemInfo::PrintTaskCpuUsage(pdMS_TO_TICKS(1000));
        // SystemInfo::PrintTaskList();
        SystemInfo::PrintHeapStats();
    }
}

void Application::DebugCommandTask() {
    constexpr int kMaxLineLength = 128;
    std::string line;
    line.reserve(kMaxLineLength);

    printf("\r\n[DBG] 调试命令已启动，输入 help 查看命令\r\n");
    printf("dbg> ");
    fflush(stdout);

    while (true) {
        int ch = getchar();
        if (ch < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            printf("\r\n");
            if (!line.empty()) {
                HandleDebugCommand(line);
                line.clear();
            }
            printf("dbg> ");
            fflush(stdout);
            continue;
        }

        if (ch == '\b' || ch == 127) {
            if (!line.empty()) {
                line.pop_back();
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if (!std::isprint(static_cast<unsigned char>(ch))) {
            continue;
        }

        if (line.size() >= kMaxLineLength) {
            continue;
        }

        line.push_back(static_cast<char>(ch));
        putchar(ch); // 回显键盘输入
        fflush(stdout);
    }
}

void Application::HandleDebugCommand(const std::string& line) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd.empty()) {
        return;
    }

    std::string arg;
    std::getline(iss, arg);
    auto trim_left = [](std::string& s) {
        size_t i = 0;
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
            ++i;
        }
        s.erase(0, i);
    };
    trim_left(arg);

    auto call_mcp_tool = [](int id, const std::string& name, const std::string& arguments_json) {
        std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":";
        payload += std::to_string(id);
        payload += ",\"method\":\"tools/call\",\"params\":{\"name\":\"";
        payload += name;
        payload += "\",\"arguments\":";
        payload += arguments_json;
        payload += "}}";
        McpServer::GetInstance().ParseMessage(payload);
    };

    if (cmd == "help") {
        printf("[DBG] 可用命令:\r\n");
        printf("  help                   - 显示帮助\r\n");
        printf("  test <n>               - 触发测试命令\r\n");
        printf("  move <vx,vyaw,time>    - 机器狗移动，如 move 20,0,1000\r\n");
        printf("  gaitdbg <0|1>          - 开关步态调试日志\r\n");
        printf("  gaitstate              - 打印当前步态参数快照\r\n");
        printf("  action <name>          - 触发动作，如 action wave\r\n");
        printf("  loop <0|1>      - 动作循环: 1开始 0停止\r\n");
        printf("  laser <0|1|2>          - 激光控制: 0关 1开 2切换\r\n");
        printf("  calibrate <0|1>        - 舵机标定: 1进入 0退出并保存\r\n");
        printf("  read_zeropos           - 读取零位(通过 MCP tool)\r\n");
        printf("  state                  - 打印当前设备状态\r\n");
        return;
    }

    if (cmd == "state") {
        printf("[DBG] state=%s\r\n", STATE_STRINGS[device_state_]);
        return;
    }

    if (cmd == "read_zeropos") {
        call_mcp_tool(201, "self.dog.read_zeropos", "{}");
        printf("[DBG] read_zeropos 已触发\r\n");
        return;
    }

    if (cmd == "action") {
        if (arg.empty()) {
            printf("[DBG] 用法: action <name>\r\n");
            printf("[DBG] 可选: wave naughty swing lookup rolling angry swimming pee stretch bouncing shaking sit scratch hug\r\n");
            return;
        }

        // 转小写，方便输入
        std::string action_name = arg;
        for (char& c : action_name) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }

        std::string tool_name;
        if (action_name == "wave") tool_name = "self.dog.Wave";
        else if (action_name == "naughty") tool_name = "self.dog.Naughty";
        else if (action_name == "swing") tool_name = "self.dog.Swing";
        else if (action_name == "lookup") tool_name = "self.dog.Lookup";
        else if (action_name == "rolling") tool_name = "self.dog.Rolling";
        else if (action_name == "angry") tool_name = "self.dog.Angry";
        else if (action_name == "swimming") tool_name = "self.dog.Swimming";
        else if (action_name == "pee") tool_name = "self.dog.Pee";
        else if (action_name == "stretch") tool_name = "self.dog.Stretch";
        else if (action_name == "bouncing") tool_name = "self.dog.Bouncing";
        else if (action_name == "shaking") tool_name = "self.dog.Shaking";
        else if (action_name == "sit") tool_name = "self.dog.Sit";
        else if (action_name == "scratch") tool_name = "self.dog.Scratch";
        else if (action_name == "hug") tool_name = "self.dog.Hug";
        else {
            printf("[DBG] 未知动作: %s\r\n", arg.c_str());
            return;
        }

        call_mcp_tool(203, tool_name, "{}");
        printf("[DBG] action %s 已触发\r\n", arg.c_str());
        return;
    }

    auto parse_int = [](const std::string& s, int& out) -> bool {
        if (s.empty()) {
            return false;
        }
        char* end_ptr = nullptr;
        long v = std::strtol(s.c_str(), &end_ptr, 10);
        if (end_ptr == s.c_str() || *end_ptr != '\0') {
            return false;
        }
        out = static_cast<int>(v);
        return true;
    };

    int value = 0;

    if (cmd == "test") {
        if (!parse_int(arg, value)) {
            printf("[DBG] 参数错误，格式应为: test <int>\r\n");
            return;
        }
        Schedule([value]() {
            ESP_LOGI(TAG, "[DBG] test value=%d", value);
        });
        printf("[DBG] test %d 已触发\r\n", value);
        return;
    }

    if (cmd == "move") {
        if (arg.empty()) {
            printf("[DBG] 用法: move <vx,vyaw,time>，例如 move 20,0,1000\r\n");
            return;
        }

        int vx = 0;
        int vyaw = 0;
        int time_ms = 0;
        char comma1 = 0;
        char comma2 = 0;
        std::istringstream move_ss(arg);
        move_ss >> vx >> comma1 >> vyaw >> comma2 >> time_ms;
        if (move_ss.fail() || comma1 != ',' || comma2 != ',') {
            printf("[DBG] 参数错误，用法: move <vx,vyaw,time>\r\n");
            return;
        }
        if (vx < -100 || vx > 100 || vyaw < -100 || vyaw > 100 || time_ms < 0 || time_ms > 10000) {
            printf("[DBG] 范围错误: vx/vyaw [-100,100], time [0,10000]\r\n");
            return;
        }

        std::string args = "{\"dog_vx\":";
        args += std::to_string(vx);
        args += ",\"dog_vyaw\":";
        args += std::to_string(vyaw);
        args += ",\"time\":";
        args += std::to_string(time_ms);
        args += "}";
        call_mcp_tool(204, "self.dog.move", args);
        printf("[DBG] move %d,%d,%d 已触发\r\n", vx, vyaw, time_ms);
        return;
    }

    if (cmd == "gaitdbg") {
        if (!parse_int(arg, value)) {
            printf("[DBG] 参数错误，格式应为: gaitdbg <0|1>\r\n");
            return;
        }
        if (value < 0 || value > 1) {
            printf("[DBG] gaitdbg 参数范围: 0~1\r\n");
            return;
        }

        SetGaitDebug(value == 1);
        PrintGaitDebugSnapshot();
        printf("[DBG] gaitdbg %d 已设置\r\n", value);
        return;
    }

    if (cmd == "gaitstate") {
        PrintGaitDebugSnapshot();
        return;
    }

    if (cmd == "laser") {
        if (!parse_int(arg, value)) {
            printf("[DBG] 参数错误，格式应为: laser <0|1|2>\r\n");
            return;
        }
        if (value < 0 || value > 2) {
            printf("[DBG] laser 参数范围: 0~2\r\n");
            return;
        }
        Schedule([value]() {
            Board::GetInstance().LaserControl(value);
        });
        printf("[DBG] laser %d 已触发\r\n", value);
        return;
    }

    if (cmd == "calibrate") {
        
        if (!parse_int(arg, value)) {
            printf("[DBG] 参数错误，格式应为: calibrate <0|1>\r\n");
            return;
        }
        if (value < 0 || value > 1) {
            printf("[DBG] calibrate 参数范围: 0~1\r\n");
            return;
        }
        if(value == 1){
            int32_t data[MOTOR_NUM] = {-1,-1,-1,-1,-1};
            esp_err_t err = esp_flash_erase_region(NULL, FLASH_ZERO_POS_ADDR, 4096);
            if (err != ESP_OK) {
                printf("[DBG] 擦除零位失败\r\n");
                return;
            }
            err = esp_flash_write(NULL, data, FLASH_ZERO_POS_ADDR, sizeof(data));
            if (err != ESP_OK) {
                printf("[DBG] 写入零位失败\r\n");
                return;
            }
        }
        std::string args = "{\"mode\":";
        args += std::to_string(value);
        args += "}";
        call_mcp_tool(202, "self.dog.calibrate", args);
        printf("[DBG] calibrate %d 已触发\r\n", value);
        return;
    }

    if (cmd == "loop") {
        if (!parse_int(arg, value)) {
            printf("[DBG] 参数错误，格式应为: loop <0|1>\r\n");
            return;
        }
        if (value < 0 || value > 1) {
            printf("[DBG] loop 参数范围: 0~1\r\n");
            return;
        }
        std::string args = "{\"flag\":";
        args += std::to_string(value);
        args += "}";
        call_mcp_tool(205, "self.dog.action_loop", args);
        printf("[DBG] action_loop %d 已触发\r\n", value);
        return;
    }

    printf("[DBG] 未知命令: %s (输入 help 查看)\r\n", cmd.c_str());
}

// Add a async task to MainLoop
void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, MAIN_EVENT_SCHEDULE);
}

// The Main Event Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainEventLoop() {
    // Raise the priority of the main event loop to avoid being interrupted by background tasks (which has priority 2)
    vTaskPrioritySet(NULL, 3);

    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, MAIN_EVENT_SCHEDULE |
            MAIN_EVENT_SEND_AUDIO |
            MAIN_EVENT_WAKE_WORD_DETECTED |
            MAIN_EVENT_VAD_CHANGE |
            MAIN_EVENT_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & MAIN_EVENT_ERROR) {
            SetDeviceState(kDeviceStateIdle);
            Alert(Lang::Strings::ERROR, last_error_message_.c_str(), "sad", Lang::Sounds::OGG_EXCLAMATION);
        }

        if (bits & MAIN_EVENT_SEND_AUDIO) {
            while (auto packet = audio_service_.PopPacketFromSendQueue()) {
                if (!protocol_->SendAudio(std::move(packet))) {
                    break;
                }
            }
        }

        if (bits & MAIN_EVENT_WAKE_WORD_DETECTED) {
            OnWakeWordDetected();
        }

        if (bits & MAIN_EVENT_VAD_CHANGE) {
            if (device_state_ == kDeviceStateListening) {
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        }

        if (bits & MAIN_EVENT_SCHEDULE) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

void Application::OnWakeWordDetected() {
    if (!protocol_) {
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        audio_service_.EncodeWakeWord();

        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                audio_service_.EnableWakeWordDetection(true);
                return;
            }
        }

        auto wake_word = audio_service_.GetLastWakeWord();
        ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_USE_AFE_WAKE_WORD || CONFIG_USE_CUSTOM_WAKE_WORD
        // Encode and send the wake word data to the server
        while (auto packet = audio_service_.PopWakeWordPacket()) {
            protocol_->SendAudio(std::move(packet));
        }
        // Set the chat state to wake word detected
        protocol_->SendWakeWordDetected(wake_word);
        SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
#else
        SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        // Play the pop up sound to indicate the wake word is detected
        audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
#endif
    } else if (device_state_ == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonWakeWordDetected);
    } else if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }
    
    clock_ticks_ = 0;
    auto previous_state = device_state_;
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);

    // Send the state change event
    DeviceStateEventManager::GetInstance().PostStateChangeEvent(previous_state, state);

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus(Lang::Strings::STANDBY);
            display->SetEmotion("neutral");
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(true);
            break;
        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");

            // Make sure the audio processor is running
            if (!audio_service_.IsAudioProcessorRunning()) {
                // Send the start listening command
                protocol_->SendStartListening(listening_mode_);
                audio_service_.EnableVoiceProcessing(true);
                audio_service_.EnableWakeWordDetection(false);
            }
            break;
        case kDeviceStateSpeaking:
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_service_.EnableVoiceProcessing(false);
                // Only AFE wake word can be detected in speaking mode
#if CONFIG_USE_AFE_WAKE_WORD
                audio_service_.EnableWakeWordDetection(true);
#else
                audio_service_.EnableWakeWordDetection(false);
#endif
            }
            audio_service_.ResetDecoder();
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    esp_restart();
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (device_state_ == kDeviceStateIdle) {
        ToggleChatState();
        Schedule([this, wake_word]() {
            if (protocol_) {
                protocol_->SendWakeWordDetected(wake_word); 
            }
        }); 
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {   
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (device_state_ != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    if (!audio_service_.IsIdle()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

void Application::SendMcpMessage(const std::string& payload) {
    Schedule([this, payload]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
        case kAecOff:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
            break;
        case kAecOnServerSide:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        case kAecOnDeviceSide:
            audio_service_.EnableDeviceAec(true);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::PlaySound(const std::string_view& sound) {
    audio_service_.PlaySound(sound);
}

void Application::XgoTask() {
    ESP_LOGI(TAG, "XGO task started");    
    while (true) {
        #ifdef CONFIG_BOARD_TYPE_LULU_ESP32S3
            xgo_control();
        #endif
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void Application::XgoRxTask() {
    ESP_LOGI(TAG, "XGO RX task started");
    while (true) {
        #ifdef CONFIG_BOARD_TYPE_LULU_ESP32S3
            xgo_rx();
        #endif
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}