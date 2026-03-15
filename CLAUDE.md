# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> For detailed architecture, development conventions, and deployment info, see `AGENTS.md`.

## Project

RIG-puppy (噜噜) is a 5-DOF quadruped robotic dog built on ESP32-S3 and the [XiaoZhi AI voice framework](https://xiaozhi.me). It supports voice interaction, camera, IMU, and UART-controlled servos.

- **Current version**: 1.8.8 (in `CMakeLists.txt`)
- **Target board**: `LULU_ESP32S3` (set via menuconfig or `CONFIG_BOARD_TYPE_LULU_ESP32S3=y`)
- **License**: Non-commercial only

## Build Commands

Requires ESP-IDF >= 5.4.0, CMake >= 3.16, Python 3.x.

```bash
idf.py set-target esp32s3       # One-time target setup
idf.py menuconfig               # Configure board type, language, audio debugger, etc.
idf.py build                    # Build
idf.py flash monitor            # Flash and open serial monitor
idf.py -p COM3 flash monitor    # Flash to specific port
idf.py fullclean                # Clean all build artifacts (fixes most build errors)
idf.py merge-bin                # Merge binaries for release
python scripts/release.py lulu-esp32s3   # Generate release package for this board
```

## Architecture

### Core Layers

```
main.cc  →  Application (Singleton)  →  Board (Singleton)
                    │
        ┌───────────┼─────────────────┐
        │           │                 │
  AudioService   Protocol          McpServer
  (voice I/O)  (WebSocket/MQTT)  (tool invocation)
        │
  XGO (servo motion) — lulu-esp32s3 board only
```

**Application** (`application.cc/h`) — central state machine with FreeRTOS event loop. Drives state transitions: `Starting → Idle → Connecting → Listening/Speaking`.

**Board** (`boards/common/board.h`) — abstract singleton factory. RIG-puppy implementation is `LuluEsp32S3Board` in `boards/lulu-esp32s3/`. Hardware pins are in `boards/lulu-esp32s3/config.h`.

**AudioService** — dual-task architecture (I/O task + Opus codec task). Three queues: encode, decode, send. Uses ESP-SR for wake word ("小陆同学") and VAD.

**Protocol** — WebSocket (default) or MQTT. Handles bidirectional Opus audio + JSON at 24000 Hz, 60ms frames. See `docs/websocket.md`.

**XGO Motion** (`boards/lulu-esp32s3/xgo.cc/h`) — UART control of 5 EM3 bus servos. High-level actions (Wave, Sit, Hug, etc.) in `xgo_action.cc/h`. To add an action: define ID in `xgo_action.h`, implement in `xgo_action.cc`, add to switch-case.

**McpServer** (`mcp_server.cc/h`) — Model Context Protocol (JSON-RPC 2.0) for AI tool discovery/invocation.

**Settings** (`settings.cc/h`) — NVS flash-based runtime settings. Compile-time config via Kconfig.

### Audio Data Flow

```
Mic → [Input Task] → Encode Queue → [Opus Encoder] → Send Queue → Protocol → Server
Server → Protocol → Decode Queue → [Opus Decoder] → Playback Queue → [Output Task] → Speaker
```

### Flash Layout (16MB default)

| Region | Address | Size |
|--------|---------|------|
| NVS | 0x9000 | 16KB |
| OTA Data | 0xD000 | 8KB |
| Model (SPIFFS) | 0x10000 | 960KB |
| App OTA0 | 0x100000 | 10.5MB |
| App OTA1 | 0xB80000 | 4.5MB |

## Code Conventions

- C++11/14; C++ exceptions enabled (`CONFIG_COMPILER_CXX_EXCEPTIONS=y`)
- Logging: `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE` with per-file `TAG`
- Critical ops: `ESP_ERROR_CHECK()`
- Memory: prefer `std::unique_ptr`/`std::shared_ptr`
- Size-optimized build (`CONFIG_COMPILER_OPTIMIZATION_SIZE=y`); PSRAM required for wake word and AFE

## Key Debugging

- **Serial commands**: available for gait debug and parameter snapshots (see recent commits)
- **Audio debugger**: enable in menuconfig → set UDP server address → use `scripts/audio_debug_server.py`
- **Gait debug**: enable via serial command to print servo parameter snapshots
- **Build failures**: `idf.py fullclean` resolves most issues
- **Wake word not working**: verify PSRAM is enabled in sdkconfig
