// WiFi 版 (src/main_wifi.cpp) の接続先。arduino_secrets.h に写して埋めること。
// arduino_secrets.h は .gitignore に入っている。パスワードをコミットしない。
//
//   cp include/arduino_secrets.example.h include/arduino_secrets.h
#pragma once

// 0: 既存のネットワークに入る (SSID / パスワードは下)
// 1: R4 自身がアクセスポイントになり、PC がそこへ入る。ルータが要らない
#define QUAD_WIFI_AP      0
#define QUAD_WIFI_SSID    "your-network"
#define QUAD_WIFI_PASS    "your-password"

// PC からの (vx, wz) 指令を待つ UDP ポート。host/quad_host.py の --udp-port と同じ
#define QUAD_UDP_PORT     9000
