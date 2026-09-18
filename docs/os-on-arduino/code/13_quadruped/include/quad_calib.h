// この機体のサーボ構成 (本文 13.8)。
//
// ポリシーの 8 出力の順に並んでいる。**並べ替えてはいけない。** 順番は学習した
// チェックポイントから来ていて、arduino_quad_policy.h の先頭にも書いてある。
//
// sign と zero は校正で決める値で、機体ごとに違う。ここに書いてあるのは
// 本書の実機のもの。自分の機体では host/quad_host.py calibrate で出し直す
// こと -- ホストとファームが同じ値を持っていないとロボットは暴れる。
#pragma once

#define QUAD_NUM_JOINTS 8

// バス上の ID。FL_hip, FL_knee, RL_hip, RL_knee, RR_hip, RR_knee, FR_hip, FR_knee
static const unsigned char QUAD_SERVO_ID[QUAD_NUM_JOINTS] = {3, 4, 7, 8, 5, 6, 1, 2};

// URDF の関節が回る向きにサーボが回るかどうか。+1 か -1 で、推測しないこと。
static const float QUAD_SERVO_SIGN[QUAD_NUM_JOINTS] = {
    -1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// 関節角 0 rad に対応するサーボの読み値。
//
// このサーボは絶対角のエンコーダを持っていて機械的な止まりが無いので、
// ホーンを何度で取り付けても、ここを測り直せば辻褄が合う (本文 13.8.1)。
// 下の 2048 は「真ん中」であって、正しい値ではない。校正して置き換えること。
static const float QUAD_SERVO_ZERO[QUAD_NUM_JOINTS] = {
    2048.0f, 2048.0f, 2048.0f, 2048.0f, 2048.0f, 2048.0f, 2048.0f, 2048.0f};
