// ArduinoQuad — servo calibration wizard.
//
// Establishes the three things the walking sketch cannot guess, and prints a
// finished JOINTS[] table to paste into arduino_quad.ino:
//
//   id    which servo on the bus drives which joint
//   zero  the raw count that corresponds to a joint angle of 0 rad
//   sign  whether raw counts increase or decrease as the joint angle increases
//
// Get any of these wrong and the robot does not walk badly -- it thrashes. So
// this asks the human to confirm each one against something visible on the
// machine instead of inferring it.
//
// Board : Arduino Uno R4 WiFi, servo bus on Serial1 at 1 Mbaud
// Serial: 115200. Type the letter of a step and follow the prompts.
//
// NOT VERIFIED ON HARDWARE. The bus calls follow the Feetech SCServo library
// (class SMS_STS); check them against your library version before trusting the
// output.

#include <SCServo.h>

SMS_STS st;

// Policy order. This is NOT arbitrary: it is the order the trained network's 8
// outputs come in, printed at the top of arduino_quad_policy.h. Keep it.
static const char *JOINT_NAME[8] = {
    "FL_hip", "FL_knee", "RL_hip", "RL_knee",
    "RR_hip", "RR_knee", "FR_hip", "FR_knee",
};

// Joint angle at the home stance, radians, same order (from the exported
// policy's default_joint_pos: hip +1.031, knee -1.763).
static const float HOME_RAD[8] = {
    1.0307f, -1.7629f, 1.0307f, -1.7629f,
    1.0307f, -1.7629f, 1.0307f, -1.7629f,
};

// The policy commands home +- 0.25 rad of action, and the URDF allows +-2 rad.
// Anything the calibration produces has to keep this span inside 0..4095 counts.
static const float SPAN_RAD = 1.2f;

static const float RAD_PER_COUNT = 6.28318530718f / 4096.0f;
static const uint8_t MAX_ID = 20;

// Bus speed. The STS3215 leaves the factory at 1 Mbps and the walking loop needs
// it: at 50 Hz the whole read-eight-positions + write-eight-targets exchange has
// to fit inside 20 ms, and one position read is 16 bytes of protocol.
//
//   1 000 000 bps   16 B = 0.16 ms   8 reads ~ 2.4-5.3 ms   fits easily
//     500 000 bps   16 B = 0.32 ms   8 reads ~ 3.5-6.5 ms   fits
//     115 200 bps   16 B = 1.39 ms   8 reads ~ 12-16 ms     does NOT fit
//
// (Plus a 64-byte sync write: 0.64 ms at 1 M, 5.6 ms at 115200.) So 115200 is
// not an option for this control rate even though the servos support it.
static const uint32_t BUS_BAUD_DEFAULT = 1000000UL;
// Bauds the wizard tries when it cannot find eight servos. A unit set to a
// different rate at the factory is silent rather than wrong, which is easy to
// misread as a wiring fault.
static const uint32_t BAUD_TRY[] = {1000000UL, 500000UL, 250000UL, 128000UL,
                                    115200UL, 76800UL, 57600UL, 38400UL};
static uint32_t g_baud = BUS_BAUD_DEFAULT;

static int8_t  g_id[8];     // -1 = not yet assigned
static int16_t g_zero[8];
static int8_t  g_sign[8];

// ---------------------------------------------------------------- helpers --
static void flushIn() { while (Serial.available()) Serial.read(); }

// Takes an F() string: every call site passes one, and on this core F()
// yields __FlashStringHelper*, which does not convert to const char*.
static void waitEnter(const __FlashStringHelper *prompt) {
  Serial.println(prompt);
  Serial.println(F("   [Enter] を押すと次へ"));
  flushIn();
  for (;;) {
    if (Serial.available()) {
      int c = Serial.read();
      if (c == '\n' || c == '\r') { flushIn(); return; }
    }
    delay(5);
  }
}

static int readLineInt(const char *prompt) {
  Serial.print(prompt);
  flushIn();
  String s = "";
  for (;;) {
    if (Serial.available()) {
      int c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (s.length()) { Serial.println(s); return s.toInt(); }
      } else if (c >= ' ') {
        s += (char)c;
      }
    }
    delay(5);
  }
}

static void releaseAll() {
  for (uint8_t id = 1; id <= MAX_ID; ++id) st.EnableTorque(id, 0);
}

// -------------------------------------------------------------- 1. scan ----
static int pingAll(bool verbose) {
  int found = 0;
  for (uint8_t id = 1; id <= MAX_ID; ++id) {
    if (st.Ping(id) != -1) {
      if (verbose) {
        int p = st.ReadPos(id);
        Serial.print(F("  ID ")); Serial.print(id);
        Serial.print(F("  pos=")); Serial.println(p);
      }
      found++;
    }
  }
  return found;
}

static void stepScan() {
  Serial.println(F("\n=== 1. バスをスキャン ==="));
  Serial.print(F("  現在 ")); Serial.print(g_baud); Serial.println(F(" bps"));
  int found = pingAll(true);
  Serial.print(F("  応答 ")); Serial.print(found); Serial.println(F(" 個"));
  if (found == 8) return;

  Serial.println(F("  8 個ではないので、他のボーレートを試します"));
  int best = found;
  uint32_t bestBaud = g_baud;
  for (unsigned i = 0; i < sizeof(BAUD_TRY) / sizeof(BAUD_TRY[0]); ++i) {
    if (BAUD_TRY[i] == g_baud) continue;
    Serial1.end();
    Serial1.begin(BAUD_TRY[i]);
    delay(60);
    int n = pingAll(false);
    Serial.print(F("    ")); Serial.print(BAUD_TRY[i]);
    Serial.print(F(" bps -> ")); Serial.print(n); Serial.println(F(" 個"));
    if (n > best) { best = n; bestBaud = BAUD_TRY[i]; }
  }
  Serial1.end();
  Serial1.begin(bestBaud);
  delay(60);
  g_baud = bestBaud;
  Serial.print(F("  -> ")); Serial.print(g_baud);
  Serial.print(F(" bps で ")); Serial.print(best); Serial.println(F(" 個"));

  if (best < 8) {
    Serial.println(F("  !! まだ 8 個そろいません。順に確認:"));
    Serial.println(F("     ・電源はバッテリから取っているか(8 個 x ストール 2.7 A)"));
    Serial.println(F("     ・半二重の方向切替をするアダプタを通しているか"));
    Serial.println(F("     ・ID が重複していないか(重複すると衝突して両方応答しない)"));
    Serial.println(F("     ・デイジーチェーンの途中で断線していないか"));
  }
  if (g_baud != BUS_BAUD_DEFAULT) {
    Serial.println(F("  !! 既定の 1 Mbps ではありません。50 Hz 制御に必要な帯域は"));
    Serial.println(F("     500 kbps 以上です。Feetech のツールで全数 1 Mbps に"));
    Serial.println(F("     揃えてから歩行スケッチを動かしてください"));
    Serial.println(F("     (arduino_quad.ino の BUS_BAUD も同じ値にすること)。"));
  }
}

// --------------------------------------------------------------- 2. map ----
// Wiggle one servo at a time and let the human name the joint that moved.
static void stepMap() {
  Serial.println(F("\n=== 2. ID と関節の対応 ==="));
  Serial.println(F("  1 個ずつ小さく動かします。動いた関節の番号を入力してください:"));
  for (int j = 0; j < 8; ++j) {
    Serial.print(F("    ")); Serial.print(j); Serial.print(F(" = ")); Serial.println(JOINT_NAME[j]);
  }
  waitEnter(F("  ロボットを浮かせて(脚が自由に動く状態にして)ください。"));

  for (int i = 0; i < 8; ++i) g_id[i] = -1;

  for (uint8_t id = 1; id <= MAX_ID; ++id) {
    if (st.Ping(id) == -1) continue;
    int p0 = st.ReadPos(id);
    if (p0 < 0) continue;
    Serial.print(F("\n  --- ID ")); Serial.print(id); Serial.println(F(" を動かします ---"));
    st.EnableTorque(id, 1);
    // +-120 counts ~= +-10 deg, slow, three times so it is unmistakable.
    for (int k = 0; k < 3; ++k) {
      st.WritePosEx(id, constrain(p0 + 120, 0, 4095), 300, 50); delay(450);
      st.WritePosEx(id, constrain(p0 - 120, 0, 4095), 300, 50); delay(450);
    }
    st.WritePosEx(id, p0, 300, 50); delay(400);
    st.EnableTorque(id, 0);
    int j = readLineInt("  動いた関節の番号 (0-7、分からなければ -1) > ");
    if (j >= 0 && j < 8) {
      if (g_id[j] != -1) {
        Serial.print(F("  !! ")); Serial.print(JOINT_NAME[j]);
        Serial.println(F(" は既に割当済み。上書きします"));
      }
      g_id[j] = (int8_t)id;
      Serial.print(F("  -> ")); Serial.print(JOINT_NAME[j]);
      Serial.print(F(" = ID ")); Serial.println(id);
    }
  }
  for (int j = 0; j < 8; ++j) {
    if (g_id[j] == -1) {
      Serial.print(F("  !! 未割当: ")); Serial.println(JOINT_NAME[j]);
    }
  }
}

// -------------------------------------------------------------- 3. zero ----
// The q = 0 pose, measured on the asset:
//
//   hip axis and knee axis are on the SAME vertical line (thigh exactly plumb)
//   the toe then sits 8 mm FORWARD of that line (shank leans 4 deg forward)
//
// So "thigh plumb" sets the hip zero and "toe just forward of the knee" sets the
// knee zero. Both are eye-checkable against a plumb line or a phone level.
//
// It does not have to be perfect. Training randomizes the encoder zero by
// +-0.03 rad (1.7 deg) and the reset joint angles by +-0.15 rad (8.6 deg), so a
// couple of degrees of calibration error is inside what the policy already
// handles. Getting the SIGN or the ID wrong is what breaks it, not 2 deg.
static void stepZero() {
  Serial.println(F("\n=== 3. ゼロ点 (q = 0 の姿勢) ==="));
  Serial.println(F("  トルクを切ります。4 本すべての脚を次の姿勢にしてください:"));
  Serial.println(F("    ・大腿(股関節軸 -> 膝軸)が鉛直。下げ振りかスマホの水平器で見る"));
  Serial.println(F("    ・そのとき足先は膝の真下から 8 mm ほど前"));
  Serial.println(F("  これが URDF の q = 0 です。数度ずれても構いません"));
  Serial.println(F("  (学習でエンコーダゼロ点を +-1.7 度、関節角を +-8.6 度ランダム化済み)。"));
  releaseAll();
  waitEnter(F("  4 本とも真下に垂らしたら Enter。"));
  for (int j = 0; j < 8; ++j) {
    if (g_id[j] < 0) continue;
    int p = st.ReadPos(g_id[j]);
    g_zero[j] = (int16_t)p;
    Serial.print(F("  ")); Serial.print(JOINT_NAME[j]);
    Serial.print(F(" (ID ")); Serial.print(g_id[j]);
    Serial.print(F(")  zero=")); Serial.println(p);
  }
}

// -------------------------------------------------------------- 4. sign ----
// Positive joint angle = the toe swings BACKWARD (toward the tail), for BOTH the
// hip and the knee, on all four legs. "Tail" is the end of the tray with the
// solid section -- the robot leads with the other end.
static void stepSign() {
  Serial.println(F("\n=== 4. 符号 ==="));
  Serial.println(F("  この機体の約束: どの関節も「正 = 足先が後ろ(尾側)へ振れる」。"));
  Serial.println(F("  尾側 = トレーの中実部がある方の端です(進行方向の反対)。"));
  releaseAll();
  for (int j = 0; j < 8; ++j) {
    if (g_id[j] < 0) continue;
    Serial.print(F("\n  --- ")); Serial.print(JOINT_NAME[j]);
    Serial.println(F(" ---"));
    Serial.println(F("  この関節だけを動かして、足先を「後ろ」へ振ってください"));
    Serial.println(F("  (股関節なら脚全体、膝なら下腿だけ)。そのまま保持して Enter。"));
    waitEnter(F("  保持できたら Enter。"));
    int p1 = st.ReadPos(g_id[j]);
    int d = p1 - g_zero[j];
    if (abs(d) < 60) {
      Serial.println(F("  !! 変化が小さすぎます(60 counts 未満)。もっと大きく振って、"));
      Serial.println(F("     この関節をやり直してください。sign は +1 のままにします"));
      g_sign[j] = 1;
    } else {
      g_sign[j] = (d > 0) ? 1 : -1;
    }
    Serial.print(F("  delta=")); Serial.print(d);
    Serial.print(F("  -> sign=")); Serial.println(g_sign[j]);
  }
}

// -------------------------------------------------------------- 5. check ---
static void stepCheck() {
  Serial.println(F("\n=== 5. 可動範囲が 0..4095 に収まるか ==="));
  bool ok = true;
  for (int j = 0; j < 8; ++j) {
    if (g_id[j] < 0) continue;
    float lo = HOME_RAD[j] - SPAN_RAD, hi = HOME_RAD[j] + SPAN_RAD;
    long c1 = lround(lo / RAD_PER_COUNT * g_sign[j] + g_zero[j]);
    long c2 = lround(hi / RAD_PER_COUNT * g_sign[j] + g_zero[j]);
    long cmin = min(c1, c2), cmax = max(c1, c2);
    Serial.print(F("  ")); Serial.print(JOINT_NAME[j]);
    Serial.print(F("  counts ")); Serial.print(cmin);
    Serial.print(F(" .. ")); Serial.print(cmax);
    if (cmin < 0 || cmax > 4095) {
      Serial.println(F("   !! 範囲外。ホーンを付け直して 3 からやり直すこと"));
      ok = false;
    } else {
      Serial.println(F("   ok"));
    }
  }
  if (!ok) {
    Serial.println(F("  ホーンを 90 度ずらして組み直すと大抵収まります。"));
    Serial.println(F("  カウントの 0/4095 をまたぐと、そこで指令が飛んで脚が振り回されます。"));
  }
}

// -------------------------------------------------------------- 6. home ----
static void stepHome() {
  Serial.println(F("\n=== 6. home 姿勢へゆっくり移動 ==="));
  Serial.println(F("  4 本とも同じ形で、膝の頂点が後ろ(尾側)に引けていれば正解です。"));
  Serial.println(F("  全部前に出ていたら、前後の規約が学習と逆になっています。"));
  waitEnter(F("  ロボットを浮かせたまま Enter。"));
  int start[8];
  for (int j = 0; j < 8; ++j) {
    if (g_id[j] < 0) continue;
    start[j] = st.ReadPos(g_id[j]);
    st.EnableTorque(g_id[j], 1);
  }
  for (int k = 0; k <= 100; ++k) {
    float a = k / 100.0f;
    for (int j = 0; j < 8; ++j) {
      if (g_id[j] < 0) continue;
      long tgt = lround(HOME_RAD[j] / RAD_PER_COUNT * g_sign[j] + g_zero[j]);
      long p = lround(start[j] + a * (tgt - start[j]));
      st.WritePosEx(g_id[j], constrain(p, 0, 4095), 0, 0);
    }
    delay(20);
  }
  Serial.println(F("  移動完了。姿勢を確認してください。"));
}

// ------------------------------------------------------------ 8. eeprom ----
// Write the CURRENT position as the servo's middle point (2048) by writing 128
// to register 40 -- Feetech's one-key middle-point calibration. Run it with the
// robot held at the q = 0 pose and every `zero` in the table becomes 2048.
//
// Worth doing: the calibration then lives in the servo, so reflashing the MCU
// cannot lose it, and centring the joint's zero at 2048 leaves +-180 deg of
// count headroom against the +-69 deg the policy actually uses -- the 0/4095
// wrap simply cannot be reached any more.
//
// The SIGN deliberately stays in software. Register 18 ("Phase") is a motor
// phase/direction setting, not a report-only inversion: if the encoder direction
// and the drive direction disagree the loop becomes positive feedback and the
// servo runs to a stop. A sign flip in this sketch is one character and shows up
// in version control.
static void stepEeprom() {
  Serial.println(F("\n=== 8. ゼロ点を EEPROM に書く (任意) ==="));
  Serial.println(F("  いまの位置を中点(2048)として各サーボに記録します。"));
  Serial.println(F("  必ず 3 と同じ q = 0 の姿勢で実行してください。"));
  Serial.println(F("  書いたあと zero は全関節 2048 になります。"));
  int go = readLineInt("  実行する? (1 = はい / 0 = やめる) > ");
  if (go != 1) { Serial.println(F("  中止")); return; }
  releaseAll();                      // EEPROM 書き込み前にトルクを切る
  delay(100);
  for (int j = 0; j < 8; ++j) {
    if (g_id[j] < 0) continue;
    int before = st.ReadPos(g_id[j]);
    // 一発中点校正: レジスタ 40 に 128。ライブラリに専用 API があればそちらでも可。
    st.writeByte(g_id[j], 40, 128);
    delay(60);
    int after = st.ReadPos(g_id[j]);
    g_zero[j] = 2048;
    Serial.print(F("  ")); Serial.print(JOINT_NAME[j]);
    Serial.print(F(" (ID ")); Serial.print(g_id[j]);
    Serial.print(F(")  ")); Serial.print(before);
    Serial.print(F(" -> ")); Serial.print(after);
    if (abs(after - 2048) > 40) {
      Serial.println(F("   !! 2048 になっていません。ライブラリの API を確認"));
    } else {
      Serial.println(F("   ok"));
    }
  }
  Serial.println(F("  完了。5 の範囲チェックをやり直してから 7 を実行してください。"));
}

// -------------------------------------------------------------- 7. emit ----
static void stepEmit() {
  Serial.println(F("\n=== 7. arduino_quad.ino に貼るコード ==="));
  Serial.println(F("static const JointMap JOINTS[QUAD_NJ] = {"));
  for (int j = 0; j < 8; ++j) {
    Serial.print(F("    {"));
    Serial.print(g_id[j] < 0 ? 0 : g_id[j]);
    Serial.print(F(", "));
    Serial.print(g_sign[j] >= 0 ? F("+1") : F("-1"));
    Serial.print(F(", "));
    Serial.print(g_zero[j]);
    Serial.print(F("},  // "));
    Serial.println(JOINT_NAME[j]);
  }
  Serial.println(F("};"));
}

// ------------------------------------------------------------------ main ---
void setup() {
  Serial.begin(115200);
  Serial1.begin(BUS_BAUD_DEFAULT);
  st.pSerial = &Serial1;
  delay(600);
  for (int j = 0; j < 8; ++j) { g_id[j] = -1; g_zero[j] = 2048; g_sign[j] = 1; }
  Serial.println(F("\nArduinoQuad 校正ウィザード"));
  Serial.println(F("  1 バスをスキャン (応答が 8 個でなければボーレートも探索)"));
  Serial.println(F("  2 ID と関節の対応"));
  Serial.println(F("  3 ゼロ点 (脚をまっすぐ下に垂らす)"));
  Serial.println(F("  4 符号 (足先を後ろへ振る)"));
  Serial.println(F("  5 可動範囲が 0..4095 に収まるか"));
  Serial.println(F("  6 home 姿勢へ移動して目視確認"));
  Serial.println(F("  7 JOINTS[] を出力"));
  Serial.println(F("  8 ゼロ点を EEPROM に書く (任意。q=0 の姿勢で)"));
  Serial.println(F("  r トルク解放"));
  Serial.println(F("番号を入力 > "));
}

void loop() {
  if (!Serial.available()) return;
  int c = Serial.read();
  switch (c) {
    case '1': stepScan();  break;
    case '2': stepMap();   break;
    case '3': stepZero();  break;
    case '4': stepSign();  break;
    case '5': stepCheck(); break;
    case '6': stepHome();  break;
    case '7': stepEmit();  break;
    case '8': stepEeprom(); break;
    case 'r': releaseAll(); Serial.println(F("released")); break;
    default:  return;
  }
  Serial.println(F("\n番号を入力 > "));
}
