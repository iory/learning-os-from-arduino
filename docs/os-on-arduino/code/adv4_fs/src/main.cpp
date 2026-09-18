/**
 * 応用編 第4章: 最小ファイルシステム — Flash 上に LittleFS を載せる
 *
 * RA4M1 内蔵の Data Flash (8 KB) を LittleFS のブロックデバイスとして使う。
 *
 * 起動するたびにカウンタを 1 増やして書き戻すので、電源を入れ直しても
 * 値が残っていることで永続化を確認できる。4.2 で説明した電源断耐性
 * （LittleFS のコピーオンライト）が効いていることの一番簡単な確かめ方。
 *
 * 注意: Data Flash の書き換え回数は 1 万〜10 万回程度。デモとはいえ
 * リセットを繰り返しすぎないこと。
 */

#include <Arduino.h>
#include "fs.h"
#include "fs_flash_hal.h"

static void print_err(const char *what, int err)
{
    Serial.print(F("[fs] "));
    Serial.print(what);
    Serial.print(F(" failed: "));
    Serial.println(err);
}

// 起動回数を /boot_count に読み書きする
static void demo_boot_count(lfs_t *lfs)
{
    lfs_file_t f;
    uint32_t count = 0;

    int err = lfs_file_open(lfs, &f, "/boot_count", LFS_O_RDWR | LFS_O_CREAT);
    if (err) { print_err("open", err); return; }

    lfs_file_read(lfs, &f, &count, sizeof(count));
    count++;
    lfs_file_rewind(lfs, &f);
    lfs_file_write(lfs, &f, &count, sizeof(count));
    lfs_file_close(lfs, &f);   // close で初めて Flash へ確定する

    Serial.print(F("boot count = "));
    Serial.println(count);
}

// ルートディレクトリの一覧（シェルの ls 相当）
static void demo_ls(lfs_t *lfs)
{
    lfs_dir_t dir;
    struct lfs_info info;

    int err = lfs_dir_open(lfs, &dir, "/");
    if (err) { print_err("dir_open", err); return; }

    Serial.println(F("--- / ---"));
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
        if (info.type == LFS_TYPE_DIR) {
            Serial.print(F("  [DIR ] "));
            Serial.println(info.name);
        } else {
            Serial.print(F("  [FILE] "));
            Serial.print(info.name);
            Serial.print(F("  "));
            Serial.print(info.size);
            Serial.println(F(" B"));
        }
    }
    lfs_dir_close(lfs, &dir);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }
    while (Serial.read() != 'S') { delay(1); }

    Serial.println(F("=== Advanced 4: LittleFS on Data Flash ==="));
    Serial.print(F("data flash: "));
    Serial.print(FLASH_DATA_SIZE);
    Serial.print(F(" B / sector "));
    Serial.print(FLASH_SECTOR_SIZE);
    Serial.println(F(" B"));

    int err = fs_init();
    if (err) {
        print_err("mount", err);
        while (1) { delay(1000); }
    }
    Serial.println(F("mounted."));

    lfs_t *lfs = fs_get();
    demo_boot_count(lfs);
    demo_ls(lfs);

    Serial.println(F("done. リセットすると boot count が増えます"));
}

void loop() { }
