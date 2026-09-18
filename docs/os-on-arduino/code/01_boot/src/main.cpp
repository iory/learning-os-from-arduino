/**
 * Chapter 1: Boot Sequence Check
 *
 * This sketch demonstrates the boot sequence by examining:
 * - BSS section (zero-initialized variables)
 * - DATA section (initialized variables copied from Flash)
 * - Vector table location
 * - Stack pointer
 */

#include <Arduino.h>

// Global variable in BSS section (uninitialized, should be zero)
int bss_var;

// Global variable in DATA section (initialized, copied from Flash)
int data_var = 12345;

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }
    while (Serial.read() != 'S') { delay(1); }

    Serial.println("=== Boot Sequence Check ===");
    Serial.println();

    // BSS section variable (should be zero-cleared)
    Serial.print("bss_var (should be 0): ");
    Serial.println(bss_var);

    // DATA section variable (initial value should be copied)
    Serial.print("data_var (should be 12345): ");
    Serial.println(data_var);

    Serial.println();
    Serial.println("=== Memory Addresses ===");

    // Show variable addresses
    Serial.print("&bss_var:  0x");
    Serial.println((uint32_t)&bss_var, HEX);

    Serial.print("&data_var: 0x");
    Serial.println((uint32_t)&data_var, HEX);

    // Vector table location (VTOR register)
    Serial.print("VTOR:      0x");
    Serial.println(SCB->VTOR, HEX);

    // Stack pointer
    uint32_t sp;
    __asm volatile ("MRS %0, MSP" : "=r" (sp));
    Serial.print("MSP:       0x");
    Serial.println(sp, HEX);

    Serial.println();
    Serial.println("If you see this, boot sequence completed!");
}

void loop()
{
    // Nothing to do
}
