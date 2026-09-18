/**
 * tp_mod_led.cpp - LED Module
 *
 * Usage:
 *   >>> import led
 *   >>> led.blink(3)
 *   >>> led.on()
 *   >>> led.off()
 */

#include "tp_module.h"
#include <Arduino.h>

static TpValue fn_blink(TpValue *args, int argc)
{
    int n = (argc >= 1) ? args[0].ival : 1;
    int ms = (argc >= 2) ? args[1].ival : 200;
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < n; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(ms);
        digitalWrite(LED_BUILTIN, LOW);
        delay(ms);
    }
    return tp_none();
}

static TpValue fn_on(TpValue *args, int argc)
{
    (void)args; (void)argc;
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    return tp_none();
}

static TpValue fn_off(TpValue *args, int argc)
{
    (void)args; (void)argc;
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    return tp_none();
}

static const TpCFuncEntry led_funcs[] = {
    {"blink", fn_blink},
    {"on",    fn_on},
    {"off",   fn_off},
};

static const TpModule led_module = {
    "led",
    led_funcs,
    sizeof(led_funcs) / sizeof(led_funcs[0]),
};

void tp_mod_led_register(void)
{
    tp_module_register(&led_module);
}
