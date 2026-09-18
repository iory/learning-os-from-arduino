/**
 * tp_mod_gpio.cpp - GPIO Module
 *
 * Usage:
 *   >>> import gpio
 *   >>> gpio.pin_mode(13, 1)
 *   >>> gpio.write(13, 1)
 *   >>> gpio.read(7)
 *   >>> gpio.analog(0)
 *
 * Also registers global aliases: pin_mode, digital_write, etc.
 */

#include "tp_module.h"
#include <Arduino.h>

static TpValue fn_pin_mode(TpValue *args, int argc)
{
    if (argc >= 2) pinMode(args[0].ival, args[1].ival);
    return tp_none();
}

static TpValue fn_write(TpValue *args, int argc)
{
    if (argc >= 2) digitalWrite(args[0].ival, args[1].ival);
    return tp_none();
}

static TpValue fn_read(TpValue *args, int argc)
{
    if (argc >= 1) return tp_int(digitalRead(args[0].ival));
    return tp_int(0);
}

static TpValue fn_analog(TpValue *args, int argc)
{
    if (argc >= 1) return tp_int(analogRead(args[0].ival));
    return tp_int(0);
}

static TpValue fn_delay(TpValue *args, int argc)
{
    if (argc >= 1) delay(args[0].ival);
    return tp_none();
}

static TpValue fn_millis(TpValue *args, int argc)
{
    (void)args; (void)argc;
    return tp_int((int32_t)millis());
}

static const TpCFuncEntry gpio_funcs[] = {
    {"pin_mode", fn_pin_mode},
    {"write",    fn_write},
    {"read",     fn_read},
    {"analog",   fn_analog},
};

static const TpModule gpio_module = {
    "gpio",
    gpio_funcs,
    sizeof(gpio_funcs) / sizeof(gpio_funcs[0]),
};

void tp_mod_gpio_register(void)
{
    tp_module_register(&gpio_module);

    // Also register global aliases (available without import)
    tp_builtin_register("pin_mode", fn_pin_mode);
    tp_builtin_register("digital_write", fn_write);
    tp_builtin_register("digital_read", fn_read);
    tp_builtin_register("analog_read", fn_analog);
    tp_builtin_register("delay", fn_delay);
    tp_builtin_register("millis", fn_millis);
}
