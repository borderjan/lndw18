/*******************************
 * GPIO Controlling Functions *
 *******************************/
#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H


//Pin masks etc.
typedef enum pinmask  { PIN_NONE = 0x00,
                        PIN_0 = 0x01,
                        PIN_1 = 0x02,
                        PIN_2 = 0x04,
                        PIN_3 = 0x08,
                        PIN_4 = 0x10,
                        PIN_5 = 0x20,
                        PIN_6 = 0x40,
                        PIN_7 = 0x80,
                        PIN_ALL = 0xff
                        };
typedef enum pinmode  { INPUT, OUTPUT };
typedef enum pinvalue { LOW = 0x00, HIGH = 0xff };

extern int gpio_initialize();
extern int gpio_shutdown();
extern int gpio_reset();
extern int gpio_set_pin_mode(pinmode mode, pinmask pin);
extern int gpio_write_pin(pinmask pin, pinvalue value);
extern int gpio_read_pin(pinmask pin, pinvalue *value);
extern int gpio_wait_for_pin(pinmask pin, pinvalue value, int trigger, int timeout);
extern int gpio_hold(int modechange, int write);
extern int gpio_commit(int modechange, int write);
extern int gpio_revert(int modechange, int write);
extern int gpio_issinglepin(pinmask pin);
#endif
