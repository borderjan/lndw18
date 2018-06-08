/*****************************
 * FTDI232H GPIO via libftdi *
 *****************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <libftdi1/ftdi.h>
#include <libusb.h>
#include <time.h>

#include "gpio.h"

//Device Selection
#define FTDI_DEV_VID 0x0403
#define FTDI_DEV_PID 0x6014

static const char *ftdiInitFails[3] = {"Unable to initialize libusb.", "Couldn't allocate struct buffers.", "Couldn't allocate read buffers."};
static const char *ftdiBindFails[3] = {"Device already open.", "USB device is unavailable.", "Unknown interface."};
struct ftdi_context *device;

static int device_initialized = 0;
static unsigned char pinmap = 0x00; //all input
static unsigned char rdbuf[1], wrbuf[1];
static int hold_pinmode = 0;
static int hold_write = 0;
static unsigned char pinmap_hold = pinmap;
static unsigned char wrbuf_hold;
static int errcode = 0;
inline void dealloc(){
    if(device){ free(device); device = NULL; }
    device_initialized = 0;
}

int gpio_shutdown(){
    if(!device_initialized){
        fprintf(stderr,"Device already shut down\n");
        return -1;
    }
    ftdi_disable_bitbang(device);
    ftdi_usb_close(device);
    ftdi_deinit(device);
    dealloc();
    return 0;
}

int gpio_initialize(){
    *rdbuf = 0;
    *wrbuf = 0xff;
    wrbuf_hold = 0xff;
    if(device_initialized){
        fprintf(stderr, "Device already initialized\n");
        return -1;
    }
    int tempReturnValue = 0;
    //struct ftdi_device_list *alldevs = NULL, *curdev;
    //setup device context
    printf("Allocating Device Context... ");fflush(stdout);
    if(!(device = ftdi_new())){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Context allocation FAIL : %s\n", ftdi_get_error_string(device));
        return 1;
    }
    //init device context
    printf("OK\nInitializing...");fflush(stdout);
    if((tempReturnValue = ftdi_init(device))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Initialization FAIL : %s / %s\n", ftdiInitFails[tempReturnValue + 3], ftdi_get_error_string(device));
        dealloc(); return 2;
    }
    //bind interface
    printf("OK\nSearching for devices...");
    if((tempReturnValue = ftdi_set_interface(device, INTERFACE_ANY))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Interface FAIL : %s / %s\n", ftdiBindFails[tempReturnValue + 3], ftdi_get_error_string(device));
        ftdi_deinit(device); dealloc(); return 3;
    }
    //open Device
    if((tempReturnValue = ftdi_usb_open(device, FTDI_DEV_VID , FTDI_DEV_PID ))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"No Devices found.\n");
        ftdi_deinit(device); dealloc(); return 4;
    }
    //set bitbang mode
    printf("OK\nStarting up...");fflush(stdout);
    if((tempReturnValue = ftdi_set_bitmode(device, pinmap, BITMODE_BITBANG))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Unable to Bitbang. *sadface* \n");
        ftdi_usb_close(device); ftdi_deinit(device); dealloc(); return 5;
    }
    device_initialized = 1;
    printf("OK\n");
    return 0;
}

int gpio_reset(){
    if(!device_initialized){
        fprintf(stderr,"Device not initialized!\n");
        return -1;
    }
    pinmap = 0x00;
    pinmap_hold = 0x00;
    if((ftdi_set_bitmode(device, pinmap, BITMODE_BITBANG))){
        fprintf(stderr,"Device Error: %s\n", ftdi_get_error_string(device));
        return 1;
    }
    return 0;
}

inline int check_pin_mask(pinmask pin){
    if(pin > PIN_ALL){
        fprintf(stderr, "Invalid Pin Mask\n");
        return 1;
    }else{ return 0; }
}

int gpio_set_pin_mode(pinmode mode, pinmask pin){
    if(check_pin_mask(pin)){ return 1; }
    switch(mode){
        case INPUT:
            pinmap_hold &= ~pin; break;
        case OUTPUT:
            pinmap_hold |= pin; break;
        default:
            fprintf(stderr,"Invalid Pin Mode\n"); return 2;
    }
    if(!hold_pinmode){
        return gpio_commit(1,0);
        return 0;
    }else{ return 0; };
}

int gpio_write_pin(pinmask pin, pinvalue value){
    static unsigned char data = 0xff;
    if(check_pin_mask(pin)){ return 1; }
    if(hold_pinmode && !hold_write){ //direct write needs consistent pin state
        fprintf(stderr, "Uncommited pin mode change present!\n");
        return 2;
    }
    switch(value){
        case LOW:
            wrbuf_hold &= ~pin; break;
        case HIGH:
            wrbuf_hold |= pin; break;
        default:
            fprintf(stderr, "Invalid pin write\n"); return 3;
    }
    if(!hold_write){
        return gpio_commit(0,1);
    }else{ return 0; }
}

int gpio_read_pin(pinmask pin, pinvalue *value){
    if(!device_initialized){
        fprintf(stderr, "Device not Initialized!\n"); return -1;
    }
    if(check_pin_mask(pin)){ return 1; }
    ftdi_read_pins(device, rdbuf);
    //output pins and unpolled pins return '1' in read buffer
    *rdbuf = *rdbuf | ~pin | pinmap;
    return 0;
}

int gpio_wait_for_pin(pinmask pin, pinvalue value, int trigger, int timeout){
    unsigned char polldat[2];
    struct timespec start, clk;
    //check if only a single pin was polled
    if(!gpio_issinglepin(pin)){
        fprintf(stderr, "Cannot wait for more than one pin!\n");
        return 1;
    }
    if(pin & pinmap){ // trying to poll an output pin makes no sense (input pins are 0 in pinmap)
        fprintf(stderr, "Trying to poll an output pin\n");
        return 2;
    }
    if(trigger){ //flank triggered - setup polldat w/ current pin level
        ftdi_read_pins(device, polldat);
        polldat[0] &= (int) pin;
    }
    //premask target pinvalue
    value &= pin;
    //approx. usb response time is ~110msec, so ~80msec sleep after poll should be ok
    //for 200msec polling period
    //setup initial time for timeout criterion
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    while(1){
        //save old pinvalue
        polldat[1] = polldat[0] & pin;
        //read new pinvalue
        ftdi_read_pins(device, polldat);
        if(value == (*polldat & pin)){ //correct level
            if(!trigger){ //level triggered - return immediately
                return 0;
            }else if(polldat[1] != (*polldat & pin)){ //transition detected
                return 0;
            }
        }
        //still wrong pinlevel - check timeout
        if(timeout > -1){
            clock_gettime(CLOCK_MONOTONIC_RAW, &clk);
            if (timeout < (clk.tv_sec - start.tv_sec)){ //timeout
                return -1;
            }
        }
        usleep(80000);//sleep for 80msec
    }
    return -1;
}

int gpio_hold(int modechange, int write){
    int commitMode = 0, commitWrite = 0;
    if(hold_pinmode && !modechange){
        commitMode = 1;
    }
    if(hold_write && !write){
        commitWrite = 1;
    }
    if(commitWrite && !commitMode){
        fprintf(stderr, "Cannot commit Write w/o committing Modechange\n"); return 1;
    }
    gpio_commit(commitMode, commitWrite);
    hold_pinmode = modechange;
    hold_write = write;
    return 0;
}

int gpio_commit(int modechange, int write){
    if(modechange){
        pinmap = pinmap_hold;
        if(ftdi_set_bitmode(device, pinmap, BITMODE_BITBANG)){
            fprintf(stderr, "Unable to set pin modes\n"); return -1;
        }
        hold_pinmode = 0;
    }
    if(write){
        *wrbuf = wrbuf_hold;
        if((errcode = ftdi_write_data(device, wrbuf, 1)) < 0){
            fprintf(stderr, "Unable to write : Error %i\n", errcode); return errcode;
        }
        hold_write = 0;
    }
    return 0;
}

int gpio_revert(int modechange, int write){
    if(modechange){
        pinmap_hold = pinmap;
        hold_pinmode = 0;
    }
    if(write){
        wrbuf_hold = *wrbuf;
        hold_write = 0;
    }
    return 0;
}

int gpio_issinglepin(pinmask pin){
    pinmask p = PIN_0;
    while(p < PIN_ALL){
        if(p == pin){ return 1; }
        else{ p = p << 1; }
    }
    return 0;
}
