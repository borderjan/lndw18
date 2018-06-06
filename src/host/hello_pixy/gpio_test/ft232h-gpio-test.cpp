/******************************************
 * FTDI232H GPIO via libftdi Test Program *
 ******************************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <libftdi1/ftdi.h>
#include <libusb.h>
//#include "sortalgorithm.h"

#define FTDI_DEV_VID 0x0403
#define FTDI_DEV_PID 0x6014
#define SYN_IN_BITMASK 0x80  //pin 7 is sync in
#define SYN_OUT_BITMASK 0x10 //pin 4 is sync output
#define DATA_BITMASK 0x0F    //pin 0..3 are data

/*struct ftdi_chipdata{
    int vendorID,
    int productID,
    char const *deviceDescription,
    char const *serialNumber
};*/

static char *ftdiInitFails[3] = {"Unable to initialize libusb.", "Couldn't allocate struct buffers.", "Couldn't allocate read buffers."};
static char *ftdiBindFails[3] = {"Device already open.", "USB device is unavailable.", "Unknown interface."};
struct ftdi_context *devicecontext;
//struct ftdi_chipdata thisChip;
unsigned char wrbuf[1];
unsigned char rdbuf[1];

void robot_wait_for_handshake(void){
    *rdbuf = SYN_IN_BITMASK;
    while(rdbuf & SYN_IN_BITMASK){ //busy-wait for SYN_IN to go active (LOW)
        ftdi_read_pins(devicecontext, rdbuf);
        *rdbuf &= SYN_IN_BITMASK;
        usleep(200000);//pause for 0.2sec to not overwhelm usb
    }
    //robot has signaled done
    *wrbuf |= SYN:OUT_BITMASK; //turn SYN_OUT off (HIGH)
    ftdi_write_data(devicecontext, wrbuf, 1);
    while(!(rdbuf & SYN_IN_BITMASK)){ //busy-wait for SYN_IN to go inactive (HIGH)
        ftdi_read_pins(devicecontext, rdbuf);
        *rdbuf &= SYN_IN_BITMASK;
        usleep(200000);//pause for 0.2sec to not overwhelm usb
    }
    //handshake done
}
void robot_moveball(int slotnum);

inline void dealloc(){
    if(devicecontext){ free(devicecontext); devicecontext = NULL; }
}

void cleanupAll(){
    ftdi_disable_bitbang(devicecontext);
    ftdi_usb_close(devicecontext);
    ftdi_deinit(devicecontext);
    dealloc();
    exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[]){
    int tempReturnValue = 0;
    struct ftdi_device_list *alldevs = NULL, *curdev;
    int device;
    //setup device context
    printf("Allocating Device Context... ");fflush(stdout);
    if(!(devicecontext = ftdi_new())){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Context allocation FAIL : %s\n", ftdi_get_error_string(devicecontext));
        exit(EXIT_FAILURE);
    }
    //init device context
    printf("OK\nInitializing...");fflush(stdout);
    if((tempReturnValue = ftdi_init(devicecontext))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Initialization FAIL : %s / %s\n", ftdiInitFails[tempReturnValue + 3], ftdi_get_error_string(devicecontext));
        dealloc(); exit(EXIT_FAILURE);
    }
    //bind interface
    printf("OK\nSearching for devices...");
    if((tempReturnValue = ftdi_set_interface(devicecontext, INTERFACE_ANY))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Interface FAIL : %s / %s\n", ftdiBindFails[tempReturnValue + 3], ftdi_get_error_string(devicecontext));
        ftdi_deinit(devicecontext); dealloc(); exit(EXIT_FAILURE);
    }
    if((tempReturnValue = ftdi_usb_open(devicecontext, FTDI_DEV_VID , FTDI_DEV_PID ))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"No Devices found.\n");
        ftdi_deinit(devicecontext); dealloc(); exit(EXIT_FAILURE);
    }
    //enable bitbang mode
    //PIN0..7 = PORTD0..7
    //0 = input, 1 = output
                      //PORTD(76543210) = 
    unsigned char portmap = 0b00011111;
    //D0..3 : Slot number output, D4 : Robot Enable, D7, Robot Done, D5,6 : NC
    printf("OK\nStarting up...");fflush(stdout);
    if((tempReturnValue = ftdi_set_bitmode(devicecontext, portmap, BITMODE_BITBANG))){
        printf("FAIL\n");fflush(stdout);
        fprintf(stderr,"Unable to Bitbang. *sadface* \n");
        ftdi_usb_close(devicecontext); ftdi_deinit(devicecontext); dealloc(); exit(EXIT_FAILURE);
    }
    printf("OK\n");
    atexit(&cleanupAll);
    printf("Testing Output (PORTD)\n");
    printf("Please Verify status on SCORBOT Control Box\n");
    *wrbuf = 0xFF; //all off
    ftdi_write_data(devicecontext, wrbuf, 1);
    int j;
    sleep(1);
    for(j = 0; j < 5; j++){
        *wrbuf &= ~(1<<j);
        printf("Output: %02hhx\n", *wrbuf);
        ftdi_write_data(devicecontext, wrbuf, 1);
        sleep(1);
        *wrbuf |= (1<<j);
    }
    *wrbuf = 0xFF;
    ftdi_write_data(devicecontext, wrbuf, 1);
    printf("Waiting For Robot...\n");
    while(1){
        ftdi_read_pins(devicecontext, rdbuf);
        printf("PORTD7 : %c\n", (*rdbuf & 0x80)?'H':'L');
        sleep(1);
    }
    exit(EXIT_SUCCESS);
}
