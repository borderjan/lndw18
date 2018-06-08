//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "pixy.h"
#include "robot.h"
#include "gpio.h"

#define BLOCK_BUFFER_SIZE    25
#define NSIGS 7
// Pixy Block buffer //
struct Block blocks[BLOCK_BUFFER_SIZE];

static bool run_flag = true;
//slot width 60mm, slot gap 5mm each side, top slot offset 5mm
static int slot_xcoord_lower[NSIGS+1] = {005,075,145,215,285,355,425,495};
static int slot_xcoord_upper[NSIGS+1] = {065,135,205,275,345,415,485,555};
static int sigs[NSIGS];
static int sig_perm_lut[NSIGS] = {1,2,3,4,5,6,7};

void handle_SIGINT(int unused)
{
    // On CTRL+C - abort! //

    run_flag = false;
}

//get ball count
int ballcount(){
    int i;
    int bc = 0;
    for(i = 0; i < NSIGS; i++){
        if(sigs[i] != -1){ bc++; }
    }
    return bc;
}

//pseudo to change Sorting LUT
int setSortingTable(int ballNumber){
    return 0;
}

int main(int argc, char * argv[])
{
    int      i = 0;
    int      index;
    int      blocks_copied;
    int      pixy_init_status;
    char     buf[128];

    // Catch CTRL+C (SIGINT) signals //
    signal(SIGINT, handle_SIGINT);
    //printf("Hello Pixy:\n libpixyusb Version: %s\n", __LIBPIXY_VERSION__);
    // Connect to Pixy //
    pixy_init_status = pixy_init();
    // Was there an error initializing pixy? //
    if(!pixy_init_status == 0){
        // Error initializing Pixy //
        printf("pixy_init(): ");
        pixy_error(pixy_init_status);
        return pixy_init_status;
    }
    // Request Pixy firmware version //
    {
        uint16_t major;
        uint16_t minor;
        uint16_t build;
        int      return_value;

        return_value = pixy_get_firmware_version(&major, &minor, &build);

        if (return_value) {
            // Error //
            printf("Failed to retrieve Pixy firmware version. ");
            pixy_error(return_value);

            return return_value;
        } else {
            // Success //
            printf(" Pixy Firmware Version: %d.%d.%d\n", major, minor, build);
        }
    }

    /*    #if 0
     *    // Pixy Command Examples //
     *    {
     *    int32_t response;
     *    int     return_value;
     *
     *    // Execute remote procedure call "cam_setAWB" with one output (host->pixy) parameter (Value = 1)
     *    //
     *    //   Parameters:                 Notes:
     *    //
     *    //   pixy_command("cam_setAWB",  String identifier for remote procedure
     *    //                        0x01,  Length (in bytes) of first output parameter
     *    //                           1,  Value of first output parameter
     *    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
     *    //                   &response,  Pointer to memory address for return value from remote procedure call
     *    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
     *    //
     *
     *    // Enable auto white balance //
     *    pixy_command("cam_setAWB", UINT8(0x01), END_OUT_ARGS,  &response, END_IN_ARGS);
     *
     *    // Execute remote procedure call "cam_getAWB" with no output (host->pixy) parameters
     *    //
     *    //   Parameters:                 Notes:
     *    //
     *    //   pixy_command("cam_setAWB",  String identifier for remote procedure
     *    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
     *    //                   &response,  Pointer to memory address for return value from remote procedure call
     *    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
     *    //
     *
     *    // Get auto white balance //
     *    return_value = pixy_command("cam_getAWB", END_OUT_ARGS, &response, END_IN_ARGS);
     *
     *    // Set auto white balance back to disabled //
     *    pixy_command("cam_setAWB", UINT8(0x00), END_OUT_ARGS,  &response, END_IN_ARGS);
}
#endif
*/
    //pixycam setup done, now for the robot...
    gpio_initialize(); //init adafruit ft232h gpio interface
    gpio_reset(); //put gpios in defined state
    //setup comm pins
    setup_robot(PIN_7, PIN_4, PIN_NONE, (pinmask)(PIN_0 | PIN_1 | PIN_2 | PIN_3));
    //setup start lamp / button pins
    gpio_set_pin_mode(INPUT, PIN_6);
    gpio_set_pin_mode(OUTPUT, PIN_5);
    gpio_write_pin((pinmask)(PIN_5 | PIN_4), HIGH); // turn lamp off & clear robot signal line
    printf("Waiting for Robot...\n");
    do_handshake_master();
    int programState = 3;
    printf("Detecting blocks...\n");
    int pos = 1;
    while(run_flag)
    {
        switch(programState){
            case 0: //waiting for buzzer / console input
                //also, keep polling pixycam
                //console input will probably not happen in v1
                //TODO
                // Wait for new blocks to be available //
                while(!pixy_blocks_are_new() && run_flag);
                if(!run_flag){ break; }//quit program early
                // Get blocks from Pixy //
                blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);
                if(blocks_copied < 0){
                    // Error: pixy_get_blocks //
                    printf("pixy_get_blocks(): ");
                    pixy_error(blocks_copied);
                }
                // Display received blocks //
                printf("frame %d:\n", i);
                // transform block array into signature-based x-coord array
                // ONLY WHEN NOT SORTING!
                if(1){
                    memset(sigs,-1,sizeof(sigs));
                    for(index = 0; index < blocks_copied; ++index) {
                        if(blocks[index].signature <= NSIGS){
                            sigs[blocks[index].signature-1]=blocks[index].x;
                        }
                        //old example code
                        blocks[index].print(buf);
                        printf("  %s\n", buf);
                    }
                }
                //sigs[] now has x-coordinates for each detected signature
                //only the last x-coord of doppelganger sigs is retained
                /*
                 * for each ball in order:
                 *   get ball slot
                 *   pick ball from slot
                 *   put ball at end
                 */
                //display all x-coordinates
                int j;
                for (j=0; j<NSIGS ; j++){
                    printf("x%i =%i\n",j,sigs[j]);
                }
                i++;
                sleep(1);
                break;
            case 1: //computing moves
                //TODO
                break;
            case 2: //sorting in progress, main robot control happens here
                //TODO
                break;
            case 3: //robot testing - manual activation only
                gpio_write_pin(PIN_0, (pos&1)?LOW:HIGH);
                gpio_write_pin(PIN_1, (pos&2)?LOW:HIGH);
                gpio_write_pin(PIN_2, (pos&4)?LOW:HIGH);
                gpio_write_pin(PIN_3, (pos&8)?LOW:HIGH);
                do_handshake_master();
                pos++;
                if(pos > 6){ pos = 1; }
                break;
        }

    }
    //tell robot to move to standby position
    gpio_write_pin(PIN_0, LOW);
    gpio_write_pin(PIN_1, LOW);
    gpio_write_pin(PIN_2, LOW);
    gpio_write_pin(PIN_3, LOW);
    do_handshake_master();
    //reset ouput pins
    gpio_write_pin(PIN_0, HIGH);
    gpio_write_pin(PIN_1, HIGH);
    gpio_write_pin(PIN_2, HIGH);
    gpio_write_pin(PIN_3, HIGH);
    //cleanup
    gpio_shutdown();
    pixy_close();
    printf("Program Done.\n");
    exit(EXIT_SUCCESS);
}
