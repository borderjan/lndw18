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
#define NSIGS 6

struct robot_move_list{
    struct robot_move_list *next;
    int signumber;
};
// Pixy Block buffer //
struct Block blocks[BLOCK_BUFFER_SIZE];

static bool run_flag = true;
//slot width 60mm, slot gap 5mm each side, top slot offset 5mm
//static int slot_xcoord_lower[NSIGS+1] = {005,075,145,215,285,355,425,495};
//static int slot_xcoord_upper[NSIGS+1] = {065,135,205,275,345,415,485,555};
// x-coordinates of signatures, indexed by signature
static int sigs[NSIGS];
static int sig_slots[NSIGS] = {2,4,1,6,3,5};
// color code / signature indexed by sort rank (low = first)
static unsigned char color_sort_rank[NSIGS] = {'r','g','b','y','n','p'};
static unsigned int sig_sort_rank[NSIGS];
void handle_SIGINT(int unused){
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

// convert signature coordinate array into signature slotnumber array
int coord_to_slot(){
    //TODO conversion algorithm here
    // !Needs info about orientation of cam!
    return 0;
}

int main(int argc, char * argv[]){
    if(argc < 2){
        puts("Please input sorting order as arguments");
        puts("\tColor codes:");
        puts("\ty - yellow");
        puts("\tr - red");
        puts("\tg - green");
        puts("\tb - blue");
        puts("\tp - pink");
        puts("\tn - brown");
        exit(EXIT_SUCCESS);
    }
    int i = 0;
    int rank = 0;
    int was_put = 0x3f;
    for(i = 0; i < NSIGS; i++){
        color_sort_rank[i] = '\0';
        sig_sort_rank[i] = -1;
    }
    for(i = 1; i < argc; i++){
        //printf("ARG%i = %c (%02hhx / %02hhx)\n", i, argv[i][0], argv[i][0], 'r');
        switch(argv[i][0]){
            case 'r':
                if(was_put & 0x01){
                    sig_sort_rank[rank] = 0;
                    color_sort_rank[rank++] = argv[i][0];
                    was_put &= 0x3e;
                }
                break;
            case 'g':
                if(was_put & 0x02){
                    sig_sort_rank[rank] = 1;
                    color_sort_rank[rank++] = argv[i][0];
                    was_put &= 0x3d;
                }
                break;
                case 'b':
                if(was_put & 0x04){
                    sig_sort_rank[rank] = 2;
                    color_sort_rank[rank++] = argv[i][0];
                    was_put &= 0x3b;
                }
                break;
            case 'y':
                if(was_put & 0x08){
                    sig_sort_rank[rank] = 3;
                    color_sort_rank[rank++] = argv[i][0];
                    was_put &= 0x37;
                }
                break;
            case 'n':
                if(was_put & 0x10){
                    sig_sort_rank[rank] = 4;
                    color_sort_rank[rank++] = argv[i][0];
                    was_put &= 0x2f;
                }
                break;
            case 'p':
                if(was_put & 0x20){
                    sig_sort_rank[rank] = 5;
                    color_sort_rank[rank++] = argv[i][0];
                    was_put &= 0x1f;
                }
                break;
            default:
                fprintf(stderr, "Unknown Colorcode : %c\n",argv[i][0]);
                break;
        }
    }
    if(was_put){
        fprintf(stderr, "Not all Colorcodes present!\n");
        fprintf(stderr, "Please provide ALL colorcodes.\n");
        exit(EXIT_FAILURE);
    }
    i = 0;
    int      index;
    int      blocks_copied;
    int      pixy_init_status;
    char     buf[128];
    struct robot_move_list *this_move = NULL, *del_move = NULL, *final_move = NULL;
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
    if(gpio_initialize()){ //init adafruit ft232h gpio interface
        //Error - need to shutdown
        pixy_close();
        exit(EXIT_FAILURE);
    }
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
    int sig = -1;
    int wait = 1;
    int hsflag = 0;
    int ballcount = 0;
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
                //display all x-coordinates
                int j;
                for (j=0; j<NSIGS ; j++){
                    printf("x%i =%i\n",j,sigs[j]);
                }
                i++;
                if(!wait){
                    //check for gap in slot positions - later
                    programState = 1; //GO!
                    break;
                }
                if(gpio_wait_for_pin(PIN_6, LOW, 1, 1) == 0){//wait for buzzer pin flank w/ 1 sec timeout
                    wait = 0;// prep for go
                }
                //sleep(1); //prevent console flooding
                break;
            case 1: //computing moves
                //TODO
                /*
                 * for each ball in order:
                 *   get ball slot
                 *   pick ball from slot
                 *   put ball at end
                 */
                //clean residual moves from move fifo
                while(this_move){
                    del_move = this_move;
                    this_move = this_move->next;
                    free(del_move);
                }
                del_move = NULL;
                final_move = NULL;
                //get signature by rank
                ballcount = 0;
                for(rank = 0; rank < NSIGS; rank++){
                    if(sigs[sig_sort_rank[rank]] != -1){//signature present
                        ballcount++;//count total signatures observed
                        pos = sig_sort_rank[rank];//specify signature to move
                        //create new move
                        del_move = malloc(sizeof(struct robot_move_list));
                        del_move->next = NULL;
                        del_move->signumber = pos;
                        if(final_move){
                            //append new move
                            final_move->next = del_move;
                            final_move = del_move;
                        }else{
                            //set move list head & end
                            this_move = final_move = del_move;
                        }
                    }
                }
                coord_to_slot();
                // all moves done - add stop move
                del_move = malloc(sizeof(struct robot_move_list));
                del_move->next = NULL;
                del_move->signumber = -1;
                final_move->next = del_move;
                final_move = del_move;
                del_move = NULL; //safety cleanup
                programState = 2; //Make it move!
                break;
            case 2: //sorting in progress, main robot control happens here
                //TODO
                gpio_write_pin(PIN_5, LOW);//turn button LED on
                while(this_move){
                    //get signature to move
                    sig = this_move->signumber;
                    if(sig == -1){
                        pos = 15;//done, move bot to standby
                    }else{
                        pos = sig_slots[sig]; //get signature slot
                        for(index = 0; index < NSIGS; index++){
                            if(sig_slots[index] == pos){//currently moving ball
                                sig_slots[index] = ballcount-1; //moved ball is now at rear of queue
                            }else if(sig_slots[index] > pos){//balls behind the moved one
                                sig_slots[index] = sig_slots[index] - 1; //balls behind the moved one roll up 1 slot
                            }
                        }
                    }
                    //write data pins
                    gpio_write_pin(PIN_0, (pos&1)?LOW:HIGH);
                    gpio_write_pin(PIN_1, (pos&2)?LOW:HIGH);
                    gpio_write_pin(PIN_2, (pos&4)?LOW:HIGH);
                    gpio_write_pin(PIN_3, (pos&8)?LOW:HIGH);
                    if((hsflag = do_handshake_master())){ //MOVE IT!
                        //handshake error - robot unresponsive
                        run_flag = 0;
                    }
                    del_move = this_move;
                    this_move = this_move->next;
                    free(del_move); //remove executed move from fifo
                }
                break;
            case 3: //robot testing - manual activation only
                //cycles through positions 1 to 6
                gpio_write_pin(PIN_0, (pos&1)?LOW:HIGH);
                gpio_write_pin(PIN_1, (pos&2)?LOW:HIGH);
                gpio_write_pin(PIN_2, (pos&4)?LOW:HIGH);
                gpio_write_pin(PIN_3, (pos&8)?LOW:HIGH);
                hsflag = do_handshake_master();
                if(hsflag){ run_flag = 0; }
                pos++;
                if(pos > 6){ pos = 1; }
                break;
        }

    }
    if(!hsflag){//tell robot to move to standby position if no handshake error occurred
        gpio_write_pin(PIN_0, LOW);
        gpio_write_pin(PIN_1, LOW);
        gpio_write_pin(PIN_2, LOW);
        gpio_write_pin(PIN_3, LOW);
        do_handshake_master();
    }
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
