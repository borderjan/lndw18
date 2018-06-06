/********************************************************
 *                Ball Sorting Algorithm                *
 * -keeps all the pixycam stuff away from robot control *
 ********************************************************/

#include "sortalgorithm.h"

//other includes
#include <stdlib.h>

struct movements{
    movements *next;
    int slotnumber;
};

struct movements *head = NULL;
struct movements *end = NULL;

void add_a_move(int slotnumber){
    movements *newmove = (movements *)malloc(sizeof(movements));
    newmove->next = NULL;
    newmove->slotnumber = slotnumber;
    if(end){
        end->next = newmove;
        end = newmove;
    }else{
        head = end = newmove;
    }
}

int get_next_move(){
    static int slotnum = -1;
    static struct movements *toKill;
    if(head){
        slotnum = head->slotnumber;
        toKill = head;
        head = head->next;
        free(toKill);
        return slotnum;
    }else{
        end = NULL;
        return -1; //move fifo empty
    }
    return -1;
}

void look_at_balls(){
    //all the pixycam detection and sorting goes here
    while(head){ get_next_move(); } //clean fifo
    add_a_move(3);
    add_a_move(1);
    add_a_move(4);
    add_a_move(5);
    add_a_move(2);
    add_a_move(7);
    add_a_move(6);
    //TODO actual algorithm
}
