/***************************
 * Robot Control Functions *
 ***************************/

#include "robot.h"
#include <stdio.h>

static int n_tx = -1;
static int n_rx = -1;

int do_handshake_master(){
    gpio_hold(0,0);//turn off any holds
    gpio_write_pin(hs_tx, SET);
    if(gpio_wait_for_pin(hs_rx, SET, 0, 60)){ // 1 minute timeout - anti-block-measure
        return -1;
    }
    gpio_write_pin(hs_tx, CLR);
    return gpio_wait_for_pin(hs_rx, CLR, 0, 60);
}

int do_handshake_slave(){
    gpio_hold(0,0);
    gpio_wait_for_pin(hs_rx, SET, 0, -1);
    gpio_write_pin(hs_tx, SET);
    gpio_wait_for_pin(hs_rx, CLR, 0, -1);
    gpio_write_pin(hs_tx, CLR);
}

int write_data(int data){
    if(n_tx > 0){
        pinmask pmask = PIN_0;
        int dmask = 1;
        gpio_hold(0,1); //hold writes for batching
        while(pmask < PIN_ALL){
            if(pmask & data_tx){
                gpio_write_pin(pmask, (data & dmask)?HIGH:LOW);
                dmask << 1;
            }
            pmask << 1;
        }
        gpio_hold(0,0); //batch write
        return 0;
    }else{
        fprintf(stderr, "No TX data lines defined!\n");
        return -1;
    }
}

int read_data(int *data){
    if(n_rx > 0){
        pinvalue val;
        gpio_read_pin(data_rx, &val);
        *data = 0;
        pinmask pmask = PIN_0;
        int dmask = 0x01;
        int wr = 0;
        while((pmask < PIN_ALL) && !dmask){
            if(pmask & data_rx){
                wr = (val & pmask)?-1:0; //expand databit to entire word
                *data = *data | (wr & dmask); //put databit in return word
                dmask = dmask << 1; //select next bit
            }
            pmask = (pinmask)pmask << 1; //select next pin
        }
    }else{
        fprintf(stderr, "No RX data lines defined\n");
        return -1;
    }
}
int setup_robot(pinmask hs_rxpin, pinmask hs_txpin, pinmask data_rxpins, pinmask data_txpins){
    if(!gpio_issinglepin(hs_rxpin) || !gpio_issinglepin(hs_txpin)){
        fprintf(stderr, "Only assign one pin to each handshake direction please.\n");
        return 1;
    }
    if(hs_rxpin == hs_txpin){
        fprintf(stderr, "Handshake RX and TX on same pin!\n");
        return 2;
    }
    if((hs_rxpin & data_rxpins) || (hs_rxpin & data_txpins) || (hs_txpin & data_rxpins) || (hs_txpin & data_txpins)){
        fprintf(stderr, "Data pins overlap handshake pins\n");
        return 3;
    }
    if(data_rxpins & data_txpins){
        fprintf(stderr, "Data RX and TX overlap each other\n");
        return 4;
    }
    hs_tx = hs_txpin;
    hs_rx = hs_rxpin;
    data_rx = data_rxpins;
    data_tx = data_txpins;
    //count tx and rx data pins
    pinmask p = PIN_0;
    n_rx = 0;
    n_tx = 0;
    while(p < PIN_ALL){
        if(p & data_rx){ n_rx++; }
        if(p & data_tx){ n_tx++; }
        p = (pinmask) p << 1;
    }
    gpio_hold(1,0);//enable pinmode hold
    gpio_set_pin_mode(OUTPUT, hs_txpin | data_txpins);
    gpio_set_pin_mode(INPUT, hs_rxpin | data_rxpins);
    gpio_hold(0,0);//commit mode change
    return 0;
}
