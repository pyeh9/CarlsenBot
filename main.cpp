#include "mbed.h"
#include "Motor.h"
#include "Servo.h"
#include "string.h"
#include <iostream>

/* Claw servo constants
 * Open: 0 max open
 * Close 1 max close **Don't set to 1
 */
#define OPEN 0.43
#define CLOSE 0.87

/* Z position height constants */
#define UP 0.36 
#define DOWN 0.512

// Object declarations
I2C i2c(p9, p10);
Serial pc(USBTX, USBRX);
Motor motor_x1(p21, p5, p6, &i2c);
Motor motor_x2(p22, p11, p12, &i2c);
Motor motor_y(p23, p15, p16, &i2c);
Motor motor_z(p24, p13, p14);
Servo claw(p25);
AnalogIn z_dist(p20);
Serial device(p28, p27);  // tx, rx
Ticker pos_monitor;
DigitalOut leds[] = { (LED1), (LED2), (LED3), (LED4) };

const int XWIDTH = 208;
const int YWIDTH = 554;
char resetAll = 0x4E; // to reset all devices on one i2c bus, write data 0x4E to address 0x00.
float height;

//function declarations
void send_char( char* );
void getAllMeasurements(void);
void moveX(int pos);
void moveY(int pos);
void moveZ(float pos);
void movePiece(char c1, char r1, char c2, char r2);

int main()
{
    // initializing routine for all i2c devices
    // no motor_z.init since it it not a i2c device
    // since motor_x2's orientation is switched, flip "is_reversed" bit
    i2c.write(0x00, &resetAll, 1);
    motor_x1.init(0x20, 1, false);
    motor_x2.init(0x22, 2, false);
    motor_y.init(0x24, 3, true);
    motor_x2.is_reversed = true;

    /* for debugging
    motor_x1.getTicks();
    motor_x2.getTicks();
    motor_y.getTicks();
    */
    
    pos_monitor.attach(&getAllMeasurements, 0.003); // interrupt routine to measure current position on all axis
    
    // get claw ready
    // note to self: in future revisions, implement a way to tare X and Y
    moveZ(UP);
    claw = OPEN;
    
    //main variables
    char char_buf[4];
    char rchar = 'z';
    char status = '0';
    int count = 0;
    int numLeds = sizeof(leds)/sizeof(DigitalOut);

    // wake up device
    // done by sending EasyVR 'b' character
    // recommended wake routine: keep sending 'b' until confirmation, via 'o'
    device.putc('b');
    while (device.getc()!='o') {
        device.putc('b');
        leds[0] = 1;
        wait(0.2);
    }

//reset all leds initially
    for(int i = 0; i < numLeds; i++) {
        leds[i] = 0;
    }

//wait until ready
    while (device.readable()!=0) {}

    //movePiece('b', '2', 'b', '4');

//main loop
    while (1) {        
        leds[0] = 0;
        leds[1] = 0;
        leds[2] = 0;
        leds[3] = 1;    //solid led4 means mbed is in wait for request mode
        while (pc.readable() == 0) {}   //wait for a readable byte
        status = pc.getc();

        if(status == 'n') {
            // REQUEST FOR VR COMMAND
            while ( count < 4 ) {
                //flash led 4 for user confirmation of new recog
                leds[3] = !leds[3];
                //start recog based off either words or numbers
                if(count == 0 || count == 2) {
                    device.putc('d');
                    wait(0.001);
                    device.putc('C'); //Ben's trained words (C) // FLOHRS (B)
                    wait(0.001);
                    //pc.printf("Say a column letter\n");
                } else if(count == 1|| count == 3) {
                    device.putc('i');
                    wait(0.001);
                    device.putc('D'); //Wordset #3 - numbers
                    wait(0.001);
                    //pc.printf("Say a row number\n");
                }
                //wait for confirmation of new recog
                while (device.readable()!=0) {}
                rchar=device.getc();

                // Word recognized
                // Each move is a sequence of 4 words. Each word recognized makes an LED light up
                if (rchar=='r') {
                    wait(.001);
                    device.putc(' ');
                    while (device.readable()!=0) {}
                    rchar=device.getc();
                    //pc.printf("Recognized:");
                    //pc.putc(rchar-'A'+'a');
                    char_buf[count] = (rchar-'A'+'a');
                    leds[count] = 1;
                    count++;
                    // error
                } else if (rchar=='s') {
                    wait(.001);
                    device.putc(' ');
                    while (device.readable()!=0) {}
                    rchar=device.getc();
                    //pc.printf("Recognized:");
                    //pc.putc(rchar-'A'+'0');
                    char_buf[count] = (rchar-'A'+'0');
                    leds[count] = 1;
                    count++;
                } else if (rchar=='e') {
                    wait(.001);
                    device.putc(' ');
                    while (device.readable()!=0) {}
                    rchar=device.getc();
                    device.putc(' ');
                    while (device.readable()!=0) {}
                    rchar=device.getc();
                }
            }
            //successful input string created

            //reset all leds
            for(int i = 0; i < numLeds; i++) {
                leds[i] = 0;
            }
            count = 0;
            //send data to EEBOX
            send_char(char_buf);
        } else if( status == 'm') {
            leds[0] = 0;
            leds[1] = 1;
            leds[2] = 0;
            leds[3] = 1;
            //status for claw move
            //wait for reply
            pc.putc('m');
            while( pc.readable() == 0);
            count = 0;
            char_buf[count] = pc.getc(); //grab char
            leds[count] = 1;
            count++;
            leds[0] = 0;
            leds[1] = 0;
            leds[2] = 0;
            leds[3] = 0;
            while (count < 4) {
                leds[count] = 1;
                pc.putc(' ');
                while (pc.readable() == 0) {}  //wait for response (beginning of string)
                char_buf[count] = pc.getc();
                count++;
            }
            leds[0] = 0;
            leds[1] = 0;
            leds[2] = 0;
            leds[3] = 0;
            //Recieved the 4 char buffer for movement
            movePiece(char_buf[0], char_buf[1], char_buf[2], char_buf[3]);
            //wait(10);
            //pc.printf("mbed recieved following string for move:\n");
            //pc.printf("%c%c%c%c\n\n", char_buf[0], char_buf[1], char_buf[2], char_buf[3]);
            count = 0;
        }
    }


}

/***** Function Definitions *****/

// sends a character to the PC
void send_char( char output[] )
{
    //Successful Request, send confirmation byte
    char status;
    do {
        pc.putc('n');
        while (pc.readable() == 0) {}       //wait for a readable byte
        status = pc.getc();
    } while( status != ' ');

    for(int i = 0; i < 4; i++) {
        do {
            while (pc.readable() == 0) {}   //wait for a readable byte
            status = pc.getc();
        } while( status != ' ');

        pc.putc(output[i]);
    }

    //terminate communication
    do {
        while (pc.readable() == 0) {}   //wait for a readable byte
        status = pc.getc();
    } while( status != ' ');

    pc.putc('s');                       //send termination byte

}
void moveX(int pos)   //make pos signed
{
    float diff1, diff2, spd1, spd2, lower = 0.06, upper = 0.8, integral1 = 0.0, integral2 = 0.0;
    //int K = 1000;
    int K = 2000, Ki = 0.001;
    while (!((abs(motor_x1.ticks - pos) <= 9) && (abs(motor_x2.ticks - pos) <= 9))) {
        diff1 = (pos - motor_x1.ticks);
        diff2 = (pos - motor_x2.ticks);
        integral1 += (diff1*0.001);
        integral2 += (diff2*0.001);
        spd1 = (diff1/K) + (Ki*integral1);
        spd2 = (diff2/K) + (Ki*integral2);
        if (spd1 >= upper) spd1 = upper;
        else if(spd1 <= lower && spd1 > 0.0) spd1 = lower;
        else if(spd1 < 0.0 && spd1 >= -lower) spd1 = -lower;
        else if(spd1 <= -upper) spd1 = -upper;
        //rationalize a speed for the motor x2
        if (spd2 >= upper) spd2 = upper;
        else if(spd2 <= lower && spd2 > 0.0) spd2 = lower;
        else if(spd2 < 0.0 && spd2 >= -lower) spd2 = -lower;
        else if(spd2 <= -upper) spd2 = -upper;

        //printf("Motor %d is at position %d, moving with speed %f\n", motor_x1.motorID, motor_x1.ticks, spd1); // debugging
        //printf("Motor %d is at position %d, moving with speed %f\n", motor_x2.motorID, motor_x2.ticks, spd2); // debugging

        //adjust speed according to difference between two motors
        if (abs(motor_x1.ticks - motor_x2.ticks) <= 10) {
            motor_x1.speed(spd1);
            motor_x2.speed(spd2);
            wait(0.001);
        }
        //if motor x1 is lagging behind motor x2, slow motor x2
        else if (abs(diff1) > abs(diff2)) {
            //apply new speed settings to x motors
            motor_x2.speed(spd2/2);
            motor_x1.speed(spd1);
            wait(0.001);
        }
        //if motor x2 is lagging behind motor x1, slow motor x1
        else if (abs(diff2) > abs(diff1)) {
            //apply new speed settings to x motors
            motor_x1.speed(spd1/2);
            motor_x2.speed(spd2);
            wait(0.001);
        }
    }//while
    motor_x1.speed(0);
    motor_x2.speed(0);
    wait(0.5);
    //adjusting
    while (!((abs(motor_x1.ticks - pos) <= 6) && (abs(motor_x2.ticks - pos) <= 6))) {
        diff1 = (pos - motor_x1.ticks);
        diff2 = (pos - motor_x2.ticks);
        if(diff1 > 0) spd1 = 0.05;
        else if(diff1 < 0) spd1 = -0.05;
        else if(diff1 == 0) spd1 = 0;
        if(diff2 > 0) spd2 = 0.05;
        else if(diff2 < 0) spd2 = -0.05;
        else if(diff2 == 0) spd2 = 0;
        //printf("Motor %d is at position %d, moving with speed %f\n", motor_x1.motorID, motor_x1.ticks, spd1); // debugging
        //printf("Motor %d is at position %d, moving with speed %f\n", motor_x2.motorID, motor_x2.ticks, spd2); // debugging

        motor_x1.speed(spd1);
        motor_x2.speed(spd2);
        wait(0.005);
    }
    motor_x1.speed(0);
    motor_x2.speed(0);
    wait(0.5);
    //printf("Done! Motor %d is at position %d\n", motor_x1.motorID, motor_x1.ticks); // debugging
    //printf("Done! Motor %d is at position %d\n", motor_x2.motorID, motor_x2.ticks); // debugging
}

void moveZ(float pos)  // ~0.31 for top, ~0.46 for bottom
{
    float tempPos = pos;
    if(pos == DOWN){
        pos = 0.47;
    }
    //float height1, height2;
    while(!((abs(height - pos) < 0.01))) {
        //pc.printf("z height: %f\n", height);
        if(height < pos) {
            motor_z.speed(0.025);
            wait(0.02);
            motor_z.speed(0.015);
        } else if(height > pos) {
            motor_z.speed(-0.16);
            wait(0.02);
            motor_z.speed(-0.08);
        }
    } // ~ .47
    pos = tempPos; 
    wait(0.5);
    while(!((abs(height - pos) < 0.01))) { // adjusting loop
        //pc.printf("z height: %f\n", height);
        if(height < pos) {
            motor_z.speed(0.02);
            wait(0.02);
            motor_z.speed(0.01);
        } else if(height > pos) {
            motor_z.speed(-0.1);
            wait(0.02);
            motor_z.speed(-0.05);
        }
    }
    motor_z.speed(0);
}
void getAllMeasurements()
{
    motor_x1.getTicks();
    motor_x2.getTicks();
    motor_y.getTicks();
    height = z_dist;
}

void moveY(int pos)
{
    motor_y.move(pos);
}

void movePiece(char c1, char r1, char c2, char r2)
{
    // c1, c2 should be within 'a' and 'h'
    // subtracting 'a' from c1 or c2 get it's "square offset" from initial position
    // i.e. if c1 = 'b', 'b'-'a'=1 tells it to move the distance of 1 square forward
    // r1, r2 should be within '1' and '9', or ':'
    // same logic, but with '1'
    
    //*** NOTE:
    //When a move is a capture, [c2 r2] will always be [b :]
    int goToY = (c1 - 'a')*YWIDTH;
    int goToX = (r1 - '1')*XWIDTH;
    moveX(goToX);
    moveY(goToY);
    
    // go down to pick up a piece, and go back up
    moveZ(DOWN);
    wait(0.7);
    claw = CLOSE;
    wait(0.5);
    moveZ(UP);

    // move to destination
    goToY = (c2 - 'a')*YWIDTH;
    goToX = (r2 - '1')*XWIDTH;
    moveX(goToX);
    moveY(goToY);
    moveZ(DOWN);
    wait(0.7);
    claw = OPEN;
    wait(0.5);
    moveZ(UP);
}
