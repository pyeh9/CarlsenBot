/* mbed simple H-bridge motor controller
 * Copyright (c) 2007-2010, sford, http://mbed.org
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Motor.h"

#include "mbed.h"
DigitalOut led1(LED1);
int count = 0;

Motor::Motor(PinName pwm, PinName fwd, PinName rev):
        _pwm(pwm), _fwd(fwd), _rev(rev) {

    // Set initial condition of PWM
    _pwm.period(0.001);
    _pwm = 0;

    // Initial condition of output enables
    _fwd = 0;
    _rev = 0;    
}

// Motor with i2c encoder
Motor::Motor(PinName pwm, PinName fwd, PinName rev, I2C *i):
        _pwm(pwm), _fwd(fwd), _rev(rev), _i2c(i) {

    // Set initial condition of PWM
    _pwm.period(0.001);
    _pwm = 0;

    // Initial condition of output enables
    _fwd = 0;
    _rev = 0;
    
    ticks = 0;
    is_reversed = false;
    deviceAddress = 0x60;
}

void Motor::speed(float speed) {
    _fwd = (speed > 0.0);
    _rev = (speed < 0.0);
    _pwm = abs(speed);
}

/* Initialize a motor
 * address - address of i2c encoder on motor
 * ID - ID to assign to the motor
 * term - set this if the motor is the last in its chain
 */
void Motor::init(char address, int ID, bool term){
    char toWrite[] = {0x4D, address}; // writing to 0x4D tells encoder that you want to change its address
    _i2c->write(0x60, toWrite, 2); 
    wait(0.01);
    
		// encoders are daisy-chained i2c devices. 
		// Writing to 0x4B tells the device you mean to put another after it
    if(!term){ 
        char disableTerm = 0x4B;
        _i2c->write(address, &disableTerm, 1);
        wait(0.01);
    }
    deviceAddress = address;
    motorID = ID;
    resetTicks();
    wait(0.01);
}

/* Move a motor to a position. Uses PI control */
void Motor::move(int pos){
    float diff1, spd1, lower = 0.09, upper = 0.1, integral1 = 0.0;
    //int K = 2612, Ki = 0.001;
    int K = 6500, Ki = 0.001;
    while (!(abs(float(ticks - pos)) <= 20))
    {    
        diff1 = (pos - ticks);
        integral1 += (diff1*0.001);
        spd1 = (diff1/K) + (Ki*integral1);      
        //rationalize a speed 
        if (spd1 >= upper) spd1 = upper;
        else if(spd1 <= lower && spd1 > 0.0) spd1 = lower;
        else if(spd1 < 0.0 && spd1 >= -lower) spd1 = -lower;
        else if(spd1 <= -upper) spd1 = -upper;  
        //printf("Motor %d is at position %d, moving with speed %f\n", motorID, ticks, spd1); // debugging
        speed(spd1);
        wait(0.001);
        
    } //while 
    speed(0);
    wait(0.5);
    
		// Adjusting for overshoots
		// TODO: kind of a hack, tweak PID constants and get rid of this
    while (!(abs(float(ticks - pos)) <= 10))
    {    
        diff1 = (pos - ticks);
        spd1 = 0.04;       
        //printf("Motor %d is at position %d, moving with speed %f\n", motorID, ticks, spd1); // debugging
        
				//rationalize a speed 
        if(diff1 < 0) spd1 = -spd1;
        speed(spd1);
        wait(0.001);
        
    }//while 
    speed(0);
    wait(0.8);
    //printf("Done! Motor %d is at position %d\n", motorID, ticks); // debugging
}

void Motor::getTicks(){
    char lsb, msb; 
    char addr[2] = {0x40, 0x41}; // look on datasheet for this
    _i2c->write(deviceAddress, &addr[0], 1);
    _i2c->read(deviceAddress, &msb, 1);
    _i2c->write(deviceAddress, &addr[1], 1);
    _i2c->read(deviceAddress, &lsb, 1);
    ticks = (msb << 8) | lsb;
    if(is_reversed){
        if(ticks != 0) ticks = 65536-ticks;
        else ticks = 0;
    }

		// When moving towards 0, you may overshoot and fill the buffer, and it will count backwards from 2^16 - 1.
		// 50000 is rather arbitrary, it just lets me know I've gone backwards from 0. This isn't a great fix. 
		// TODO: find a better way to tell if I'm at zero after moving backwards
		if(ticks > 50000){  
        ticks = 0;    
    }
}

void Motor::resetTicks(){
    char reset = 0x4A;
    _i2c->write(deviceAddress, &reset, 1);
}



