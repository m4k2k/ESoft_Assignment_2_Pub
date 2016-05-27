/*******************************************************************************************
*
*    Cyclic Execution based Frequency Measurement ( Embedded Software Assignment 2 )
*
*                   a program that runs a cyclic execution demo
*                       actions are for example
*               frequency measurement and serial logging
*
*
* @author Markus
* @version 1.0
* @lastupdate 02.03.2016 
*                             
*
*******************************************************************************************/

// ########
// INCLUDES
// ########

#include "mbed.h"                                                   // mbed header file
#include "MCP23017.h"                                               // include 16-bit parallel I/O header file
#include "WattBob_TextLCD.h"                                        // include 2*16 character display header file
#include "AnalogIn.h"                                               // include Analog communication
#include <deque>                                                    // used for the avaraging

// #######################
// CONFIGURATION VARIABLES
// #######################

#define SAMPLE_FREQ_US 1
#define TICKRATE_US 12500
#define NYQUIST_CORR 2.002
#define WATCHDOG_PULSE_TIME_MS 5

// ###############################
// ALL PORTS USED ARE DEFINED HERE
// ###############################

#define AnalogPort1 p17                                             // used for Analog Input 1
#define AnalogPort2 p16                                             // used for Analog Input 2
#define FrequencyPort p15                                           // used to measure frequency
#define SwitchPort p5                                               // used for the error code switch
#define WatchPort p6                                                // used for the watchdog pulse
#define OnOffPort p7                                                // used for the on/off switch
#define TaskExecPort p8                                             // used for the task execution pulse


// ####################################
// MISC VARIABLES (shouln't be changed)
// ####################################

MCP23017            *par_port;                                      // pointer to 16-bit parallel I/O object
WattBob_TextLCD     *lcd;                                           // pointer to 2*16 chacater LCD object
Serial              serpc(USBTX, USBRX);                            // serial usb connection tx, rx
Ticker ticker;                                                      // our ticker that runs the main function

DigitalIn dInFreq(FrequencyPort);                                   // Port for the frequency measurement
DigitalIn dInSwitch(SwitchPort);                                    // Port for the switch
DigitalOut dOutWatchd(WatchPort);                                   // Port for WatchDog pulse
DigitalIn dInOnOff(OnOffPort);                                      // Port for the On/Off Switch
DigitalOut dOutTaskExec(TaskExecPort);

std::deque<float> AnalogDB1;                        
std::deque<float> AnalogDB2;

int tick_counter(0);                                                // used for counting the ticks / slots
bool OnOffSwitchStatus(0);                                          // saving the last switch status
int freq(0);                                                        // saving the last frequency status
int error_last(9);                                                  // we init with an impossible error number only 0 and 3 are
                                                                    // possible, using this we force an initialization
                                                                    
float avgAn1(0);                                                    // saving the last avarage of analog 1
float avgAn2(0);                                                    // saving the last avarage of analog 2
bool Switch1(0);                                                    // saving the last switch status

DigitalOut ledd1(LED1);                                             
DigitalOut ledd3(LED3);

// ################################
// FORWARD DECLARATION OF FUNCTIONS
// ################################

void SendPulse(int ms, DigitalOut pn);
void tick();
void printLCD(int& freq,bool& sw, float& avgAn1, float& avgAn2);
void logTime(int TaskNumber);

int MeasureFrequency(DigitalIn& freqIn,int samplingFreqUS);
float ReadAnalogInAVG(std::deque<float>& db,PinName Port);
int CheckError(bool& Switch1,float& avgAn1,float& avgAn2);

/*
========================================================================================
=                                                                                      =
=                       MAIN PROCEDURE - generating the Blocks                         =
=                                                                                      =
========================================================================================
*/
// ############################
// the entrance for the program
// ############################

int main()
{
    serpc.baud(19200);                                              // setup the bautrate
    serpc.printf("Init Software\r\n");
    par_port = new MCP23017(p9, p10, 0x40);                         // initialise 16-bit I/O chip (0x40 = 64)
    lcd = new WattBob_TextLCD(par_port);                            // initialise 2*26 char display
    par_port->write_bit(1,BL_BIT);                                  // turn LCD backlight ON
    lcd->cls();                                                     // clear display
    lcd->locate(0,0);                                               // set cursor to location (0,0) - top left corner
    lcd->printf("f0000 S0");                                        // print labels on the lcd
    lcd->locate(1,0);                                               
    lcd->printf("a1 0.00  a2 0.00");
    ticker.attach_us(&tick, TICKRATE_US);                           // attach the ticker to a procedure
}

// #########################
// the main loop (as ticker)
// #########################
void tick()
{
    // use a timing pulse -> extra digi port -> everytime something happens -> sent pulse
    
    // tickcounter%interval==offset
    if((tick_counter%80)==1) {
        //every 1000 mS
        freq = MeasureFrequency(dInFreq,SAMPLE_FREQ_US);            // measure the frequency

    } else if((tick_counter%24)==2) {
        //every 300 mS, check Switch1 (for error)
        if (dInSwitch)                                              // check the switch status for later use
            Switch1 = true;
        else Switch1 = false;

    } else if((tick_counter%24)==3) {
        // (460 mS) now 300 mS, send watchdog pulse
        SendPulse(WATCHDOG_PULSE_TIME_MS,dOutWatchd);               // send the watchdog pulse

    } else if((tick_counter%32)==4) {
        // every 400 mS
        // EXECUTION SIGNAL!
        SendPulse(10,dOutTaskExec);                                 // sending the execution signal
        avgAn1 = ReadAnalogInAVG(AnalogDB1,AnalogPort1);            // reaning analog values
        avgAn2 = ReadAnalogInAVG(AnalogDB2,AnalogPort2);            // ^^

    } else if((tick_counter%160)==5) {
        // every 2000 Ms
        printLCD(freq,Switch1,avgAn1,avgAn2);                       // print to the lcd screen

    } else if((tick_counter%64)==8) {
        // every 800 mS
        int error_current = CheckError(Switch1,avgAn1,avgAn2);
        if (error_last!=error_current) {                            // we check if the error changed (safe to time )
            if(!error_current) {                                    // we check if its zero
                ledd1=1;
                ledd3=0;
            } else {                                                // else it must be 3
                ledd1=0;
                ledd3=1;
            }
            error_last=error_current;
        }
        
    } else if((tick_counter%400)==9) {
        // every 5000 mS - print to serial
           serpc.printf("%i,%i%,%i,%i\r\n",freq,Switch1,(int)avgAn1,(int)avgAn2);

    } else {
        // in every other slot, check the on off switch
        if(OnOffSwitchStatus!=dInOnOff) {                           // checking the on off switch
            OnOffSwitchStatus = dInOnOff;                           // safe the last switch status
            par_port->write_bit(OnOffSwitchStatus,BL_BIT);          // turn LCD backlight ON/OFF
        }
    }

    tick_counter++;
}

/*
========================================================================================
=                                                                                      =
=                                  FUNCTIONS BLOCK                                     =
=                                                                                      =
========================================================================================
*/

// #########################################################################################
// prints to the lcd, only overwrites values that are necesarry to overwrite (safes some mS)
// #########################################################################################
void printLCD(int& freq,bool& sw, float& avgAn1, float& avgAn2)
{
    lcd->locate(0,1);
    lcd->printf("%04i",freq);                                       // write the frequency to the lcd board
    lcd->locate(0,7);
    lcd->printf("%i",sw);                                           // write the switch value
    lcd->locate(1,3);
    lcd->printf("%.2f",avgAn1);                                     // write avarage analog value 1
    lcd->locate(1,12);
    lcd->printf("%.2f",avgAn2);                                     // write avarage analog value 2
}

// #################################################
// checks the error status and returns it as integer
// #################################################
int CheckError(bool& Switch1,float& avgAn1,float& avgAn2)
{
    if(Switch1 && (avgAn1 > avgAn2))                                // if swicht1 AND averageAnalogValue1 > averageAnalogValue2
        return 3;
    return 0;
}

// ####################################################################
// reads analog values and returns the average over the last 4 readings
// ####################################################################
float ReadAnalogInAVG(std::deque<float>& db,PinName Port)           // (we are using deque because you cant iterate over a queue)
{
    AnalogIn Ain(Port);
    float sum(0);
    if(db.size()>=4)                                                // if we already got 4 values, we have to
        db.pop_front();                                             // make space by deleting the oldest value
    db.push_back(Ain.read());                                       // safe a new reading
    for(deque<float>::const_iterator i = db.begin(); i != db.end(); ++i)
        sum+= *i;                                                   // calculate the average by iterating over the queue
    return sum/db.size();
}

// ##################################################
// used for sending a short pulse for ms Milliseconds
// ##################################################
void SendPulse(int ms, DigitalOut pn)
{
    pn = 1;                                                         // set the analog port high
    wait_ms(ms);                                                    // wait for ms Milliseconds
    pn = 0;                                                         // set the analog port low
}

// ###################################################
// measures the frequency and returns an integer in Hz
// ###################################################
int MeasureFrequency(DigitalIn& freqIn,int samplingFreqUS)
{
    Timer timer;                                                    // timer used to time
    if(!freqIn) {                                                   // if the edge is low
        while(!freqIn)                                              // wait for the edge
            wait_us(samplingFreqUS);                                // let the cpu do nothing meanwhile
        timer.start();                                              // now we can start counting
        while(freqIn)                                               // as long as the freq is high
            wait_us(samplingFreqUS);
    } else {                                                        // else.. it has to be 1
        while(freqIn)                                               // wait for the edge
            wait_us(samplingFreqUS);                                // let the cpu do nothing meanwhile
        timer.start();                                              // now we can start counting
        while(!freqIn)                                              // as long as the frequency is slow
            wait_us(samplingFreqUS);                                // let the cpu do nothing
    }
    timer.stop();                                                   // stop the timer - measuring finished
    return 1000000.0/(timer.read_us()*NYQUIST_CORR);                // Convert to period multiply by 1mil to get freq and correct by Nequ
}
