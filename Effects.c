#include <F28x_Project.h>
#include <interrupt.h>
#include <AIC23.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sram.h"


void init_rx_int(void);
interrupt void MCBSPB_Rx_ISR(void);
void init_adc(void);
void init_gpios(void);
void init_adc(void);


float in;
int16_t flag = 0;
int16_t adcvalR = 0;
int16_t adcvalL = 0;
int16_t adcval3 = 0;
uint32_t counter = 0;
uint16_t pins;

volatile float By0, By1, By2, Bx1, Bx2;
volatile float Ba1, Ba2, Bb0, Bb1, Bb2;
volatile float Ty0, Ty1, Ty2, Tx1, Tx2;
volatile float Ta1, Ta2, Tb0, Tb1, Tb2;
volatile float My0, My1, My2, Mx1, Mx2;
volatile float Ma1, Ma2, Mb0, Mb1, Mb2;
volatile float G, Bfc, fs, Q, K, V0, root2, pi;
volatile float Gt, Tfc, Qt, Kt, V0t, root2t;
volatile float Gm, Mfc, Qm, Km, V0m;

volatile long time = 0;
volatile float output, modulator, z, frac, width;
volatile float a = 0.3;
volatile int frequency;
volatile int delay;
volatile int i;

volatile float Wyl0, Wyl1, Wyb0, Wyb1, Wyh, Wq, Wf1, Wfs, Wfc;//floats for da wah

volatile float Pfc, Pc, Pd, Px0, Px1, Px2, Py0, Py1, Py2;

//flags for mode
volatile bool vibrato, tremelo, EQ, wahwah, robot, phaser, autophaser;

uint16_t command;
bool speaker;
int main(void)
 {
    EALLOW;

    InitSysCtrl();
    init_spi();
    init_gpios();
    InitSPIA();
    InitMcBSPb();
    init_rx_int();
    InitBigBangedCodecSPI();
    InitAIC23();
    init_adc();

    Bfc = 250;
    Mfc = 1000;
    Tfc = 4000;
    fs = 48000;
    pi = 3.14159265;
    K = tan((pi * Bfc)/fs);
    Kt = tan((pi * Tfc)/fs);
    Km = tan((pi* Mfc)/fs);
    Q = 1;
    Qm = 3;
    root2 = 1/Q;
    root2t = 1/Q;

    //Wah Wah shi
    Wf1 = 2*sinf((3.141592653*Wfc)/48000.0);
    Wq = 2*0.05;//dampening coefficient


    while(1){
        AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;//start conversion
        while(AdcaRegs.ADCCTL1.bit.ADCBSY);//wait for conversion to finish
        adcvalR = AdcaResultRegs.ADCRESULT0;

        AdcbRegs.ADCSOCFRC1.bit.SOC2 = 1;//start conversion
        while(AdcbRegs.ADCCTL1.bit.ADCBSY);//wait for conversion to finish
        adcvalL = AdcbResultRegs.ADCRESULT2;

        AdccRegs.ADCSOCFRC1.bit.SOC2 = 1;//start conversion
        while(AdccRegs.ADCCTL1.bit.ADCBSY);//wait for conversion to finish
        adcval3 = AdccResultRegs.ADCRESULT2;

        //reading in the position of the dip switches to control which mode is being used
        pins = (GpioDataRegs.GPADAT.all & 0xF);
        pins ^= 0xF;
        if(pins == 0){
            EQ = 1;
            tremelo = 0;
            phaser = 0;
            autophaser = 0;
            wahwah = 0;
            robot = 0;
            vibrato = 0;
        }else if(pins == 1){
            EQ = 0;
            tremelo = 0;
            phaser = 0;
            autophaser = 0;
            wahwah = 0;
            robot = 0;
            vibrato = 1;
        }else if(pins == 2){
            EQ = 0;
            tremelo = 1;
            phaser = 0;
            autophaser = 0;
            wahwah = 0;
            robot = 0;
            vibrato = 0;
        }else if(pins == 3){
            EQ = 0;
            tremelo = 0;
            phaser = 1;
            autophaser = 0;
            wahwah = 0;
            robot = 0;
            vibrato = 0;
        }else if(pins == 4){
            EQ = 0;
            tremelo = 0;
            phaser = 0;
            autophaser = 1;
            wahwah = 0;
            robot = 0;
            vibrato = 0;
        }else if(pins == 5){
            EQ = 0;
            tremelo = 0;
            phaser = 0;
            autophaser = 0;
            wahwah = 1;
            robot = 0;
            vibrato = 0;
        }else if(pins == 6){
            EQ = 0;
            tremelo = 0;
            phaser = 0;
            autophaser = 0;
            wahwah = 0;
            robot = 1;
            vibrato = 0;
        }else{
            EQ = 1;
            tremelo = 0;
            phaser = 0;
            autophaser = 0;
            wahwah = 0;
            robot = 0;
            vibrato = 0;
        }
        //switching between using the aux input and the microphone input
        if(pins & 0x8){
            if(!speaker){
                command = aaudpath();
                BitBangedCodecSpiTransmit(command);
                speaker = true;
                DELAY_US(600);
                command = fullpowerup();
                BitBangedCodecSpiTransmit(command);
            }
        }else{
            if(speaker){
                command = nomicaaudpath();
                BitBangedCodecSpiTransmit(command);
                speaker = false;
                DELAY_US(600);
                command = nomicpowerup();
                BitBangedCodecSpiTransmit(command);
            }
        }

        if(EQ){
//%%%%%%%%%%%%%%%%%%%%
//% BASE BOOST
//%%%%%%%%%%%%%%%%%%%%

            G = (2047.0-((int32_t)adcvalR))*(-0.005859375);

            if(G < 0){
                G *= -1;
                V0 = powf(10,(G/20));

                Bb0 = (1 + root2*K + powf(K,2)) / (V0 + root2*sqrt(V0)*K + powf(K,2));
                Bb1 = (2 * (powf(K,2) - 1) ) / (V0 + root2*sqrt(V0)*K + powf(K,2));
                Bb2 = (1 - root2*K + powf(K,2)) / (V0 + root2*sqrt(V0)*K + powf(K,2));
                Ba1 = (2 * (powf(K,2)/V0 - 1) ) / (1 + root2/sqrt(V0)*K + powf(K,2)/V0);
                Ba2 = (1 - root2/sqrt(V0)*K + powf(K,2)/V0) / (1 + root2/sqrt(V0)*K + powf(K,2)/V0);
            }else{//two different filter coefficients depending on positive or negative gain
                V0 = powf(10,(G/20));

                Bb0 = (1 + sqrt(V0)*root2*K + V0*powf(K,2)) / (1 + root2*K + powf(K,2));
                Bb1 = (2 * (V0*powf(K,2) - 1) ) / (1 + root2*K + powf(K,2));
                Bb2 = (1 - sqrt(V0)*root2*K + V0*powf(K,2)) / (1 + root2*K + powf(K,2));
                Ba1 = (2 * (powf(K,2) - 1) ) / (1 + root2*K + powf(K,2));
                Ba2 = (1 - root2*K + powf(K,2)) / (1 + root2*K + powf(K,2));
            }

//%%%%%%%%%%%%%%%%%%%%
//% MID BOOST
//%%%%%%%%%%%%%%%%%%%%
            Gm = (2047.0-((int32_t)adcval3))*(0.00732421875);
            if(Gm<0){
                Gm *= -1;//flip Gm
                V0m = powf(10,(Gm/20));

                Mb0 = (1 + ((1/Qm)*Km) + powf(Km,2)) / (1 + ((V0m/Qm)*Km) + powf(Km,2));
                Mb1 = (2 * (powf(Km, 2) - 1)) / (1 + ((V0m/Qm)*Km) + powf(Km, 2));
                Mb2 = (1 - ((1/Qm)*Km) + powf(Km, 2)) / (1 + ((V0m/Qm)*Km) + powf(Km, 2));
                Ma1 = Mb1;
                Ma2 =  (1 - ((V0m/Qm)*Km) + powf(Km, 2)) / (1 + ((V0m/Qm)*Km) + powf(Km, 2));
            }else{//two different filter coefficients depending on positive or negative gain
                V0m = powf(10,(Gm/20));

                Mb0 = (1 + ((V0m/Qm)*Km) + powf(Km,2)) / (1 + ((1/Qm)*Km) + powf(Km,2));
                Mb1 = (2 * (powf(Km, 2) - 1)) / (1 + ((1/Qm)*Km) + powf(Km, 2));
                Mb2 = (1 - ((V0m/Qm)*Km) + powf(Km, 2)) / (1 + ((1/Qm)*Km) + powf(Km, 2));
                Ma1 = Mb1;
                Ma2 =  (1 - ((1/Qm)*Km) + powf(Km, 2)) / (1 + ((1/Qm)*Km) + powf(Km, 2));
            }

//%%%%%%%%%%%%%%%%%%%%
//% TREBLE BOOST
//%%%%%%%%%%%%%%%%%%%%
            Gt = (2047.0-((int32_t)adcvalL))*(-0.00732421875);

            if(Gt<0){
                Gt *= -1;
                V0t = powf(10,(Gt/20));

                Tb0 = (1 + root2t*Kt + powf(Kt,2)) / (V0t + root2t*sqrt(V0t)*Kt + powf(Kt,2));
                Tb1 = (2 * (powf(Kt,2) - 1) ) / (V0t + root2t*sqrt(V0t)*Kt + powf(Kt,2));
                Tb2 = (1 - root2t*Kt + powf(Kt,2)) / (V0t + root2t*sqrt(V0t)*Kt + powf(Kt,2));
                Ta1 = (2 * (powf(Kt,2)/V0t - 1) ) / (1 + root2t/sqrt(V0t)*Kt + powf(Kt,2)/V0t);
                Ta2 = (1 - root2t/sqrt(V0t)*Kt + powf(Kt,2)/V0t) / (1 + root2t/sqrt(V0t)*Kt + powf(Kt,2)/V0t);
            }else{//two different filter coefficients depending on positive or negative gain
                V0t = powf(10,(Gt/20));

                Tb0 = (V0t + root2t*sqrt(V0t)*Kt + powf(Kt,2)) / (1 + root2t*Kt + powf(Kt,2));
                Tb1 = (2 * (powf(Kt,2) - V0t) ) / (1 + root2t*Kt + powf(Kt,2));
                Tb2 = (V0t - root2t*sqrt(V0t)*Kt + powf(Kt,2)) / (1 + root2t*Kt + powf(Kt,2));
                Ta1 = (2 * (powf(Kt,2) - 1) ) / (1 + root2t*Kt + powf(Kt,2));
                Ta2 = (1 - root2t*Kt + powf(Kt,2)) / (1 + root2t*Kt + powf(Kt,2));
            }

        }else if(vibrato){
        //%%%%%%%%%%%%%%%%%%%%
        //% VIBE Check that brado
        //%%%%%%%%%%%%%%%%%%%%
                a = 0.5;
                delay = 360;
        }
    }
}

void init_gpios(void){
    GpioCtrlRegs.GPADIR.all &= ~(0xF);//set to input
    GpioCtrlRegs.GPAPUD.all &= ~(0xF);//set to pullup
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;
    GpioDataRegs.GPADAT.bit.GPIO22 = 0;
}


void init_rx_int(void){
    DINT;
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;//enable
    PieCtrlRegs.PIEIER6.bit.INTx7 = 1;
    // Enable Receive Interrupt
    McbspbRegs.MFFINT.bit.RINT = 1;
    Interrupt_enable(INT_MCBSPB_RX);
    Interrupt_register(INT_MCBSPB_RX, &MCBSPB_Rx_ISR);
    IER |= M_INT6;
    EINT;
}

void init_adc(void)
{
    EALLOW;
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;// single ended conversion
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;//no pre-scaler
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;// all analog circuitry in core is powfered on
    DELAY_US(1000);//allow ADC system time to boot up
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 8;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0;//triggered by software only
//initializing channel 2 ADC B
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;// single ended conversion
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;//no pre-scaler
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;// all analog circuitry in core is powfered on
    DELAY_US(1000);//allow ADC system time to boot up
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 8;
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0;//triggered by software only
//initializing channel 2 ADC C
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;// single ended conversion
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;//no pre-scaler
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;// all analog circuitry in core is powfered on
    DELAY_US(1000);//allow ADC system time to boot up
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 8;
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0;//triggered by software only
}

interrupt void MCBSPB_Rx_ISR(void)
{
    //input
    in = (float)(((int32_t)((int16_t)McbspbRegs.DRR1.all) + (int32_t)((int16_t)McbspbRegs.DRR2.all))/2);

    if(EQ){
        /********************EQUILIZER******************************/
        //First stage IIR filter for bass
        By0 = in*Bb0 + Bx1*Bb1 + Bx2*Bb2 - By1*Ba1 - By2*Ba2;
        By2 = By1;
        By1 = By0;
        Bx2 = Bx1;
        Bx1 = in;
        //Second stage IIR filter for mids
        My0 = By0*Mb0 + Mx1*Mb1 + Mx2*Mb2 - My1*Ma1 - My2*Ma2;
        My2 = My1;
        My1 = My0;
        Mx2 = Mx1;
        Mx1 = By0;
        //Third stage IIR filter for treble
        Ty0 = My0*Tb0 + Tx1*Tb1 + Tx2*Tb2 - Ty1*Ta1 - Ty2*Ta2;
        Ty2 = Ty1;
        Ty1 = Ty0;
        Tx2 = Tx1;
        Tx1 = My0;

        McbspbRegs.DXR1.all = (int16_t)Ty0;
        McbspbRegs.DXR2.all = (int16_t)Ty0;

    }else if(tremelo){

        /****************TREMELO*******************/
        //this effect is accomplished by oscillating the amplitude of the output and adjusting the oscillation speed
        output = (((float)adcvalR)/4095.0)*cosf(((float)(time++))*0.00013089969*floor(1+(4*(((float)adcvalL)/4095.0))));
        if(time == 48000) time = 0;
        output = (1+output)*in;
        McbspbRegs.DXR1.all = (int16_t)output;
        McbspbRegs.DXR2.all = (int16_t)output;

    }else if(vibrato){
        /******************VIBErato******************/
        //Vibrato utilizes an oscilating delay with variable delay depth
        frequency = (5+(int)(9*(((float)adcvalR)/4095.0)));
        modulator = sinf(((float)time)*0.00013089969*((float)frequency));
        width = 120*((float)adcvalL)/4095.0;
        z=1+delay+width*modulator;
        i=floor(z);
        frac=z-i;

        //linear interpolation is used for delay times that are not integer multiples of the sampling period
        output = ((float)((int16_t)read_buffer(counter-i-1)))*frac+((float)((int16_t)read_buffer(counter-i)))*(1-frac);

        write_buffer(++counter, (int16_t)in);
        if(time++ == 48000) time = 0;

        McbspbRegs.DXR1.all = (int16_t)output;
        McbspbRegs.DXR2.all = (int16_t)output;

    }else if(robot){
        /******************MR ROBOTO******************/
        //this effect uses a modified vibrato to emulate a robot voice
        frequency = (80+(int)(10*(((float)adcvalR)/4095.0)));
        modulator = sinf(((float)time)*0.00013089969*((float)frequency));

        width = 120*((float)adcvalL)/4095.0;

        z=1+delay+width*modulator;
        i=floor(z);
        frac=z-i;

        //linear interpolation
        output = ((float)((int16_t)read_buffer(counter-i-1)))*frac+((float)((int16_t)read_buffer(counter-i)))*(1-frac);

        write_buffer(++counter, (int16_t)in);
        if(time++ == 48000) time = 0;

        McbspbRegs.DXR1.all = (int16_t)output;
        McbspbRegs.DXR2.all = (int16_t)output;
    }else if(wahwah){
        /******************WAHWAH******************/

        if(adcvalL > 2048){
            frequency = floor(1+(8*(((float)adcvalR)/4095.0)));
            Wfc = 2750 + (sinf(((float)time)*0.00013089969* frequency) * 2250);//center frequency ranges between 500 and 3000
            if(time++ == 48000) time = 0;

        }else{
            Wfc = 500 + ((((float)adcvalR)/4095.0) * 2500);//center frequency ranges between 500 and 3000
        }
        Wyh = in - Wyl1 - Wq*Wyb1;
        Wyb0 = Wf1*Wyh + Wyb1;
        Wyl0 = Wf1*Wyb0 + Wyl1;
        Wf1 = 2*sinf((3.141592653*Wfc)/48000.0);

        Wyb1 = Wyb0;
        Wyl1 = Wyl0;

        McbspbRegs.DXR1.all = (int16_t)Wyb0;
        McbspbRegs.DXR2.all = (int16_t)Wyb0;

    }else if(phaser){
        /******************PHASER******************/
        Pfc = (500 + 2500*(((float)adcvalR)/4095.0));

        Pc = (tanf(0.00013089969*Pfc) - 1)/(tanf(0.00013089969*Pfc) + 1);
        Pd = -cosf(0.00013089969*Pfc);

        Py0 = -Pc*in + Pd*(1-Pc)*Px1 + Px2 - Pd*(1-Pc)*Py1 + Pc*Py2;
        output  = (in - Py0)/2;
        //update time delayed values;
        Py1 = Py0;
        Py2 = Py1;
        Px1 = in;
        Px2 = Px1;

        McbspbRegs.DXR1.all = (int16_t)output;
        McbspbRegs.DXR2.all = (int16_t)output;

    }else if(autophaser){
        /******************AUTO PHASER******************/
        Pfc = 500 + 2500*sinf( ((float)(time++)) * 0.00013089969 * (1+floor(5*(((float)adcvalR)/4095.0))) );

        Pc = (tanf(0.00013089969*Pfc) - 1)/(tanf(0.00013089969*Pfc) + 1);
        Pd = -cosf(0.00013089969*Pfc);

        Py0 = -Pc*in + Pd*(1-Pc)*Px1 + Px2 - Pd*(1-Pc)*Py1 + Pc*Py2;
        output  = (in - Py0)/2;

        //update time delayed values;
        Py1 = Py0;
        Py2 = Py1;
        Px1 = Px0;
        Px2 = Px1;

        if(time == 48000) time = 0;

        McbspbRegs.DXR1.all = (int16_t)output;
        McbspbRegs.DXR2.all = (int16_t)output;

    }



    PieCtrlRegs.PIEACK.bit.ACK6 = 1;
}

