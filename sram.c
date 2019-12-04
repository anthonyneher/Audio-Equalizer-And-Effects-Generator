#include "sram.h"

void cs1_high (void){GpioDataRegs.GPCDAT.bit.GPIO67 = 1;}
void cs0_high (void){GpioDataRegs.GPCDAT.bit.GPIO66 = 1;}
void cs1_low (void){GpioDataRegs.GPCDAT.bit.GPIO67 = 0;}
void cs0_low (void){GpioDataRegs.GPCDAT.bit.GPIO66 = 0;}

void init_spi(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_B = 1;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0; // sets the clock to 200MHz

    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;//set CS0 to output
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;//set CS1 to output

    //Change port B and C muxes
    EALLOW;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0x3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0x3;//set MOSI mux
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;//set MISO and clk mux
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;//set MISO and clk mux

    SpibRegs.SPISTS.bit.OVERRUN_FLAG = 1;
    SpibRegs.SPICCR.bit.SPISWRESET = 0;//software reset false
    SpibRegs.SPICCR.all = 0x27;
    SpibRegs.SPICTL.bit.CLK_PHASE = 1;
    SpibRegs.SPICTL.bit.TALK = 1;
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpibRegs.SPIBRR.all = 0x04;

    SpibRegs.SPICCR.bit.SPISWRESET = 1;//reset
    SpibRegs.SPIPRI.bit.FREE = 1;
    Uint16 boiii = SpibRegs.SPIRXBUF;
    cs1_high();
    cs0_high();

}

Uint16 spi_byte(Uint16 data)
{//sends a byte over spi and returns what's shifted into MISO
    SpibRegs.SPITXBUF = (data<<8);
    while(!SpibRegs.SPISTS.bit.INT_FLAG);
    return SpibRegs.SPIRXBUF;
}

void write_buffer(Uint32 addr, Uint16 data){
    sram_write((addr & 0x3FFFF), data);
}

Uint16 read_buffer(Uint32 addr){
    return sram_read(addr & 0x3FFFF);
}

void sram_write(Uint32 address, Uint16 data)
{
    bool above128k = false;
    if(address<0x20000) {
        cs0_low();
        address = address * 2;
    }
    else{
        cs1_low();
        above128k = true;
        address = address - 0x20000;
        address = address * 2;
    }
    spi_byte(0x02);
    spi_byte((char)(address>>16));
    spi_byte((char)(address>>8));
    spi_byte((char)(address));//send address
    spi_byte((char)data);
    spi_byte((char)(data>>8));//little endian
    cs0_high();
    cs1_high();//put both high
}

Uint16 sram_read(Uint32 address)
{
    bool above128k = false;
    if(address<0x20000) {
        cs0_low();
        address = address * 2;
    }
    else{
        cs1_low();
        above128k = true;
        address = address - 0x20000;
        address = address * 2;
    }
    spi_byte(0x03);
    spi_byte((char)(address>>16));
    spi_byte((char)(address>>8));
    spi_byte((char)(address));
    spi_byte(0xFF);//garbage write
    Uint16 data = (spi_byte(0xff) | spi_byte(0xff)<<8);
    cs0_high();
    cs1_high();
    return data;
}

