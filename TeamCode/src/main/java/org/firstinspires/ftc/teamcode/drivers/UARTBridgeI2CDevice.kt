package org.firstinspires.ftc.teamcode.drivers

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType

@I2cDeviceType
@DeviceProperties(name = "UART bridge device", xmlTag = "GEAR_UP_UART_I2C_DEVICE")
class UARTBridgeI2CDevice(deviceClient: I2cDeviceSynch, deviceClientIsOwned: Boolean) :
    I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, deviceClientIsOwned), HardwareDevice {
    override fun getManufacturer() = HardwareDevice.Manufacturer.Other

    override fun getDeviceName() = "SandboxElectronics_SC16IS750_I2C_UART_Bridge"

    private class InterruptEnable {
        var rxDataAvailable: Boolean = false
        var thrEmpty: Boolean = false
        var receiveStatus: Boolean = false
        var modemStatus: Boolean = false
        var sleep: Boolean = false
        var xOff: Boolean = false
        var rtsInterrupt: Boolean = false
        var ctsInterrupt: Boolean = false
    }

    enum class Register(val byteN: Byte) {
        /**
         * Receive Holding Register (read only)
         */
        RHR(0x00),

        /**
         * Transmit Holding Register (write only)
         */
        THR(0x00),

        /**
         * Interrupt Enable Register (read/write)
         */
        IER(0x01),

        /**
         * Interrupt Identification Register (read)
         * Do not burst-read
         */
        IIR(0x02),

        /**
         * FIFO Control Register (write)
         */
        FCR(0x02),

        /**
         * Line Control Register (read/write)
         */
        LCR(0x03),

        /**
         * Modem Control Register (read/write)
         */
        MCR(0x04),

        /**
         * Line Status Register (read)
         */
        LSR(0x05),

        /**
         * Modem Status Register (read)
         */
        MSR(0x06),

        /**
         * Scratchpad Register (read/write)
         */
        SPR(0x07),

        /**
         * Transmission Control Register (read/write)
         */
        TCR(0x06),

        /**
         * Trigger Level Register (read/write)
         */
        TLR(0x07),

        /**
         * Transmit FIFO Level Register (read)
         */
        TXLVL(0x08),

        /**
         * Receive FIFO Level Register (read)
         */
        RXLVL(0x09),

        /**
         * I/O pin Direction Register (read/write)
         */
        IODir(0x0a),

        /**
         * I/O pin States Register (read)
         */
        IOState(0x0b),

        /**
         * I/O Interrupt Enable Register (read/write)
         */
        IOIntEna(0x0c),
        RESERVED1(0x0d),

        /**
         * I/O pins Control Register (read/write)
         */
        IOControl(0x0e),

        /**
         * Extra Features Register (read/write)
         */
        EFCR(0x0f),

        /**
         * Divisor latch LSB (read/write)
         */
        DLL(0x00),
        /**
         * Divisor latch MSB (read/write)
         */
        DLH(0x01),

        /**
         * Enhanced Feature Register (read/write)
         */
        EFR(0x02),
        /**
         * Xon1 word (read/write)
         */
        XON1(0x04),
        /**
         * Xon2 word (read/write)
         */
        XON2(0x05),
        /**
         * Xoff1 word (read/write)
         */
        XOFF1(0x06),
        /**
         * Xoff2 word (read/write)
         */
        XOFF2(0x07)
    }

    @Synchronized
    override fun doInitialize(): Boolean {

        TODO()
    }
}