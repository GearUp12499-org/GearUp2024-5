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
         * Line Control Register (read)
         */
        LCR(0x03),

        /**
         * Modem Control Register (read)
         */
        MCR(0x04),
        LSR(0x05),
        MSR(0x06),
        SPR(0x07),
        TCR(0x06),
        TLR(0x07),
        TXLVL(0x08),
        RXLVL(0x09),
        IODir(0x0a),
        IOState(0x0b),
        IOIntEna(0x0c),
        RESERVED1(0x0d),
        IOControl(0x0e),
        EFCR(0x0f),

        DLL(0x00),
        DLH(0x01),

        EFR(0x02),
        XON1(0x04),
        XON2(0x05),
        XOFF1(0x06),
        XOFF2(0x07)
    }

    @Synchronized
    override fun doInitialize(): Boolean {

        TODO()
    }
}