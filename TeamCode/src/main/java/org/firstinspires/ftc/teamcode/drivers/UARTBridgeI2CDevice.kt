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

    private var ier: Byte = 0

    private object IER {
        const val RX_DATA_AVAILABLE_INT = 1 shl 0
        const val THR_EMPTY_INT = 1 shl 1
        const val RECV_LINE_STATUS_INT = 1 shl 2
        const val MODEM_STATUS_INT = 1 shl 3
        const val SLEEP_MODE = 1 shl 4
        const val XOFF = 1 shl 5
        const val RTS_INT = 1 shl 6
        const val CTS_INT = 1 shl 7
    }

    private object FCR {
        enum class RXTriggerN(val regVal: Int) {
            C8(0b00),
            C16(0b01),
            C56(0b10),
            C60(0b11),
        }

        enum class TXTriggerN(val regVal: Int) {
            S8(0b00),
            S16(0b01),
            S32(0b10),
            S56(0b11),
        }

        fun setTriggerLevel(rx: RXTriggerN) = rx.regVal shl 6
        fun setTriggerLevel(tx: TXTriggerN) = tx.regVal shl 4

        const val FIFO_EN = 1 shl 0
        const val RX_FIFO_RESET = 1 shl 1
        const val TX_FIFO_RESET = 1 shl 2
        const val TX_TRIGGER_LEVEL_LSB = 1 shl 4
        const val TX_TRIGGER_LEVEL_MSB = 1 shl 5
        const val RX_TRIGGER_LEVEL_LSB = 1 shl 6
        const val RX_TRIGGER_LEVEL_MSB = 1 shl 7
    }

    class IIR(private val value: Int) {
        enum class IntSource(val priorityValue: Int) {
            RECV_LINE_STATUS_ERR(0b00011),
            RECV_TIMEOUT_INT(0b00110),
            RHR_INT(0b00010),
            THR_INT(0b00001),
            MODEM_INT(0b00000),
            INPUT_PIN_CHANGED(0b11000),
            XOFF_SIGNAL_OR_SPECIAL_CHAR(0b01000),
            CTS_RTS_DEACTIVATE_EDGE(0b10000)
        }

        companion object {
            const val INT_STATUS = 1 shl 0
            const val FIFO_EN1 = 1 shl 6
            const val FIFO_EN2 = 1 shl 7

            const val PRIORITY_MASK = 0b00111110

            fun interruptPriority(priority: Int): Int {
                if (priority > 0b11111) throw IllegalArgumentException("priority is 0 to 31")
                return priority shl 1
            }
        }

        val isInterruptPending = (value and INT_STATUS) == 0
        val isFIFOEnabled = (value and FIFO_EN1) > 0
        val interrupt: IntSource

        init {
            val prioValue = (value and PRIORITY_MASK) shr 1
            var result: IntSource? = null
            for (source in IntSource.entries) {
                if (prioValue == source.priorityValue) {
                    result = source
                    break
                }
            }
            interrupt = result
                ?: throw IllegalArgumentException("IIR register doesn't match any known interrupt")
        }
    }

    private object LCR {
        const val STOP_BIT = 1 shl 2
        const val PARITY_EN = 1 shl 3
        const val PARITY_TYPE = 1 shl 4 // odd = 0; even = 1
        const val SET_PARITY_BIT = 1 shl 5
        const val BREAK_CONTROL_BIT = 1 shl 6
        const val DIVISOR_LATCH_EN = 1 shl 7
    }

    enum class Register(val byteN: Int) {
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