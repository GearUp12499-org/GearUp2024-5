package org.firstinspires.ftc.teamcode.mmooover.kinematics

import org.firstinspires.ftc.teamcode.mmooover.Pose
import java.io.ByteArrayOutputStream
import java.io.DataInputStream
import java.io.DataOutputStream
import java.nio.charset.Charset
import kotlin.math.abs

const val EPSILON = 1e-6

/*
 * This file provides minimal interfaces for the various types used for waypoints
 * and in the generated bytecode files, as well as minimal implementations.
 *
 * The intent is that if additional data is needed (e.g. timing) when generating
 * paths, then the interfaces provide the extensibility.
 *
 * The minimal implementations (data class, XYZImpl) are used when *importing*
 * data, because they represent the entirety of the available data.
 */

/**
 * Commands that are supported when 'authoring' the path; used in the path{} function
 * and other generators.
 */
interface AuthoringCommand

/**
 * Commands that can be dumped to / loaded from a file support this interface.
 *
 * @see BytecodeUnit.import
 * @see BytecodeUnit.export
 */
sealed interface BytecodeUnit {
    companion object {
        fun import(target: DataInputStream): BytecodeUnit = when (target.readByte()) {
            Kind.MOVE.bytecodeID -> MoveCommand.import(target)
            Kind.RUN.bytecodeID -> RunCommand.import(target)
            Kind.RUN_ASYNC.bytecodeID -> RunAsyncCommand.import(target)
            Kind.AWAIT.bytecodeID -> AwaitCommand.import(target)
            else -> throw IllegalArgumentException("No matching bytecode type; is the file generated by a compatible version of the robot code?")
        }
    }

    enum class Kind(val bytecodeID: Byte) {
        MOVE(0x00),
        RUN(0x01),
        RUN_ASYNC(0x02),
        AWAIT(0x03);
    }

    val kind: Kind

    fun writeKind(target: DataOutputStream) = target.writeByte(kind.bytecodeID.toInt())
    fun export(target: DataOutputStream)
}

/**
 * Marker interface for things that cause the robot to move.
 * Doesn't do anything on its own, just useful for typing purposes.
 */
interface MotionCommand

/**
 * Command describing the position of the robot.
 */
interface XYCommand : AuthoringCommand, MotionCommand {
    val x: Double
    val y: Double

    infix fun approx(other: Any?) = when(other) {
        null -> false
        !is XYCommand -> false
        else -> abs(other.x - this.x) <= EPSILON && abs(other.y - this.y) <= EPSILON
    }
}
data class XYImpl(override val x: Double, override val y: Double): XYCommand

/**
 * Command describing the rotation of the robot.
 */
interface RCommand : AuthoringCommand, MotionCommand {
    val r: Double

    infix fun approx(other: Any?) = when(other) {
        null -> false
        !is RCommand -> false
        else -> abs(other.r - this.r) <= EPSILON
    }
}
data class RImpl(override val r: Double): RCommand

/**
 * Command describing position and rotation.
 */
interface XYRCommand : XYCommand, RCommand {
    companion object {
        fun zip(xy: XYCommand, r: RCommand) = XYRImpl(xy.x, xy.y, r.r)
    }

    override fun approx(other: Any?): Boolean = when(other) {
        null -> false
        !is XYRCommand -> false
        else -> abs(other.x - this.x) <= EPSILON
                && abs(other.y - this.y) <= EPSILON
                && abs(other.r - this.r) <= EPSILON
    }
}

data class XYRImpl(
    override val x: Double,
    override val y: Double,
    override val r: Double
): XYRCommand

/**
 * Interface describing commands that interact with the 'procedure calling' interface.
 * This means that they hold a 'event name' that the Command is operating on.
 */
interface RPCCommand {
    companion object {
        internal fun <T> importPartial(target: DataInputStream, make: (String) -> T): T {
            val size = target.readInt()
            // Collect <size> elements into a byte array
            val result = ByteArrayOutputStream().use { baos ->
                repeat(size) {
                    baos.write(target.readByte().toInt())
                }
                baos.toByteArray()
            }.toString(Charset.forName("UTF-8"))
            return make(result)
        }
    }

    fun exportPartial(target: DataOutputStream) {
        // size, [...bytes]
        val bytes = eventName.toByteArray(Charset.forName("UTF-8"))
        target.writeInt(bytes.size)
        for (byte in bytes) {
            target.writeByte(byte.toInt())
        }
    }

    val eventName: String
}

/**
 * Run this event 'synchronously' (as in 'wait for process to finish before continuing',
 * not 'block main loop until complete')
 */
interface RunCommand : AuthoringCommand, BytecodeUnit, RPCCommand {
    companion object {
        fun import(target: DataInputStream) = RPCCommand.importPartial(target, ::RunImpl)
    }

    override val kind: BytecodeUnit.Kind
        get() = BytecodeUnit.Kind.RUN

    override fun export(target: DataOutputStream) {
        writeKind(target)
        exportPartial(target)
    }
}
data class RunImpl(override val eventName: String): RunCommand

/**
 * Launch this event to run in parallel with the next sections of the path.
 * @see AwaitCommand
 */
interface RunAsyncCommand : AuthoringCommand, BytecodeUnit, RPCCommand {
    companion object {
        fun import(target: DataInputStream) = RPCCommand.importPartial(target, ::RunAsyncImpl)
    }

    override val kind: BytecodeUnit.Kind
        get() = BytecodeUnit.Kind.RUN_ASYNC

    override fun export(target: DataOutputStream) {
        writeKind(target)
        exportPartial(target)
    }
}
data class RunAsyncImpl(override val eventName: String): RunAsyncCommand

/**
 * Wait for the completion of an event by name. Similar to 'joining' a thread.
 * @see RunAsyncCommand
 */
interface AwaitCommand : AuthoringCommand, BytecodeUnit, RPCCommand {
    companion object {
        fun import(target: DataInputStream) = RPCCommand.importPartial(target, ::AwaitImpl)
    }

    override val kind: BytecodeUnit.Kind
        get() = BytecodeUnit.Kind.AWAIT

    override fun export(target: DataOutputStream) {
        writeKind(target)
        exportPartial(target)
    }
}
data class AwaitImpl(override val eventName: String): AwaitCommand

/**
 * Bytecode-supporting variant of the [XYRCommand] interface.
 */
interface MoveCommand : BytecodeUnit, XYRCommand {
    companion object {
        fun import(target: DataInputStream) = MoveImpl(
            target.readDouble(),
            target.readDouble(),
            target.readDouble()
        )
    }

    override val kind: BytecodeUnit.Kind
        get() = BytecodeUnit.Kind.MOVE

    override fun export(target: DataOutputStream) {
        writeKind(target)
        target.writeDouble(x)
        target.writeDouble(y)
        target.writeDouble(r)
    }

    val pose: Pose
}
data class MoveImpl(
    override val x: Double,
    override val y: Double,
    override val r: Double
): MoveCommand {
    override val pose by lazy { Pose(x, y, r) }
}
