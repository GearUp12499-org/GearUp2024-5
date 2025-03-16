package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import android.view.View;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.jetbrains.annotations.NotNull;

import java.util.Set;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskTemplate;

public class HSlideProxy extends TaskTemplate {
    public enum Position {
        IN(Hardware.RIGHT_SLIDE_IN),
        OUT(Hardware.RIGHT_SLIDE_OUT),
        TRANSFER(Hardware.RIGHT_SLIDE_TRANSFER),
        KEEP_CLEAR(Hardware.RIGHT_SLIDE_KEEP_CLEAR),
        HOLD(Hardware.RIGHT_SLIDE_HOLD);

        public final int actual;
        Position(int actual) {
            this.actual = actual;
        }
    }

    private static final Set<SharedResource> requires = Set.of(Hardware.Locks.horizontal);
    private static int INSTANCE_COUNT = 0;
    public final SharedResource CONTROL = new SharedResource("HSlideProxy" + (++INSTANCE_COUNT));
    private final Hardware hardware;
    private final Scheduler scheduler = getScheduler();
    public Position positionEn = Position.IN;
    private int positionActual = 0;
    private int positionVirtual = 0;
    private int offset;
    private ITask activeTask = null;

    public HSlideProxy(@NotNull Scheduler scheduler, Hardware hardware, Position start, Position init) {
        super(scheduler);
        this.hardware = hardware;
        positionEn = start;
        positionActual = 0;
        positionVirtual = init.actual;
        offset = init.actual;
        moveTo(start.actual);
        hardware.horizontal.setTargetPosition(0);
        hardware.horizontal.setPower(1.0);
        Log.i("hardware", hardware.horizontal.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());
        // pIDF: 10.0, 0.05, 0, 0 original
        hardware.horizontal.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15.0, 0.25, 0.05, 0, MotorControlAlgorithm.LegacyPID));
        hardware.horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        update();
    }

    public boolean isOut() {
        return positionEn == Position.OUT;
    }

    @Override
    @NotNull
    public Set<SharedResource> requirements() {
        return requires;
    }

    @Override
    public boolean getDaemon() {
        return true;
    }

    @Override
    public void invokeOnStart() {
        update();
    }

    public void update() {
        hardware.horizontal.setTargetPosition(positionActual);
    }

    private void moveTo(int newPos) {
        positionVirtual = newPos;
        positionActual = newPos - offset;
        update();
    }

    public void moveInSync() {
        moveTo(Position.IN.actual);
        positionEn = Position.IN;
    }

    public ITask moveToPreset(Position p) {
        return moveToPreset(p, Hardware.SLIDE_INWARD_TIME);
    }

    public ITask moveToPreset(Position p, double durat) {
        return new TaskTemplate(scheduler) {
            ElapsedTime timer;

            @Override
            public void invokeOnStart() {
                if (positionEn == p) finishEarly();
                if (activeTask != null) activeTask.requestStop();
                activeTask = this;
                moveTo(p.actual);
                positionEn = p;
                timer = new ElapsedTime();
                timer.reset();
            }

            @Override
            public boolean invokeIsCompleted() {
                // FIXME: multiple starting locations et al
                return timer.time() >= durat;
            }
        };
    }

    public ITask moveTransfer() {
        return moveToPreset(Position.TRANSFER);
    }

    public ITask moveIn() {
        return new TaskTemplate(scheduler) {
            ElapsedTime timer;

            @Override
            public void invokeOnStart() {
                if (positionEn == Position.IN) finishEarly();
                if (activeTask != null) activeTask.requestStop();
                activeTask = this;
                positionEn = Position.IN;
                moveInSync();
                timer = new ElapsedTime();
                timer.reset();
            }

            @Override
            public void invokeOnFinish() {
                moveTo(Hardware.RIGHT_SLIDE_IN);
            }

            @Override
            public boolean invokeIsCompleted() {
                // FIXME: multiple starting locations et al
                return timer.time() >= Hardware.SLIDE_INWARD_TIME;
            }
        };
    }

    public void moveOutSync() {
        moveTo(Position.OUT.actual);
        positionEn = Position.OUT;
    }

    public ITask moveOut() {
        return moveToPreset(Position.OUT);
    }
}
