package org.firstinspires.ftc.teamcode.hardware;

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

        public final double actual;
        Position(double actual) {
            this.actual = actual;
        }
    }

    private static final Set<SharedResource> requires = Set.of(Hardware.Locks.horizontalRight);
    private static int INSTANCE_COUNT = 0;
    public final SharedResource CONTROL = new SharedResource("HSlideProxy" + (++INSTANCE_COUNT));
    private final Hardware hardware;
    private final Scheduler scheduler = getScheduler();
    public Position positionEn = Position.IN;
    private double position = Hardware.RIGHT_SLIDE_IN;
    private ITask activeTask = null;

    public HSlideProxy(@NotNull Scheduler scheduler, Hardware hardware, Position start) {
        super(scheduler);
        this.hardware = hardware;
        positionEn = start;
        position = start.actual;

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
        hardware.horizontalRight.setPosition(position);
        hardware.horizontalLeft.setPosition(1.05 - position);
    }

    private void moveTo(double newPos) {
        position = newPos;
        update();
    }

    public void moveInSync() {
        moveTo(Position.IN.actual);
        positionEn = Position.IN;
    }

    public ITask moveToPreset(Position p) {
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
                return timer.time() >= Hardware.SLIDE_INWARD_TIME;
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
