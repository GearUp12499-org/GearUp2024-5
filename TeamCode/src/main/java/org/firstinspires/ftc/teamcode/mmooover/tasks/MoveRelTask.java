package org.firstinspires.ftc.teamcode.mmooover.tasks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.MMoverDataPack;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;

import dev.aether.collaborative_multitasking.Scheduler;

public class MoveRelTask extends MoveToTask {

    @NotNull
    private final Pose offset;

    public MoveRelTask(
            @NotNull Scheduler scheduler,
            @NotNull MMoverDataPack mmoverData,
            @NotNull Pose offset,
            @NotNull Telemetry telemetry
    ) {
        super(scheduler, mmoverData, new Pose(0, 0, 0), telemetry);
        this.offset = offset;
    }

    @Override
    public void invokeOnStart() {
        Pose current = tracker.getPose();
        target = current.add(offset);
        super.invokeOnStart();
    }
}
