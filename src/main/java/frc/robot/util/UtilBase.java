package frc.robot.util;

/** Extend this from a util class for a sort of periodic */
public abstract class UtilBase {
    UtilBase() {
        UtilRunner.getInstance().addRunnable(this::run);
    }

    public abstract void run();
}
