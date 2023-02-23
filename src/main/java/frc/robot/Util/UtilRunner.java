package frc.robot.util;

import java.util.ArrayList;

/** 
 * Extend this class for util classes that need a periodic
 * Make sure UtilRunner.getInstance.run() is being called in RobotPeriodic in Robot.java
 */
public class UtilRunner {
  private static UtilRunner instance;
  private ArrayList<Runnable> m_runnables;

  private UtilRunner() {}

  public static UtilRunner getInstance() {
    if (instance == null) {
      instance = new UtilRunner();
    }
    return instance;
  }

  public void addRunnable(Runnable runnable) {
    m_runnables.add(runnable);
  }

  public void run() {
    for (Runnable runnable : m_runnables) {
      runnable.run();
    }
  }

}
