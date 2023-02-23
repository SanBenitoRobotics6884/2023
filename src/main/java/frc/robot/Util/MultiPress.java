// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** For double pressing buttons */
public class MultiPress extends UtilBase {
  public static final double DEFAULT_TIME_RANGE = 0.3; // Used if no time range is specified
  public static final int DEFAULT_TIMES = 2;
  // How much time the condition should take to go from
  // false --> true --> false --> true
  private double m_timeRange;
  private double m_firstHitTimestamp;
  private int m_times; // How many hits are needed to return true
  private int m_hits; // The current number of hits in a sequence of hits
  private boolean m_doublePress; // This is true on a double press
  private BooleanChecker m_checker; // Used to check when the condition turns from false to true

  public MultiPress(BooleanSupplier condition) {
    this(DEFAULT_TIMES, condition, DEFAULT_TIME_RANGE);
  }

  public MultiPress(BooleanSupplier condition, double timeRange) {
    this(DEFAULT_TIMES, condition, timeRange);
  }

  public MultiPress(int times, BooleanSupplier condition) {
    this(times, condition, DEFAULT_TIMES);
  }

  public MultiPress(int times, BooleanSupplier condition, double timeRange) {
    m_times = times;
    m_checker = new BooleanChecker(condition);
    m_timeRange = timeRange;
  }

  @Override
  public void run() {
    boolean hitHappened = m_checker.check();
    if (hitHappened && m_hits == m_times - 1 && inTimeRange()) {
      // The first hit has happened and there was a press
      m_doublePress = true;
      m_hits = 0;
    } else {
      m_doublePress = false;
      if (hitHappened && m_hits == 0) {
        // The first hit hasn't happened before but a press happened
        m_hits = 1;
        m_firstHitTimestamp = Timer.getFPGATimestamp();
      } else if (hitHappened && inTimeRange()) {
        m_hits++;
      } else if (inTimeRange()) {
        // The time range has passed since the first hit
        m_hits = 0;
      }
    }
  }

  public boolean get() {
    return m_doublePress;
  }

  public Trigger getTrigger() {
    return new Trigger(() -> m_doublePress);
  }

  private boolean inTimeRange() {
    return Timer.getFPGATimestamp() + 0.020 > m_firstHitTimestamp + m_timeRange;
  }
}
