package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * A wrapper class for the ADIS16470_IMU for our needs with charge station. 
 */
public class SpaceGyro implements Sendable {
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private boolean hasCalibrated = false;
  private boolean hasReset = false;
  
  public ADIS16470_IMU getGyro() {
    return m_gyro;
  }

  public void calibrate() {
    m_gyro.calibrate();
    hasCalibrated = true;
  }

  public void reset() {
    m_gyro.reset();
    hasReset = true;
  }

  public boolean hasCalibrated() {
    return hasCalibrated;
  }

  public boolean hasReset() {
    return hasReset;
  }

  public double getAccelX() {
    return m_gyro.getAccelX();
  }

  public double getAccelY() {
    return m_gyro.getAccelY();
  }

  public double getAccelZ() {
    return m_gyro.getAccelZ();
  }

  public double getNetAccel() {
    return Math.sqrt(
        Math.pow(getAccelX(), 2)
        + Math.pow(getAccelY(), 2)
        + Math.pow(getAccelZ(), 2));
  }

  public double getYawRate() {
    return m_gyro.getRate();
  }

  public double getYaw() {
    return m_gyro.getAngle();
  }

  /** Unlikely to be accurate if robot is accelerating (uses gravity) */
  public double getPitch() {
    return Units.radiansToDegrees(
        Math.atan2(getAccelX(), Math.hypot(getAccelY(), getAccelZ())));
  }

  /** Unlikely to be accurate if robot is accelerating (uses gravity) */
  public double getRoll() {
    return Units.radiansToDegrees(
        Math.atan2(getAccelY(), Math.hypot(getAccelX(), getAccelZ())));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Yaw", this::getYaw, null);
    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Roll", this::getRoll, null);
    builder.addDoubleProperty("AccX", this::getAccelX, null);
    builder.addDoubleProperty("AccY", this::getAccelY, null);
    builder.addDoubleProperty("AccZ", this::getAccelZ, null);
    builder.addDoubleProperty("Net", this::getNetAccel, null);
    builder.addBooleanProperty("Has Calibrated", this::hasReset, null);
    builder.addBooleanProperty("Has Reset", this::hasReset, null);
  }
}
