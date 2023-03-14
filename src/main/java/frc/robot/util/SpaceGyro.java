package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * A wrapper class for the ADIS16470_IMU for our needs with charge station. 
 */
public class SpaceGyro implements Sendable {
  ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  
  public ADIS16470_IMU getGyro() {
    return m_gyro;
  }

  public void calibrate() {
    m_gyro.calibrate();
  }

  public void reset() {
    m_gyro.reset();
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

  public double getYawRate() {
    return m_gyro.getRate();
  }

  public double getYaw() {
    return m_gyro.getAngle();
  }

  public double getPitch() {
    return Units.radiansToDegrees(
        Math.atan2(getAccelX(), Math.hypot(getAccelY(), getAccelZ())));
  }

  public double getRoll() {
    return Units.radiansToDegrees(
        Math.atan2(getAccelY(), Math.hypot(getAccelX(), getAccelZ())));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Angle", null, null);
    
  }

  
}
