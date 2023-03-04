// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.RobotConstants.LED.*;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led = new AddressableLED(LED_PORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LENGTH);
  private LEDColor m_color = LEDColor.OFF;

  private enum LEDColor {
    PURPLE,
    YELLOW,
    OFF;
  }

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < LENGTH; i++) {
      switch (m_color) {
        case PURPLE:
          m_ledBuffer.setLED(i, PURPLE);
          break;
        case YELLOW:
          m_ledBuffer.setLED(i, YELLOW);
          break;
        case OFF:
          m_ledBuffer.setLED(i, OFF);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void putPurple() {
    m_color = LEDColor.PURPLE;
  }

  public void putYellow() {
    m_color = LEDColor.YELLOW;
  }

  public void putOff() {
    m_color = LEDColor.OFF;
  }
}
