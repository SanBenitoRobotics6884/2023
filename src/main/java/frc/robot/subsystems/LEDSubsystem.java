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
  private enum LEDMode {
    CONFORMIST, // All colors are the same
    INDIVIDUAL; // Set the colors individually
  }
  private enum LEDColor {
    PURPLE,
    YELLOW,
    RED,
    DARK_RED,
    BLUE,
    OFF;
  }
  private final AddressableLED m_led = new AddressableLED(LED_PORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LENGTH);
  private LEDColor m_color = LEDColor.OFF;
  private LEDMode m_mode = LEDMode.CONFORMIST;
  private Color[] m_colors = new Color[LENGTH];

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    for (int i = 0; i < LENGTH; i++) {
      m_colors[i] = new Color(0, 0, 0);
    }
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    if (m_mode == LEDMode.CONFORMIST){
      for (int i = 0; i < LENGTH; i++) {
        switch (m_color) {
          case PURPLE:
            m_ledBuffer.setLED(i, PURPLE);
            break;
          case YELLOW:
            m_ledBuffer.setLED(i, YELLOW);
            break;
          case RED:
            m_ledBuffer.setLED(i, RED);
          case DARK_RED:
            m_ledBuffer.setLED(i, DARK_RED);
          case BLUE:
            m_ledBuffer.setLED(i, BLUE);
          case OFF:
            m_ledBuffer.setLED(i, OFF);
        }
      }
    } else {
      for (int i = 0; i < LENGTH; i++) {
        m_ledBuffer.setLED(i, m_colors[i]);
      }
    }
    
    m_led.setData(m_ledBuffer);
  }

  public double getLength() {
    return m_ledBuffer.getLength();
  }

  public void putPurple() {
    m_color = LEDColor.PURPLE;
  }

  public void putYellow() {
    m_color = LEDColor.YELLOW;
  }
  
  public void putRed() {
    m_color = LEDColor.RED;
  }

  public void putDarkRed() {
    m_color = LEDColor.DARK_RED;
  }

  public void putBlue() {
    m_color = LEDColor.BLUE;
  }

  public void putOff() {
    m_color = LEDColor.OFF;
  }

  public void setLED(int index, Color color) {
    m_colors[index % LENGTH] = color;
  }

  public void setConformistMode() {
    m_mode = LEDMode.CONFORMIST;
  }

  public void setIndividualMode() {
    m_mode = LEDMode.INDIVIDUAL;
  }
}
