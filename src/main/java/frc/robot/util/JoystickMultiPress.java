package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Analogous to the JoystickButton class but for double clicks */
public class JoystickMultiPress extends Trigger {
  public JoystickMultiPress(GenericHID joystick, int buttonNumber) {
    super(new BooleanSupplier() {
      private MultiPress m_multiPress = 
          new MultiPress(() -> joystick.getRawButton(buttonNumber));
      
      @Override
      public boolean getAsBoolean() {
        return m_multiPress.get();
      }
    });
  }

  public JoystickMultiPress(GenericHID joystick, int buttonNumber, double timeRange) {
    super(new BooleanSupplier() {
      private MultiPress m_multiPress = 
          new MultiPress(() -> joystick.getRawButton(buttonNumber), timeRange);
      
      @Override
      public boolean getAsBoolean() {
        return m_multiPress.get();
      }
    });
  }

  public JoystickMultiPress(int times, GenericHID joystick, int buttonNumber) {
    super(new BooleanSupplier() {
      private MultiPress m_multiPress = 
          new MultiPress(times, () -> joystick.getRawButton(buttonNumber));
      
      @Override
      public boolean getAsBoolean() {
        return m_multiPress.get();
      }
    });
  }

  public JoystickMultiPress(int times, GenericHID joystick, int buttonNumber, double timeRange) {
    super(new BooleanSupplier() {
      private MultiPress m_multiPress = 
          new MultiPress(times, () -> joystick.getRawButton(buttonNumber), timeRange);
      
      @Override
      public boolean getAsBoolean() {
        return m_multiPress.get();
      }
    });
  }
}
