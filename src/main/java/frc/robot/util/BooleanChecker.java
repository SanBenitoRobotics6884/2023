package frc.robot.util;

import java.util.function.BooleanSupplier;

public class BooleanChecker {
    private boolean m_prevBoolean = false;
    private BooleanSupplier m_booleanSupplier;

    public BooleanChecker(BooleanSupplier booleanSupplier) {
        m_booleanSupplier = booleanSupplier;
    }

    public boolean check() {
        Boolean value = m_booleanSupplier.getAsBoolean();
        if (value && !m_prevBoolean) {
            m_prevBoolean = value;
            return true;
        } else {
            m_prevBoolean = value;
            return false;
        }
    }
}
