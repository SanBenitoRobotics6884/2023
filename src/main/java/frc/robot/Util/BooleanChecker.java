package frc.robot.util;

import java.util.function.BooleanSupplier;

public class BooleanChecker {
    private boolean m_prevBoolean = false;
    private BooleanSupplier m_condition;

    public BooleanChecker(BooleanSupplier condition) {
        m_condition = condition;
    }

    /** 
     * Returns true when the supplier returns true
     * and the last time this method was called the supplier returned false
     */
    public boolean check() {
        boolean value = m_condition.getAsBoolean();
        if (value && !m_prevBoolean) {
            m_prevBoolean = value;
            return true;
        } else {
            m_prevBoolean = value;
            return false;
        }
    }
}
