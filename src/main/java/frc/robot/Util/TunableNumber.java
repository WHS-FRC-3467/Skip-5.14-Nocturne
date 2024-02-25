package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotConstants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or value not in dashboard.
 */
public class TunableNumber {
    private static final String m_tableKey = "TunableNumbers";

    private String m_key;
    private double m_defaultValue;
    private boolean m_hasDefault = false;

    /**
     * Create a new TunableNumber
     * 
     * @param dashboardKey Key on dashboard
     */
    public TunableNumber(String dashboardKey) {
        this.m_key = m_tableKey + "/" + dashboardKey;
    }

    /**
     * Create a new TunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        setDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
    */
    public void setDefault(double defaultValue) {
        if (!m_hasDefault) {
            m_hasDefault = true;
            this.m_defaultValue = defaultValue;
            if (RobotConstants.kIsTuningMode) {
                SmartDashboard.putNumber(m_key, SmartDashboard.getNumber(m_key, defaultValue));
            }
        }
    }

    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault() {
        return m_defaultValue;
    }

    /**
     * Publishes a new value. Note that the value will not be returned by {@link #get()} until the next cycle.
     */
    public void set(double value) {
        if (RobotConstants.kIsTuningMode) {
            SmartDashboard.putNumber(m_key, value);
        } else {
            m_defaultValue = value;
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        if (!m_hasDefault) {
            return 0.0;
        } else {
            return RobotConstants.kIsTuningMode ? SmartDashboard.getNumber(m_key, m_defaultValue) : m_defaultValue;
        }
    }
}

