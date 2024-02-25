package frc.robot.Util;

import edu.wpi.first.wpilibj.Timer;

public class shootTimer {
    Timer m_timer = new Timer();

    public shootTimer() {
        // Start the timer
        m_timer.start();
    }
    public double giveTime() {
        return m_timer.get();
    }

    public void resetTimer() {
        m_timer.reset();
    }

    public void timerState(boolean isStart) {
        if(isStart) {
            m_timer.start();
        } else {
            m_timer.stop();
        }
    }
}
