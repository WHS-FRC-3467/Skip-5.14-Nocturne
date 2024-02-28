// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    /** Creates a new LEDSubsystem. */
    private static final CANdle m_candle = new CANdle(Constants.CanConstants.LED_CANDLE);

    /*
     * Colors
     */
    class Color {
        int r, g, b;

        private Color(int red, int green, int blue) {
            this.r = red; this.g = green; this.b = blue;
        }
    }

    Color black = new Color(0, 0, 0); // This will Turn off the CANdle
    Color green = new Color(50, 168, 82); // Green
    Color red = new Color(171, 41, 43); // Red
    Color yellow = new Color(107, 107, 199); // Yellow
    Color white = new Color(255, 230, 220); // White
    Color blue = new Color(8, 32, 255); // Blue
    Color purple = new Color(184, 0, 185); // Purple

    /*
     * LED Segments
     */
    class LEDSegment {

        int startIndex;
        int segmentSize;
        int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            m_candle.setLEDs(color.r, color.g, color.b, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            m_candle.animate(animation, animationSlot);
        }

    }
    
    /*
     * Constructor
     */
    public LEDSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(candleConfiguration, 100);

    }

    @Override
    public void periodic() {
    }

    public void setBrightness(double percent) {
        /* Here we will set the brightness of the LEDs */
        m_candle.configBrightnessScalar(percent, 100);
    }

    public void fullClear() {
        clearAnimations();
        disableLEDs();
    }

    public void clearAnimations() {
        m_candle.clearAnimation(m_CANDleMatrix.animationSlot);
        m_candle.clearAnimation(m_TimerStrip.animationSlot);
        m_candle.clearAnimation(m_ShooterStrip.animationSlot);
        m_candle.clearAnimation(m_ArmStrip.animationSlot);
        m_candle.clearAnimation(m_IntakeStrip.animationSlot);
    }

    public void disableLEDs() {
        m_CANDleMatrix.setColor(black);
        m_TimerStrip.setColor(black);
        m_ShooterStrip.setColor(black);
        m_ArmStrip.setColor(black);
        m_IntakeStrip.setColor(black);
    }
    
    /* 
     * CANdle Matrix
     * Aim is locked onto Speaker: Flashing Yellow
     * Otherwise: Off
     */
    LEDSegment m_CANDleMatrix = new LEDSegment(0, 8, 0);
    Animation a_SpeakerLock = new StrobeAnimation(yellow.r, yellow.g, yellow.b, 0, 0.09, m_CANDleMatrix.segmentSize, m_CANDleMatrix.startIndex);
    
    public void lockedOnSpeaker() {
        m_CANDleMatrix.setAnimation(a_SpeakerLock);
    }

    public void noSpeakerLock() {
        m_CANDleMatrix.setColor(black);
    }

    public void CANdleDisabled() {
        m_CANDleMatrix.setColor(black);
    }

    /* 
     * Arm Strip
     * Arm Stowed: Off
     * Arm Moving: Animated Red
     * Arm At Setpoint: Solid Green
     * Disabled: Rainbow
     */
    LEDSegment m_ArmStrip = new LEDSegment(85, 48, 3);
    Animation a_ArmNotReady = new StrobeAnimation(red.r, red.g, red.b, 0, 0.09, m_ArmStrip.segmentSize, m_ArmStrip.startIndex); // Flash
    Animation a_ArmDisabled = new RainbowAnimation(0.7, 0.2, m_ArmStrip.segmentSize, false, m_ArmStrip.startIndex);
    
    public void armStowed() {
        m_ArmStrip.setColor(black);
    }

    public void armNotAtPos() {
        m_ArmStrip.setAnimation(a_ArmNotReady);
    }

    public void armAtPos() {
        m_ArmStrip.setColor(green);
    }

    public void armDisabled() {
        m_ArmStrip.setAnimation(a_ArmDisabled);
    }

     /* 
     * Shooter Strip
     * Shooter Off or Idling: Off
     * Shooter Ramping Up: Animated Red
     * Shooter At Speed: Solid Green
     * Disabled: Rainbow
    */
    LEDSegment m_ShooterStrip = new LEDSegment(37, 48, 2);
    Animation a_notReady2Shoot = new StrobeAnimation(red.r, red.g, red.b, 0, .5, m_ShooterStrip.segmentSize, m_ShooterStrip.startIndex);
    Animation a_ShooterDisabled = new RainbowAnimation(0.7, 0.2, m_ShooterStrip.segmentSize, false, m_ShooterStrip.startIndex);

    public void ready2Shoot() {
        m_ShooterStrip.setColor(green);
    }

    public void notReady2Shoot() {
        m_ShooterStrip.setAnimation(a_notReady2Shoot);
    }

    public void shooterDisabled() {
        m_ShooterStrip.setAnimation(a_ShooterDisabled);
    }

    /* 
     * Intake Strip
     * Intake & Stage Off: Off
     * Intake and Stage Running: Animated Red
     * Note In Stage: Solid Green
     * Disabled: Rainbow
     */
    LEDSegment m_IntakeStrip = new LEDSegment(133, 86, 4);
    Animation a_noNote = new StrobeAnimation(red.r, red.g, red.b, 0, 0.09, m_IntakeStrip.segmentSize, m_IntakeStrip.startIndex);
    Animation a_IntakeDisabled = new RainbowAnimation(0.7, 0.2, m_IntakeStrip.segmentSize, false, m_IntakeStrip.startIndex);

    public void noNote() {
        m_IntakeStrip.setAnimation(a_noNote);
    }

    public void yesNote() {
        m_IntakeStrip.setColor(green);
    }

    public void intakeDisabled() {
        m_IntakeStrip.setColor(black);
    }


    /* Match Timer Strip
    * Autonomous (15 sec): Yellow Ping-pong
    * 2:15 -> 1:00: Solid Green
    * 1:00 -> 0:20: Solid Yellow
    * 0:20 -> 0:10: Solid Red
    * 0:10 -> 0:00: Strobing Red
    * Non-auto periods & Disabled: Off
    */
    LEDSegment m_TimerStrip = new LEDSegment(8, 29, 1);
    Animation a_TimeExpiring = new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_TimerStrip.segmentSize, m_TimerStrip.startIndex);
    Animation a_InAutonomous = new LarsonAnimation(yellow.r, yellow.g, yellow.b, 0, 0.7, m_TimerStrip.segmentSize, BounceMode.Back, m_TimerStrip.startIndex);

    public void runMatchTimerPattern() {

        double matchTime = DriverStation.getMatchTime();

        if (matchTime > 60.0) {
            m_TimerStrip.setColor(green);
        } else if (matchTime > 20.0) {
            m_TimerStrip.setColor(yellow);
        } else if (matchTime > 10.0) {
            m_TimerStrip.setColor(red);
        } else if (matchTime > 0.0) {
            m_TimerStrip.setAnimation(a_TimeExpiring);
        } else {
            m_TimerStrip.setColor(black);
        }
    }

    public void runTimerAuto() {
        m_TimerStrip.setAnimation(a_InAutonomous);
    }

    public void timerDisabled() {
        m_TimerStrip.setColor(black);
    }

    public void runDisabledPatterns() {
        this.CANdleDisabled();
        this.timerDisabled();;
        this.shooterDisabled();
        this.armDisabled();
        this.intakeDisabled();
    }

    public void runAutonomousPatterns() {
        this.CANdleDisabled();;
        this.runTimerAuto();
        this.shooterDisabled();
        this.armDisabled();
        this.intakeDisabled();
    }

}
