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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.LED.LEDSubsystem.LEDSegment;

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
    Color white = new Color(255, 255, 255);
    Color red = new Color(255, 0, 0);
    Color green = new Color(0, 255, 0);
    Color blue = new Color(0, 0, 255);
    Color yellow = new Color(255, 255, 0);
    Color cyan = new Color(0, 255, 255);
    Color magenta = new Color(255, 0, 255);
//    Color green = new Color(50, 168, 82); // Green
//    Color red = new Color(171, 41, 43); // Red
//    Color yellow = new Color(107, 107, 199); // Yellow
//    Color blue = new Color(8, 32, 255); // Blue
//    Color purple = new Color(184, 0, 185); // Purple


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
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(color.r, color.g, color.b, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.9);
        }

        private void setAnimation(Animation animation) {
            m_candle.clearAnimation(animationSlot);
            m_candle.animate(animation, animationSlot);
            m_candle.modulateVBatOutput(0.9);
        }

        public void setOff() {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.0);

        }

    }
    
    /*
     * Constructor
     */
    public LEDSubsystem() {
        
        m_candle.configFactoryDefault();
        
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        m_candle.getAllConfigs(candleConfiguration);
        System.out.println(candleConfiguration.toString());

        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(candleConfiguration, 100);

        m_candle.getAllConfigs(candleConfiguration);
        System.out.println(candleConfiguration.toString());

        m_candle.configLEDType(LEDStripType.RGB, 300);

        m_candle.getAllConfigs(candleConfiguration);
        System.out.println(candleConfiguration.toString());
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
        m_candle.modulateVBatOutput(0.0);
    }

    public void clearAnimations() {
        m_candle.clearAnimation(m_Matrix.animationSlot);
        m_candle.clearAnimation(m_Timer.animationSlot);
        m_candle.clearAnimation(m_Shooter.animationSlot);
        m_candle.clearAnimation(m_Arm.animationSlot);
        m_candle.clearAnimation(m_Intake.animationSlot);
    }

    public void disableLEDs() {
        m_Matrix.setOff();
        m_Timer.setOff();
        m_Shooter.setOff();
        m_Arm.setOff();
        m_Intake.setOff();
    }
    
    /* 
     * CANdle Matrix
     * Aim is locked onto Speaker: Flashing Yellow
     * Otherwise: Off
     */
    LEDSegment m_Matrix = new LEDSegment(0, 8, 0);
    Animation a_SpeakerLock = new StrobeAnimation(yellow.r, yellow.g, yellow.b, 0, 0.09, m_Matrix.segmentSize, m_Matrix.startIndex);
    
    public void lockedOnSpeaker() {
        m_Matrix.setAnimation(a_SpeakerLock);
    }

    public void noSpeakerLock() {
        m_Matrix.setOff();
    }

    public void CANdleDisabled() {
        m_Matrix.setOff();
    }

    /* 
     * Arm Strip
     * Arm Stowed: Off
     * Arm Moving: Animated Red
     * Arm At Setpoint: Solid Green
     * Disabled: Rainbow
     */
    LEDSegment m_Arm = new LEDSegment(8, 48, 1);
    Animation a_ArmNotReady = new StrobeAnimation(red.r, red.g, red.b, 0, 0.09, m_Arm.segmentSize, m_Arm.startIndex); // Flash
    Animation a_ArmDisabled = new RainbowAnimation(0.7, 0.5, m_Arm.segmentSize, false, m_Arm.startIndex);
    
    public void armStowed() {
        m_Arm.setOff();
    }

    public void armNotAtPos() {
        m_Arm.setAnimation(a_ArmNotReady);
    }

    public void armAtPos() {
        m_Arm.setColor(green);
    }

    public void armDisabled() {
        m_Arm.setAnimation(a_ArmDisabled);
    }

     /* 
     * Shooter Strip
     * Shooter Off or Idling: Off
     * Shooter Ramping Up: Animated Red
     * Shooter At Speed: Solid Green
     * Disabled: Rainbow
    */
    LEDSegment m_Shooter = new LEDSegment(56, 44, 2);
    Animation a_ShooterSpoolUp = new StrobeAnimation(red.r, red.g, red.b, 0, .5, m_Shooter.segmentSize, m_Shooter.startIndex);
    Animation a_ShooterIdle = new LarsonAnimation(red.r, red.g, red.b, 0, 0.1, m_Shooter.segmentSize, BounceMode.Back, m_Shooter.startIndex);
    Animation a_ShooterDisabled = new RainbowAnimation(0.7, 0.5, m_Shooter.segmentSize, false, m_Shooter.startIndex);

    public void ready2Shoot() {
        m_Shooter.setColor(green);
    }

    public void shooterSpoolUp() {
        m_Shooter.setAnimation(a_ShooterSpoolUp);
    }

    public void shooterIdle() {
        m_Shooter.setAnimation(a_ShooterIdle);
    }

    public void shooterOff() {
        m_Shooter.setOff();
    }

    public void shooterDisabled() {
        m_Shooter.setAnimation(a_ShooterDisabled);
    }

    /* 
     * Intake Strip
     * Intake & Stage Off: Off
     * Intake and Stage Running: Animated Red
     * Note In Stage: Solid Green
     * Disabled: Rainbow
     */
    LEDSegment m_Intake = new LEDSegment(128, 86, 4);
    Animation a_noNote = new StrobeAnimation(red.r, red.g, red.b, 0, 0.09, m_Intake.segmentSize, m_Intake.startIndex);
    Animation a_IntakeDisabled = new RainbowAnimation(0.7, 0.5, m_Intake.segmentSize, false, m_Intake.startIndex);

    public void lookingForNote() {
        m_Intake.setAnimation(a_noNote);
    }

    public void yesNoteInStage() {
        m_Intake.setColor(green);
    }

    public void intakeStopped() {
        m_Intake.setOff();
    }

    public void intakeDisabled() {
        m_Intake.setAnimation(a_IntakeDisabled);
    }


    /* Match Timer Strip
    * Autonomous (15 sec): Yellow Ping-pong
    * 2:15 -> 1:00: Solid Green
    * 1:00 -> 0:20: Solid Yellow
    * 0:20 -> 0:10: Solid Red
    * 0:10 -> 0:00: Strobing Red
    * Non-auto periods & Disabled: Off
    */
    LEDSegment m_Timer = new LEDSegment(100, 29, 3);
    Animation a_TimeExpiring = new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_Timer.segmentSize, m_Timer.startIndex);
    Animation a_InAutonomous = new LarsonAnimation(yellow.r, yellow.g, yellow.b, 0, 0.7, m_Timer.segmentSize, BounceMode.Back, m_Timer.startIndex);
    Timer m_pseudoTimer = new Timer();

    public void runMatchTimerPattern() {

        double matchTime = DriverStation.getMatchTime();
        
        if (matchTime < 0.0) {
            m_pseudoTimer.start();            
            matchTime = (int) (150.0 - m_pseudoTimer.get());
        }

        if (matchTime > 60.0) {
            m_Timer.setColor(green);
        } else if (matchTime > 20.0) {
            m_Timer.setColor(yellow);
        } else if (matchTime > 10.0) {
            m_Timer.setColor(red);
        } else if (matchTime > 0.0) {
            m_Timer.setAnimation(a_TimeExpiring);
        } else {
            m_Timer.setColor(black);
        }
    }

    public void runTimerAuto() {
        m_Timer.setAnimation(a_InAutonomous);
    }

    public void timerDisabled() {
        m_Timer.setColor(white);
        m_pseudoTimer.stop();
        m_pseudoTimer.reset();
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

    public void startTeleopPatterns() {
        this.noSpeakerLock();
        this.runMatchTimerPattern();
        this.shooterIdle();
        this.armStowed();
        this.intakeStopped();
    }

}
