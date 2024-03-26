// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Vision.PhotonVision;

public class LEDSubsystem extends SubsystemBase {

    // Subsystems to query
    StageSubsystem m_stageSub;
    IntakeSubsystem m_intakeSub;
    ArmSubsystem m_armSub;
    ShooterSubsystem m_shooterSub;
    CommandSwerveDrivetrain m_driveSub;
    PhotonVision m_photonVision;
    
    // Control everything with a CANdle
    private static final CANdle m_candle = new CANdle(Constants.CanConstants.LED_CANDLE);

    /*
     * Robot LED States
     */
    private static enum LEDState {
        START, DISABLED, DISABLED_TARGET, AUTONOMOUS, ENABLED, INTAKING, FEEDING, CLIMBING, HAVENOTE, AIMING, ARM_LOCKED, AIM_LOCKED
    }
    LEDState m_currentState = LEDState.START;

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
            m_candle.modulateVBatOutput(0.95);
        }

        private void setAnimation(Animation animation) {
            m_candle.clearAnimation(animationSlot);
            m_candle.animate(animation, animationSlot);
            m_candle.modulateVBatOutput(0.95);
        }

        public void setOff() {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.0);

        }

    }
    
    /*
     * Constructor
     * Creates a new LEDSubsystem
     */
    public LEDSubsystem(StageSubsystem stageSub,
                        IntakeSubsystem intakeSub,
                        ArmSubsystem armSub,
                        ShooterSubsystem shootSub,
                        CommandSwerveDrivetrain driveSub,
                        PhotonVision photonsub) {
        
        m_stageSub = stageSub;
        m_intakeSub = intakeSub;
        m_armSub = armSub;
        m_shooterSub = shootSub;
        m_driveSub = driveSub;
        m_photonVision = photonsub;

        m_candle.configFactoryDefault();
        
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        m_candle.getAllConfigs(candleConfiguration);
        //System.out.println(candleConfiguration.toString());

        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(candleConfiguration, 100);

        m_candle.getAllConfigs(candleConfiguration);
        //System.out.println(candleConfiguration.toString());

        m_candle.configLEDType(LEDStripType.RGB, 300);

        m_candle.getAllConfigs(candleConfiguration);
        //System.out.println(candleConfiguration.toString());
    }

    @Override
    public void periodic() {

        LEDState newState = LEDState.DISABLED;

        if (DriverStation.isDisabled()) {
            if (m_photonVision.hasTarget()) {
                newState = LEDState.DISABLED_TARGET;
            } else {
                newState = LEDState.DISABLED;
            }            

        } else if (DriverStation.isAutonomousEnabled()) {
            newState = LEDState.AUTONOMOUS;

        } else {
            // If not Disabled or in Auto, run MatchTimer
            runMatchTimerPattern();

            // If Note in Stage
            if (m_stageSub.isNoteInStage()) {

                if (m_armSub.preparing2Feed()) {
                    // Special FEEDing pattern
                    newState = LEDState.FEEDING;
                    
                } else if (m_currentState == LEDState.AIMING) {
                    // Look for proper arm position and shooter speed
                    if (m_armSub.isArmAtSetpoint() && m_shooterSub.isShooterAtSpeed()) {
                        newState = LEDState.ARM_LOCKED;
                    }

                } else if (m_currentState == LEDState.ARM_LOCKED) {
                    // Look for proper alignment
                    if (m_driveSub.isAtFutureAngle()) {
                        newState = LEDState.AIM_LOCKED;
                    }

                } else if (m_armSub.preparing2Shoot()) {
                    // Is arm being setup to shoot?
                        newState = LEDState.AIMING;

                } else {
                    // Just holding Note
                    newState = LEDState.HAVENOTE;
                }
                
            } else if (m_stageSub.isStageRunning() && m_intakeSub.isIntakeRunning()) {
                // If Intake & Stage Running
                newState = LEDState.INTAKING; 

            } else if (m_armSub.preparing2Climb()) {
                // If preparing to Climb
                newState = LEDState.CLIMBING;

            } else {
                // Just Enabled
                newState = LEDState.ENABLED;
            }
        }

        // If State has changed, run the state machine to change LED patterns
        if (newState != m_currentState) {
            LEDStateMachine(newState);
            m_currentState = newState;
        }

    }
    
    private void LEDStateMachine(LEDState newState) {
        
        switch (newState) {
            case DISABLED:
                m_Matrix.setOff();
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == Alliance.Blue) {
                        m_VerticalLeft.setAnimation(a_LeftBlueFlow);
                        m_VerticalRight.setAnimation(a_RightBlueFlow);
                        m_Intake.setAnimation(a_IntakeBlueFlow);
                    } else {
                        m_VerticalLeft.setAnimation(a_LeftRedFlow);
                        m_VerticalRight.setAnimation(a_RightRedFlow);
                        m_Intake.setAnimation(a_IntakeRedFlow);
                    }
                }

                this.timerDisabled();
            break;

        case DISABLED_TARGET:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftRainbow);
            m_VerticalRight.setAnimation(a_RightRainbow);
            m_Intake.setAnimation(a_IntakeRainbow);
            this.timerDisabled();
            break;

        case AUTONOMOUS:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftFlame);
            m_VerticalRight.setAnimation(a_RightFlame);
            m_Intake.setAnimation(a_IntakePingPong);
            m_Timer.setAnimation(a_InAutonomous);
            break;

        case ENABLED:
            m_Matrix.setOff();
            m_VerticalLeft.setColor(white);
            m_VerticalRight.setColor(white);
            m_Intake.setColor(white);
            break;

        case INTAKING:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftStrobe);
            m_VerticalRight.setAnimation(a_RightStrobe);
            m_Intake.setAnimation(a_IntakeStrobe);;
            break;

        case FEEDING:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftPingPong);
            m_VerticalRight.setAnimation(a_RightPingPong);
            m_Intake.setAnimation(a_IntakePingPong);
            break;

        case CLIMBING:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftFlame);
            m_VerticalRight.setAnimation(a_RightFlame);
            m_Intake.setAnimation(a_IntakePingPong);
            break;

        case HAVENOTE:
            m_Matrix.setAnimation(a_MatrixStrobe);
            m_VerticalLeft.setColor(green);
            m_VerticalRight.setColor(green);
            m_Intake.setColor(green);
            break;

        case AIMING:
            m_Matrix.setOff();
            m_VerticalLeft.setColor(blue);
            m_VerticalRight.setColor(blue);
            m_Intake.setColor(blue);
            break;

        case ARM_LOCKED:
            m_Matrix.setOff();
            m_VerticalLeft.setColor(yellow);
            m_VerticalRight.setColor(yellow);
            m_Intake.setColor(yellow);
            break;

        case AIM_LOCKED:
            m_Matrix.setAnimation(a_MatrixStrobe);
            m_VerticalLeft.setColor(magenta);
            m_VerticalRight.setColor(magenta);
            m_Intake.setColor(magenta);
            break;

        default:
            break;
        }
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
        m_candle.clearAnimation(m_VerticalRight.animationSlot);
        m_candle.clearAnimation(m_VerticalLeft.animationSlot);
        m_candle.clearAnimation(m_Intake.animationSlot);
    }

    public void disableLEDs() {
        m_Matrix.setOff();
        m_Timer.setOff();
        m_VerticalRight.setOff();
        m_VerticalLeft.setOff();
        m_Intake.setOff();
    }
    
    LEDSegment m_Matrix = new LEDSegment(0, 8, 0);
    LEDSegment m_VerticalLeft = new LEDSegment(8, 47, 1);
    LEDSegment m_VerticalRight = new LEDSegment(55, 46, 2);
    LEDSegment m_Timer = new LEDSegment(101, 27, 3);
    LEDSegment m_Intake = new LEDSegment(128, 89, 4);

    Animation a_MatrixStrobe = new StrobeAnimation(white.r, white.g, white.b, 0, 0.2, m_Matrix.segmentSize, m_Matrix.startIndex);
   
    Animation a_LeftStrobe = new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_VerticalLeft.segmentSize, m_VerticalLeft.startIndex); // Flash
    Animation a_RightStrobe = new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_VerticalRight.segmentSize, m_VerticalRight.startIndex);
    Animation a_LeftRainbow = new RainbowAnimation(0.7, 0.5, m_VerticalLeft.segmentSize, true, m_VerticalLeft.startIndex);
    Animation a_RightRainbow = new RainbowAnimation(0.7, 0.5, m_VerticalRight.segmentSize, false, m_VerticalRight.startIndex);
    Animation a_LeftFlame = new FireAnimation(0.9, 0.75, m_VerticalLeft.segmentSize, 1.0, 0.3, true, m_VerticalLeft.startIndex);
    Animation a_RightFlame = new FireAnimation(0.9, 0.75, m_VerticalRight.segmentSize, 1.0, 0.3, false, m_VerticalRight.startIndex);
    Animation a_LeftPingPong = new LarsonAnimation(green.r, green.g, green.b, 0, 0.8, m_VerticalLeft.segmentSize, BounceMode.Back, 6, m_VerticalLeft.startIndex);
    Animation a_RightPingPong = new LarsonAnimation(green.r, green.g, green.b, 0, 0.8, m_VerticalRight.segmentSize, BounceMode.Back, 6, m_VerticalRight.startIndex);
    Animation a_LeftRedFlow = new ColorFlowAnimation(red.r, red.g, red.b, 0, 0.7, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightRedFlow = new ColorFlowAnimation(red.r, red.g, red.b, 0, 0.7, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);
    Animation a_LeftBlueFlow = new ColorFlowAnimation(blue.r, blue.g, blue.b, 0, 0.7, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightBlueFlow = new ColorFlowAnimation(blue.r, blue.g, blue.b, 0, 0.7, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);

    Animation a_LeftGreenTwinkle = new ColorFlowAnimation(green.r, green.g, green.b, 0, 0.2, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightGreenTwinkle = new ColorFlowAnimation(green.r, green.g, green.b, 0, 0.2, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);

    Animation a_IntakeStrobe = new StrobeAnimation(red.r, red.g, red.b, 0, 0.09, m_Intake.segmentSize, m_Intake.startIndex);
    Animation a_IntakeRainbow = new RainbowAnimation(0.7, 0.5, m_Intake.segmentSize, false, m_Intake.startIndex);
    Animation a_IntakePingPong = new LarsonAnimation(green.r, green.g, green.b, 0, 0.8, m_Intake.segmentSize, BounceMode.Back, 6, m_Intake.startIndex);
    Animation a_IntakeRedFlow = new ColorFlowAnimation(red.r, red.g, red.b, 0, 0.7, m_Intake.segmentSize, Direction.Forward, m_Intake.startIndex);
    Animation a_IntakeBlueFlow = new ColorFlowAnimation(blue.r, blue.g, blue.b, 0, 0.7, m_Intake.segmentSize, Direction.Forward, m_Intake.startIndex);
    Animation a_IntakeGreenTwinkle = new ColorFlowAnimation(green.r, green.g, green.b, 0, 0.2, m_Intake.segmentSize, Direction.Forward, m_Intake.startIndex);

    Animation a_InAutonomous = new LarsonAnimation(yellow.r, yellow.g, yellow.b, 0, 0.8, m_Timer.segmentSize, BounceMode.Back, 3, m_Timer.startIndex);
    Animation a_TimeExpiring = new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_Timer.segmentSize, m_Timer.startIndex);

   /* Match Timer Strip
    * Autonomous (15 sec): Yellow Ping-pong
    * 2:15 -> 1:00: Solid Green
    * 1:00 -> 0:20: Solid Yellow
    * 0:20 -> 0:10: Solid Red
    * 0:10 -> 0:00: Strobing Red
    * Non-auto periods & Disabled: White
    */
    Timer m_pseudoTimer = new Timer();
    Color currentColor = black;

    private void runMatchTimerPattern() {

        Color newColor = black;

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0.0) {
            m_pseudoTimer.start();            
            matchTime = (int) (150.0 - m_pseudoTimer.get());
        }

        if (matchTime > 60.0) {
            newColor = green;
        } else if (matchTime > 20.0) {
            newColor = yellow;
        } else if (matchTime > 10.0) {
            newColor = red;
        } else if (matchTime > 0.0) {
            newColor = magenta;
        } else {
            newColor = white;
        }

        if (newColor != currentColor) {
            if (newColor == magenta) {
                m_Timer.setAnimation(a_TimeExpiring);
            } else {
                m_Timer.setColor(newColor);
            }
            currentColor = newColor;
        }
    }

    public void timerDisabled() {
        m_Timer.setColor(white);
        m_pseudoTimer.stop();
        m_pseudoTimer.reset();
    }

}
