package frc.robot.Subsystems.Stage;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.StageConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.Util.ThriftyNova;

public class StageSubsystem extends SubsystemBase {

    // Initialize devices
    //TalonSRX m_stageMotor = new WPI_TalonSRX(CanConstants.ID_StageMotor);
    ThriftyNova thrifty_nova = new ThriftyNova(CanConstants.ID_StageMotor);
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.kStageBeamBreak);
    boolean m_noteInStage = false;
    BooleanSupplier m_noteInStageSupplier;
    boolean m_stageRunning = false;

    /** Creates a new StageSubsystem. */
    public StageSubsystem() {

        // Set motor to factory defaults
        //m_stageMotor.configFactoryDefault();

        // Invert motor?
        thrifty_nova.setInverted(false);
        //m_stageMotor.setInverted(true);

        // Set motor to Brake
        thrifty_nova.setBrakeMode(true);
        //m_stageMotor.setNeutralMode(NeutralMode.Brake);

        // Config current limit

        /* Config the peak and nominal outputs */
        //m_stageMotor.configNominalOutputForward(0.0, 30);
        //m_stageMotor.configNominalOutputReverse(0.0, 30);
        //m_stageMotor.configPeakOutputForward(1.0, 30);
        //m_stageMotor.configPeakOutputReverse(1.0, 30);

        // slows unneeded CAN status fames
        //m_stageMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        //m_stageMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        //m_stageMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);

    }

    public void simulationInit() {
        /* If running in Simulation, setup simulated Falcons */
        //PhysicsSim.getInstance().addTalonSRX(m_stageMotor, 1.0, 89975.0);
    }

    @Override
    public void periodic() {

        // Check for change in beam break
        // Sensor returns true when beam NOT broken
        m_noteInStage = m_stageBeamBreak.get() ? false : true;

        SmartDashboard.putBoolean("Note In Stage?", m_noteInStage);

        if (RobotConstants.kIsStageTuningMode) {
            //SmartDashboard.putNumber("Stage Current Draw", m_stageMotor.getSupplyCurrent());
            SmartDashboard.putBoolean("Is Stage Running", isStageRunning());
        }
    }

    public void simulationPeriodic() {
        // If running in simulation, update the sims
        PhysicsSim.getInstance().run();
    }

    /**
     * 
     * @param speed speed to set Stage motor at
     */
    public void runStage(double speed) {
        thrifty_nova.setPercentOutput(speed);
        //m_stageMotor.set(ControlMode.PercentOutput, speed);
        m_stageRunning = true;
        System.out.println("Running Stage");
    }

    public void runStage() {
        thrifty_nova.setPercentOutput(StageConstants.kIntakeSpeed);
        //m_stageMotor.set(ControlMode.PercentOutput, StageConstants.kIntakeSpeed);
        m_stageRunning = true;
        
    }

    public void stopStage() {
        thrifty_nova.setPercentOutput(0.0);
        //m_stageMotor.set(ControlMode.PercentOutput, 0.0);
        m_stageRunning = false;
        System.out.println("Stopping Stage");
    }

    // Do not use if the shooter's target velocity is zero.
    public void ejectFront(double speed) {
        this.runStage(speed);
    }

    public void ejectFront() {
        this.runStage(StageConstants.kFeedToShooterSpeed);
    }

    public void ejectBack(double speed) {
        this.runStage((-1.0) * speed);
    }

    public boolean isNoteInStage() {
        return m_noteInStage;
    }

    public boolean isStageRunning() {
        return m_stageRunning;
    }

    public void checkBeam() {
        if(!isNoteInStage()) {
            ejectFront(0.1);
        } else {
            stopStage();
        }
    }

    /*
     * Command Factories
     */

    // Pass the Note to the Shooter
    public ConditionalCommand feedNote2ShooterCommand() {
        return new ConditionalCommand(feedWithBeam(),feedWithTimeout(),() -> m_noteInStage); 
    }

    public Command feedWithTimeout() {
        return new RunCommand(() -> this.ejectFront(1.0), this)
                .withTimeout(0.5) // run for time
                .andThen(() -> this.stopStage());

    }

    public Command feedWithBeam() {
        return new RunCommand(() -> this.ejectFront(1.0), this)
                .until(() -> !isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage());
    }

    public Command feedStageCommand() {
        return new RunCommand(() -> this.ejectFront(.8), this)
                .until(() -> isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage());
    }

    public Command ejectFrontCommand() {
        return new InstantCommand(() -> this.ejectFront(.8),this);
    }

    public Command stopStageCommand() {
        return new InstantCommand(() -> this.stopStage(),this);
    }

    // Testing
    public Command checkBeamCommand() {
        return new RunCommand(() -> this.ejectFront(0.3), this)
                .until(() -> isNoteInStage()) // run until there is a Note in the Stage
                .andThen(() -> this.stopStage());
    }

}
