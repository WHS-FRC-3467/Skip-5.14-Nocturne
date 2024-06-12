// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
import frc.robot.Util.Setpoints.GameState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* 
 * ArmSubsystem - Subsystem to control all Arm motion using a Trapezoidal Profiled PID controller
 * 
 * For more details on how this works, see:
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/profilepid-subsystems-commands.html
 *
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {

    public TunableNumber tuneArmSetpoint = new TunableNumber("Tunable Arm Angle", 0.0);
    
    Debouncer atSetpointDebouncer = new Debouncer(.1,DebounceType.kRising);
    /* Creates a new ArmSubsystem */
    private TalonFX m_armLeader = new TalonFX(CanConstants.ID_ArmLeader);
    private TalonFX m_armFollower = new TalonFX(CanConstants.ID_ArmFollower);

    /* Set up to control the motors by Voltage */
    private VoltageOut m_VoltageOutput = new VoltageOut(0.0);

    /* Neutral output control for disabling the Arm */
    private final NeutralOut m_neutral = new NeutralOut();

    /* Set up the REV Through-Bore encoder as a DutyCycle (Absolute) encoder */
    private DutyCycleEncoder m_encoder = new DutyCycleEncoder(DIOConstants.kArmAbsEncoder);

    /* Feed Forward object to assist Profile Controller */
    private ArmFeedforward m_feedforward = new ArmFeedforward(
            ArmConstants.kS,
            ArmConstants.kG,
            ArmConstants.kV,
            ArmConstants.kA);

    /* Working (current) setpoint, tolerance, and state */
    private double m_armSetpoint;
    private double m_tolerance;
    private GameState m_armState;

    

    private boolean armSteadyAtSetpoint = false;

    /*
     * Constructor
     */
    public ArmSubsystem() {
        super(
                new ProfiledPIDController(
                        ArmConstants.kP,
                        ArmConstants.kI,
                        ArmConstants.kD,
                        new TrapezoidProfile.Constraints(ArmConstants.kArm_MaxVelocity, ArmConstants.kArm_MaxAcceleration)));

        
        // Start arm at rest in STOWED position
        setArmSetpoint(RobotConstants.STOWED);

        // Config Duty Cycle Range for the encoders
        m_encoder.setDutyCycleRange(ArmConstants.kDuty_Cycle_Min, ArmConstants.kDuty_Cycle_Max);
        m_encoder.setDistancePerRotation(360);
        m_encoder.setPositionOffset(ArmConstants.kARM_STARTING_OFFSET/360);

        // Config Motors 
        var armMotorConfiguration = new TalonFXConfiguration();

        // Set the output mode to brake
        armMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set the motor's Neutral Deadband
        armMotorConfiguration.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.kNeutral_Deadband;

        /* Set the turning direction */
        armMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Current Limiting for the arm */
        armMotorConfiguration.CurrentLimits = Constants.MotorCurrentConfigs.kArmCurrentConfig;

        /*
         * Apply the configurations to the motors, and set one to follow the other in
         * the same direction
         */
        m_armLeader.getConfigurator().apply(armMotorConfiguration);
        m_armFollower.getConfigurator().apply(armMotorConfiguration);
        m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));


        // Put controls for the PID controller on the dashboard
        if (RobotConstants.kIsArmTuningMode) {
        SmartDashboard.putData(this.m_controller);    
        }   
    }

    @Override
    public void periodic() {
        

        // This method will be called once per scheduler run

        // Make sure the parent controller gets to do its own updates
        super.periodic();
        
        //TODO: Test if encoder disconnect code works
        if (Robot.isReal()) {
            if (!m_encoder.isConnected()) {
                System.out.println("Arm Encoder error reported in periodic().");
                // Stop the arm and disable the PID
                m_armLeader.setControl(m_neutral);
                this.disable();
            }

        }
        
        // Checks if at setpoint for more then debounce time
        if (getController().atGoal()) {
            armSteadyAtSetpoint = atSetpointDebouncer.calculate(getController().atGoal());
        }
        // Display useful info on the SmartDashboard
        if (Constants.RobotConstants.kIsTuningMode) {
            SmartDashboard.putBoolean("Arm Joint at Setpoint?", getController().atGoal());
            SmartDashboard.putBoolean("Arm Steady?", armSteadyAtSetpoint);
            SmartDashboard.putNumber("Arm Joint Setpoint", getController().getGoal().position);
            SmartDashboard.putNumber("Arm Current Angle", m_encoder.getDistance());
            SmartDashboard.putNumber("Arm Joint Error", getController().getPositionError());
        }
    }

    /**
     * Consumes the output from the ProfiledPIDController.
     * 
     * The PIDSubsystem will automatically call this method from its periodic()
     * block, and pass it the computed output of the control loop.
     * 
     * @param output   the output of the ProfiledPIDController
     * @param setpoint the setpoint state of the ProfiledPIDController
     */
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {

        // Correct the passed-in current setpoint before calculating the feedforward
        double correctedPosition = correctArmJointForFeedFwd(setpoint.position);

        // Calculate the feedforward using the corrected setpoint
        double feedforward = m_feedforward.calculate(correctedPosition, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        m_armLeader.setControl(m_VoltageOutput.withOutput(output + feedforward));
    }

    /**
     * Returns the measurement of the process variable used by the
     * ProfiledPIDController.
     * 
     * The PIDSubsystem will automatically call this method from its periodic()
     * block, and pass the returned value to the control loop.
     * 
     * @return the measurement of the process variable, in this case, the Arm angle,
     *         in degrees corrected to 0.0 at the STOWED position
     */
    @Override
    public double getMeasurement() {
        return m_encoder.getDistance();
    }

    /**
     * Update the PID controller's current Arm setpoint and Tolerance
     * 
     * @param setpoints - the desired position as a Setpoints object
     */
    public void setArmSetpoint(Setpoints setpoints) {

        // Convert degrees to radians and set the profile goal
        m_armSetpoint = MathUtil.clamp(setpoints.arm, ArmConstants.kMIN_ARM_ANGLE, ArmConstants.kMAX_ARM_ANGLE) ;
        m_tolerance = setpoints.tolerance;
        m_armState = setpoints.state;

        getController().setTolerance(m_tolerance);
        setGoal(m_armSetpoint);

        // Display requested Arm State to dashboard
        Setpoints.displayArmState(m_armState);
    }

    /** Override the enable() method so we can set the goal to the current position
     * 
     *  The super method resets the controller and sets its current setpoint to the 
     *  current position, but it does not reset the goal, which will cause the Arm
     *  to jump from the current position to the old goal. 
     */
    @Override
    public void enable() {
        super.enable();
        setGoal(m_encoder.getDistance());
    }

    // Drive the Arm directly by providing a supply voltage value
    public void setArmVoltage(double voltage) {
        super.disable();
        m_armLeader.setControl(m_VoltageOutput.withOutput(voltage));
    }

    // Takes a position in degrees relative to STOWED, and corrects it to be
    // relative to a HORIZONTAL position of zero.
    // This is used for Feedforward only, where we account for gravity using a
    // cosine function
    private double correctArmJointForFeedFwd(double position) {
        return position - (ArmConstants.kARM_HORIZONTAL_OFFSET - ArmConstants.kARM_STARTING_OFFSET);
    }

    public boolean isArmSteady() {
        return armSteadyAtSetpoint;
    }

    /*
     * Command Factories
     */

    // To position for Intake, move Arm to INTAKE position
    public Command prepareForIntakeCommand() {
        return new RunCommand(()-> this.setArmSetpoint(RobotConstants.INTAKE), this)
            .until(()-> getController().atGoal());
    }

    public Command setArmSetpointCommand(Setpoints setpoint) {
        return Commands.runOnce(() -> {
            super.enable();
            setArmSetpoint(setpoint);
        }, this);
    }

    public Command setArmAngleCommand(double angle) {
        return Commands.runOnce(() -> {
            super.enable();
            setGoal(angle);
        }, this);
    }

    public Command disableArmCommand() {
        return Commands.runOnce(() -> {
            this.disable();
            m_armLeader.setControl(m_neutral);
        }, this);
    }

    Trigger encoderDisconnect = new Trigger(()-> !m_encoder.isConnected() && Robot.isReal()).onTrue(disableArmCommand());




}
