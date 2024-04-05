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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

/* 
 * ArmSubsystem - Subsystem to control all Arm motion using a Trapezoidal Profiled PID controller
 * 
 * For more details on how this works, see:
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/profilepid-subsystems-commands.html
 *
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {

    private static TunableNumber tuneArmSetpoint = new TunableNumber("Tunable Arm Setpoint", 0.0);
    

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
            Constants.ArmConstants.kS,
            Constants.ArmConstants.kG,
            Constants.ArmConstants.kV,
            Constants.ArmConstants.kA);

    /* Create our own TP State object so a new one is not created on every setGoal() call  */
    private TrapezoidProfile.State m_tpState = new TrapezoidProfile.State(0.0, 0.0);

    /* Working (current) setpoint */
    private double m_armSetpoint;

    /* Working (current) tolerance */
    private double m_tolerance;

    /* Working (currently requested) arm game state */
    private GameState m_armState;

    /* Current Arm Action */
    private enum armAction {
        kSTOWED,
        kAIMING,
        kONPOINT
    }
    private armAction m_armAction = armAction.kSTOWED;

    TunableNumber tempDegree = new TunableNumber("Set Arm To Degrees", 0.0);
    
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    private int scansAtPos = 0;
    private boolean armSteadyAtSetpoint = false;

    /*
     * Constructor
     */
    public ArmSubsystem() {
        

        /* Create the Trapezoidal motion profile controller */
        super(
                new ProfiledPIDController(
                        ArmConstants.kP,
                        ArmConstants.kI,
                        ArmConstants.kD,
                        new TrapezoidProfile.Constraints(ArmConstants.kArm_Cruise, ArmConstants.kArm_Acceleration)));

        // Start arm at rest in STOWED position
        updateArmSetpoint(RobotConstants.STOWED);

        // Config Duty Cycle Range for the encoders
        m_encoder.setDutyCycleRange(ArmConstants.kDuty_Cycle_Min, ArmConstants.kDuty_Cycle_Max);

        // Config Motors
        var leadConfiguration = new TalonFXConfiguration();
        var followerConfiguration = new TalonFXConfiguration();
        
        // Set the output mode to brake
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set the motor's Neutral Deadband
        leadConfiguration.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.kNeutral_Deadband;
        followerConfiguration.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.kNeutral_Deadband;

        /* Set the turning direction */
        leadConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Current Limiting for the arm */
        m_currentLimits.SupplyCurrentLimit = 30; // Limit to 30 amps
        m_currentLimits.SupplyCurrentThreshold = 50; // If we exceed 50 amps
        m_currentLimits.SupplyTimeThreshold = 0.1; // For at least 0.1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 60; // Limit stator to 60 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        leadConfiguration.CurrentLimits = m_currentLimits;
        followerConfiguration.CurrentLimits = m_currentLimits;

        /*
         * Apply the configurations to the motors, and set one to follow the other in
         * the same direction
         */
        m_armLeader.getConfigurator().apply(leadConfiguration);
        m_armFollower.getConfigurator().apply(followerConfiguration);
        m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));

        // optimize StatusSignal rates for the Talons
        //m_armLeader.getSupplyVoltage().setUpdateFrequency(4);
        //m_armLeader.optimizeBusUtilization();
        //m_armFollower.getSupplyVoltage().setUpdateFrequency(4);
        //m_armFollower.optimizeBusUtilization();


        // Put controls for the PID controller on the dashboard
        if (RobotConstants.kIsArmTuningMode) {
        SmartDashboard.putData(this.m_controller);    
        SmartDashboard.putData("Arm Coast Command", armCoastCommand());    
        SmartDashboard.putData("Arm Brake Command", armBrakeCommand()); 
        }   
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run

        // Make sure the parent controller gets to do its own updates
        super.periodic();

        boolean atSetpoint = isArmJointAtSetpoint();
        
        // Validate current encoder reading; stop motors if out of range
        double armPos = getJointPosAbsolute();
        //TODO: Test if encoder disconnect code works
        if (Robot.isReal()) {
            if (!m_encoder.isConnected() || (armPos < 0.1 || armPos >= 1.0)) {
                System.out.println("Arm Encoder error reported in periodic().");
                System.out.println(armPos);
                // Stop the arm and disable the PID
                neutralOutput();
                this.disable();
            }

        }
        
        // Arm Action logic
        if (m_armState == GameState.STOWED) {
            m_armAction = armAction.kSTOWED;

        } else if (atSetpoint) {
            m_armAction = armAction.kONPOINT;            
        }

        if (atSetpoint) {
            if (scansAtPos < 6) {
                scansAtPos += 1;
            } else {
                armSteadyAtSetpoint = true;
            }
        } else {
            scansAtPos = 0;
            armSteadyAtSetpoint = false;
        }
        // Display useful info on the SmartDashboard
        if (Constants.RobotConstants.kIsTuningMode) {
            SmartDashboard.putBoolean("Arm Joint at Setpoint?", atSetpoint);
            SmartDashboard.putBoolean("Arm at Setpoint?", isArmAtSetpoint());
            SmartDashboard.putBoolean("Arm Steady?", armSteadyAtSetpoint);
            SmartDashboard.putNumber("Count", scansAtPos);
            SmartDashboard.putNumber("Arm Joint Setpoint", m_armSetpoint);
            SmartDashboard.putNumber("Raw Arm Encoder ", getJointPosAbsolute());
            SmartDashboard.putNumber("Arm Angle Uncorrected", dutyCycleToDegrees(getJointPosAbsolute()));
            SmartDashboard.putNumber("Arm Current Angle", getArmJointDegrees());
            SmartDashboard.putNumber("Arm Joint Error", getArmJointError());
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
        double correctedPosition = correctArmJointRadiansForFeedFwd(setpoint.position);

        // Calculate the feedforward using the corrected setpoint
        double feedforward = m_feedforward.calculate(correctedPosition, setpoint.velocity);

        if (Constants.RobotConstants.kIsArmTuningMode) {
            SmartDashboard.putNumber("Arm corrected FF position", correctedPosition);
            SmartDashboard.putNumber("Arm PID output", output);
            SmartDashboard.putNumber("Arm Feed Forward Output", feedforward);
        }

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
     *         in radians corrected to 0.0 at the STOWED position
     */
    @Override
    public double getMeasurement() {
        return getArmJointRadians();
    }

    /**
     * Update the PID controller's current Arm setpoint and Tolerance
     * 
     * @param setpoints - the desired position as a Setpoints object
     */
    public void updateArmSetpoint(Setpoints setpoints) {

        // Convert degrees to radians and set the profile goal
        m_armSetpoint = setpoints.arm;
        m_tolerance = setpoints.tolerance;
        m_armState = setpoints.state;
        m_armAction = armAction.kAIMING;

        // Arm setpoint must be passed  in radians
        m_tpState.position = degreesToRadians(setpoints.arm);
        setGoal(m_tpState);

        // Display requested Arm State to dashboard
        Setpoints.displayArmState(m_armState);
    }

    /**
     * Indicate if arm is being setup to shoot in any way
     */
    public boolean preparing2Shoot() {
        return ((m_armState != GameState.STOWED) && (m_armState != GameState.INTAKE) && (m_armState != GameState.CLIMB) && (m_armState != GameState.HARMONY));
    }
    
    /**
     * Indicate if arm is being prepared to CLIMB
     */
    public boolean preparing2Climb() {
        return ((m_armState == GameState.CLIMB));
    }
    
    /**
     * Indicate if arm is being prepared to FEED
     */
    public boolean preparing2Feed() {
        return ((m_armState == GameState.FEED));
    }
    
    /**
     * Update the PID controller's current Arm setpoint in degrees
     * 
     * @param degrees - the desired Arm position in degrees
     */
    public void updateArmInDegrees(double degrees) {

        // Convert degrees to radians and set the profile goal
        m_armSetpoint = degrees;
        m_tpState.position = degreesToRadians(degrees);
        setGoal(m_tpState);
    }

    public double getDegrees() {
        // Get the smart dashboard
        return tempDegree.get();

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
        m_armSetpoint = getArmJointDegrees(); 
        setGoal(getArmJointRadians());
    }

    // Get the current Arm Joint position error (in degrees)
    public double getArmJointError() {
        return Math.abs(m_armSetpoint - getArmJointDegrees());
    }

    // Check if Arm is at the setpoint (or within tolerance) - private method
    public boolean isArmJointAtSetpoint() {
        return getArmJointError() < m_tolerance;
    }

    // Public method to just check if arm is at setpoint
    public boolean isArmAtSetpoint() {
        return (m_armAction == armAction.kONPOINT);
    }

    // Drive the Arm directly by providing a supply voltage value
    public void setArmVoltage(double voltage) {
        m_armLeader.setControl(m_VoltageOutput.withOutput(voltage));
    }

    // Set the lead motor to the Neutral state (no output)
    public void neutralOutput() {
        m_armLeader.setControl(m_neutral);
    }

    // Returns the current encoder absolute value in DutyCycle units (~0 -> ~1)
    public double getJointPosAbsolute() {
        return m_encoder.getAbsolutePosition();
    }

    // Converts DutyCycle units to Degrees
    public double dutyCycleToDegrees(double dutyCyclePos) {
        return dutyCyclePos * 360;
    }

    // Converts the current encoder reading to Degrees, and corrects relative to a
    // STOWED position of zero.
    public double getArmJointDegrees() {
        return dutyCycleToDegrees(getJointPosAbsolute()) - ArmConstants.kARM_STARTING_OFFSET;
    }

    // Converts DutyCycle units to Radians
    public double dutyCycleToRadians(double dutyCyclePos) {
        return dutyCyclePos * 2.0 * Math.PI;
    }

    // Converts degrees to Radians
    public double degreesToRadians(double degrees) {
        return (degrees * Math.PI) / 180.0;
    }

    // Converts the current encoder reading to Degrees, and corrects relative to a
    // STOWED position of zero.
    public double getArmJointRadians() {
        return dutyCycleToRadians(getJointPosAbsolute()) - degreesToRadians(ArmConstants.kARM_STARTING_OFFSET);
    }

    // Takes a position in radians relative to STOWED, and corrects it to be
    // relative to a HORIZONTAL position of zero.
    // This is used for Feedforward only, where we account for gravity using a
    // cosine function
    public double correctArmJointRadiansForFeedFwd(double position) {
        return position - degreesToRadians(ArmConstants.kARM_HORIZONTAL_OFFSET - ArmConstants.kARM_STARTING_OFFSET);
    }

    public boolean isArmSteady() {
        return armSteadyAtSetpoint;
    }

    public void armCoastMode() {
        m_armLeader.setNeutralMode(NeutralModeValue.Coast);
        m_armFollower.setNeutralMode(NeutralModeValue.Coast);
        disable();
        m_armLeader.disable();

        //m_armFollower.disable();
        //m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));
    } 

    public void armBrakeMode() {
        m_armLeader.setNeutralMode(NeutralModeValue.Brake);
        m_armFollower.setNeutralMode(NeutralModeValue.Brake);
        setGoal(getArmJointRadians());
        enable();
        //m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));
    }



    /*
     * Command Factories
     */

    // To position for Intake, move Arm to INTAKE position
    public Command prepareForIntakeCommand() {
        return new RunCommand(()-> this.updateArmSetpoint(RobotConstants.INTAKE), this)
            .until(()->this.isArmJointAtSetpoint());
    }   
    // To tune the lookup table using SmartDashboard
    public Command tuneArmSetPointCommand() {
        return new RunCommand(()-> this.updateArmInDegrees(tuneArmSetpoint.get()), this)
            .until(()->this.isArmJointAtSetpoint());
    }

    public Command moveToDegreeCommand() {
        return new RunCommand(() -> this.updateArmInDegrees(this.getDegrees()));
    }

    public Command armCoastCommand() {
        return new InstantCommand(() -> armCoastMode());
    }

    public Command armBrakeCommand() {
        return new InstantCommand(() -> armBrakeMode());
    }
}
