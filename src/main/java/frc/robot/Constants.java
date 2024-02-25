// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.Setpoints;
import frc.robot.Util.Setpoints.GameState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be
 * used for any other purpose. All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotConstants {

        public static final boolean kIsTuningMode = true;
        public static final boolean kIsDriveTuningMode = true;
        public static final boolean kIsArmTuningMode = false;
        public static final boolean kIsIntakeTuningMode = false;
        public static final boolean kIsStageTuningMode = false;
        public static final boolean kIsShooterTuningMode = true;

        // Shooter and Arm Setpoints
        public static final Setpoints STOWED = new Setpoints(1.0, 0.4, 0.0, 0.0, GameState.STOWED);
        public static final Setpoints INTAKE = new Setpoints(3.0, 2.0, 0.0, 0.0, GameState.INTAKE);
        public static final Setpoints SUBWOOFER = new Setpoints(3.0, 1.0, 25.0,25.0,  GameState.SUBWOOFER);
        public static final Setpoints AMP = new Setpoints(88.0, 0.4, 30.0,30.0,  GameState.AMP);
        public static final Setpoints PODIUM = new Setpoints(23.0, 0.4, 50.0,50.0,  GameState.PODIUM);
        public static final Setpoints WING = new Setpoints(30.0, 0.4, 70.0,60.0,  GameState.WING);
        public static final Setpoints PREPCLIMB = new Setpoints(0.0, 0.4, 0.0,0.0,  GameState.PREPCLIMB);
        public static final Setpoints CLIMB = new Setpoints(0.0, 0.4, 0.0,0.0,  GameState.CLIMB);
        public static final Setpoints TRAP = new Setpoints(0.0, 0.4, 20.0,20.0,  GameState.TRAP);
        public static final Setpoints LOOKUP = new Setpoints(0.0, 0.4, 20.0,25.0,  GameState.LOOKUP);
    }
    
    public static final class CanConstants {

        // Drivebase CAN IDs are 1 -> 13
        // See generated/TunerConstants.java

        // Shooter CAN IDs
        public static final int ID_ShooterLeft = 15;
        public static final int ID_ShooterRight = 17;

        // Intake CAN IDs
        public static final int ID_IntakeMotor = 19;
        public static final int ID_IntakeCtrRoller = 20;

        // Stage CAN IDs
        public static final int ID_StageMotor = 23;

        // Arm CAN IDs
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;
        public static final int LED_CANDLE = 27;
    }

    public static final class DIOConstants {

        public static final int kArmAbsEncoder = 0;
        public static final int kStageBeamBreak = 1;
    }

    public static final class StageConstants {

        public static final double kIntakeSpeed = 0.6;
        public static final double kFeedToShooterSpeed = 1.0;
        public static final double kFeedToAmpSpeed = 0.7;
        public static final double kFeedToTrapSpeed = 0.5;

        public static final double kFeedToShooterTime = 1.0;
        public static final double kFeedToAmpTime = 1.0;
        public static final double kFeedToTrapTime = 5.0;
    }

    public static final class IntakeConstants {

        public static final double kIntakeSpeed = 1.0;
        public static final double kEjectSpeed = -0.75;
    }

    public static final class ShooterConstants {

        // Shooter speeds are set in the individual position Setpoints at the top of this file
        
        public static final double kShooterTolerance = 10.0;
        public static final double timeToShoot = .3; //Time it takes before note leave the shooter
    }

    public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d(0));
    public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));



    public static final class ArmConstants {
/*
 * Calibrating the Arm Angle
 * 
 * - Turn the robot off and push the Arm against its hard stop in the STOWED position <br>
 * - Turn the robot on and connect to the robot (Do not enable) <br>
 * - Open Shuffleboard and find the box with the value for "Arm Angle Uncorrected" <br>
 * - Copy this value into the constant named kARM_STARTING_OFFSET in the "ArmConstants" section of Constants.java <br>
 * - The value should be > 0.0 (i.e. not negative). If it is 0.0 or less, then there is an encoder issue.
 * - The value should be between 30-120 degrees. Anything over 200 likely means the encoder zero point is not in the right spot)<br>
 * - You want to make sure the value you choose is just slightly smaller than the lowest number that appears in "Arm Angle Uncorrected".
 * - Otherwise you may get negative readings for the Arm Current Angle, and error checking may prevent the Arm motors from moving.
 * - Move the Arm to the horizontal position and again check the value in the "Arm Angle Uncorrected" box. <br>
 * - Copy this value into the constant named kARM_HORIZONTAL_OFFSET. (It should be between 90-160 degrees).<br>
 * - Save the file and deploy code to the robot. Make sure the Arm starts in the STOWED position. <br>
 * - If the value for Arm Current Angle is a negative value do not enable, and try to do the offsets again <br>
 * - If it is still negative, then there is an issue with the encoder. <br>
 * 
 * 
 */
        // Observed Arm Offsets
        // Measured against the hardstop when the Arm is in the STOWED position
        public static final double kARM_STARTING_OFFSET = 98.0;
        // Measured when the Arm is exactly horizontal
        public static final double kARM_HORIZONTAL_OFFSET = 115.0;

        // Feedforward Gains
        public static final double kS = .5;  // The Static Gain, in volts
        public static final double kG = .25;  // The Gravity Gain, in volts
        public static final double kV = 3.45;  // The Velocity Gain, in volt seconds per radian
        public static final double kA = .01;  // The acceleration gain, in volt seconds^2 per radian

        // PID Control Gains
        public static final double kP = 18.0; // P Gain - Volts
        public static final double kI = 0.00;  // I Gain - Volts
        public static final double kD = 0.2;  // D Gain - Volts


        // Profiled PID Constants
        public static final double kArm_Cruise = 4.0;           // Radians per second
        public static final double kArm_Acceleration = 10.0;   // Radians per second^2

        public static final double kDuty_Cycle_Min = 1.0/1025.0;
        public static final double kDuty_Cycle_Max = 1024.0/1025.0;

        // Motor Neutral dead-band : Range 0.001 -> 0.25
        public static final double kNeutral_Deadband = 0.005;

	}
	
    public static class Vision {
        public static final String kCameraName = "front_cam";
        //public static final String kCameraName = "USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(.174,-0.186, 0.588), new Rotation3d(0, Math.toRadians(10),0));

        //public static final Transform3d kRobotToCam =
                //new Transform3d(new Translation3d(0.264922,0.2465578, 0.2182876), new Rotation3d(0, Math.toRadians(22.09),Math.toRadians(5)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

}
