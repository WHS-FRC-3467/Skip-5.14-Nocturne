
# 3467 Base Alpha Code

- This is now the code base for the Beta robot. Repos for subsequent robots will most likely be based on this code as well.
- This code base was started with Jason Daming's improved version of the CTRE Swerve Example (see notes below).
- Added basic Subsystems: Drive, Arm, Shooter, Intake and Stage.
- Cleaned up and reorganized RobotContainer.jave to make it clearer for new programmers.
- Shooter subsystem features TunableNumber constants for tuning Velocity PID via the Shuffleboard.
- Arm, Shooter and Intake use Falcons and therefore the Phoenix 6 library. Intake and Stage use Talon SRX, and therefore the Phoenix 5 library.
- This code uses the CTRE Swerve Generator built into Tuner X. After generating a project using the Generator, copy the newly-generated TunerConstants.java into this code.


# CTRE Swerve Example

Repo: https://github.com/jasondaming/ctre_swerve

Features:
- Improved Limelight support
- Improved PathPlanner autonomous setup
- Added SysID options for running swerve drivetrain characterization
- Different xbox joystick controller layout to try for driver comfort
- Easily scale down the max speed to help newer drivers learn
- "Turtle Mode" (left bumper) temporarily slows the drivetrain down for fine adjustments

This is an expanded version of the CTRE [SwerveWithPathPlanner](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner) example using the CTRE Swerve Builder.  To use it copy the generated/TunerConstants.java file from the generated project and replace the generated/TunerConstants.java file in this project.  You will also need to set your team number.

To use the limelight ensure you have a 36h11 pipeline properly configured and then change the [enable constant](https://github.com/jasondaming/ctre_swerve/blob/master/src/main/java/frc/robot/Vision/Limelight.java#L22) to true.

Will add specific steps on characterization after testing.

# Calibrating the Arm Angle
 
 1. Turn the robot off and push the Arm against its hard stop in the STOWED position <br>
 1. Turn the robot on and connect to the robot (Do not enable) <br>
 1. Open Shuffleboard and find the box with the value for "Arm Angle Uncorrected" <br>
 1. Copy this value into the constant named kARM_STARTING_OFFSET in the "ArmConstants" section of Constants.java <br>
    > The value should be > 0.0 (i.e. not negative). If it is 0.0 or less, then there is an encoder issue. <br>
    > The value should be between 30-120 degrees. Anything over 200 likely means the encoder zero point is not in the right spot).<br>
    > You want to make sure the value you choose is just slightly smaller than the lowest number that appears in "Arm Angle Uncorrected".<br>
    >Otherwise you may get negative readings for the Arm Current Angle, and error checking may prevent the Arm motors from moving. <br>
 1. Move the Arm to the horizontal position and again check the value in the "Arm Angle Uncorrected" box. <br>
 1. Copy this value into the constant named kARM_HORIZONTAL_OFFSET. (It should be between 90-160 degrees).<br>
 1. Save the file and deploy code to the robot. Make sure the Arm starts in the STOWED position. <br>
 1. If the value for Arm Current Angle is a negative value do not enable, and try to do the offsets again <br>
 1. If it is still negative, then there is an issue with the encoder. <br>
 