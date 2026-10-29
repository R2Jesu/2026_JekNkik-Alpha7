// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix6.HootAutoReplay;
// import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.vision.stream.CameraServer;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Rotation2d;
// import org.wpilib.math.geometry.Rotation2d;
// import org.wpilib.math.util.Units;
import org.wpilib.util.sendable.Sendable;
import org.wpilib.util.sendable.SendableBuilder;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.framework.TimedRobot;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
// import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.utilities.LimelightHelpers;
// import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /* PHX6EX log and replay timestamp and joystick data */
  /*   private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
  .withTimestampReplay()
  .withJoystickReplay();  */

  // private final boolean kUseLimelight = false; //PHX6EX

  // R2JESU
  private final Field2d ourfield = new Field2d(); // R2JESU
  double omegaRPS; // R2JESU
  Optional<Alliance> alliance = MatchState.getAlliance();
  private List<Double> trenchTags = new ArrayList<>();
  LimelightHelpers.PoseEstimate myLimelightPose; 

  boolean trenchHazard;
  public static boolean perspectiveSet;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  // R2JESU
  public Pigeon2 getPigeon() {
    Pigeon2 pigeon2 = m_robotContainer.m_robotDrive.getPigeon2();
    return pigeon2;
  }

  @Override // PHX6ex DOESN'T HAVE A ROBOT INIT?
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    SmartDashboard.putData("Field", ourfield);
    perspectiveSet=false;
    m_robotContainer.m_robotDrive.getPigeon2().reset();
/*     alliance = MatchState.getAlliance();
    if (alliance.get() == Alliance.RED) {
      System.out.println("Red, setting yaw 180");
      m_robotContainer.m_robotDrive.getPigeon2().setYaw(180.0);
    } else {
      System.out.println("Not red, set to 0 yaw");
      m_robotContainer.m_robotDrive.getPigeon2().setYaw(0.0);
    }  */
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    LimelightHelpers.SetIMUMode(Constants.kLimelightName, 1);
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[0].angle.getRadians(),
                null);
            builder.addDoubleProperty(
                "Front Left Velocity",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[0].velocity,
                null);

            builder.addDoubleProperty(
                "Front Right Angle",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[1].angle.getRadians(),
                null);
            builder.addDoubleProperty(
                "Front Right Velocity",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[1].velocity,
                null);

            builder.addDoubleProperty(
                "Back Left Angle",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[2].angle.getRadians(),
                null);
            builder.addDoubleProperty(
                "Back Left Velocity",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[2].velocity,
                null);

            builder.addDoubleProperty(
                "Back Right Angle",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[3].angle.getRadians(),
                null);
            builder.addDoubleProperty(
                "Back Right Velocity",
                () -> m_robotContainer.m_robotDrive.getState().ModuleStates[3].velocity,
                null);

            builder.addDoubleProperty(
                "Robot Angle",
                () -> m_robotContainer.m_robotDrive.getState().Pose.getRotation().getRadians(),
                null);
          }
        });
    trenchTags.add(7.0);
    trenchTags.add(6.0);
    trenchTags.add(12.0);
    trenchTags.add(1.0);
    trenchTags.add(17.0);
    trenchTags.add(28.0);
    trenchTags.add(22.0);
    trenchTags.add(23.0);
  }

  @Override
  public void robotPeriodic() {
    // m_timeAndJoystickReplay.update(); //MEE PHX5ex

    CommandScheduler.getInstance().run();

    // Limelight addition
    LimelightHelpers.SetRobotOrientation(
        Constants.kLimelightName,
        m_robotContainer.m_robotDrive.getState().RawHeading.getDegrees(),
        0,
        0,
        0,
        0,
        0);
/*     alliance = MatchState.getAlliance();
    if (alliance.get() == Alliance.RED && teleopEra) {
    myLimelightPose =
        LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(
            Constants.kLimelightName);
    } else { */
    myLimelightPose = 
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.kLimelightName);
  //  } 
 // changed "limelight" to constant for consistency
    // MEE PHX6ex? omegaRps = Units.radiansToRotations(driveState.Speeds.omega);
    // omegaRPS = Units.degreesToRotations(m_robotContainer.m_robotDrive.getTurnRate());
    // PHX6ex includes && Math.abs(omegaRps)<2.0 ... radians per second, not degrees
    if (myLimelightPose != null
        && myLimelightPose.tagCount > 0
        && myLimelightPose.avgTagDist < 6.0) {
      m_robotContainer.m_robotDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.9, 0.9, 0.9));
      m_robotContainer.m_robotDrive.addVisionMeasurement(
          myLimelightPose.pose, myLimelightPose.timestampSeconds);
    }

    ourfield.setRobotPose(m_robotContainer.m_robotDrive.getState().Pose);

    // PoseEstimate poseEstimate =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    // double distance = poseEstimate.avgTagDist;

    // odometry aiming and ranging:
    // docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-and-ranging

    if (myLimelightPose != null
        && myLimelightPose.tagCount > 0
        && myLimelightPose.avgTagDist < 2.0
        && trenchTags.contains(LimelightHelpers.getFiducialID(Constants.kLimelightName))
        && m_robotContainer.m_intakeSubsystem.isIntakeRaised()) {

      trenchHazard = true;
    } else {
      trenchHazard = false;
    }

    alliance = MatchState.getAlliance();
    if (alliance.get() == Alliance.RED && !perspectiveSet) {
      perspectiveSet=true;
      m_robotContainer.m_robotDrive.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));
    }
    else {
      perspectiveSet=true;
    }

    SmartDashboard.putString("Choice", m_autonomousCommand.toString());
/*     if (myLimelightPose != null) {
       SmartDashboard.putNumber("Tag Count", myLimelightPose.tagCount);
    } */
    SmartDashboard.putNumber(
        "Pigeonyaw", m_robotContainer.m_robotDrive.getState().RawHeading.getDegrees());
    SmartDashboard.putNumber(
        "pigeon2 yaw", Math.floorMod((int) getPigeon().getYaw().getValueAsDouble(), 360));
    SmartDashboard.putBoolean("trenchHazard", !trenchHazard);
    // SmartDashboard.putNumber("Distance", distance);
    // SmartDashboard.putNumber("omegaRPS", Math.abs(omegaRPS));
    // SmartDashboard.putNumber("Robotx", m_robotContainer.m_robotDrive.getState().Pose.getX());
    // SmartDashboard.putNumber("Roboty", m_robotContainer.m_robotDrive.getState().Pose.getY());
    // SmartDashboard.putNumber("Robotrotation",
    // m_robotContainer.m_robotDrive.getState().Pose.getRotation().getDegrees());
     //SmartDashboard.putNumber("Limelightx", poseEstimate.pose.getX());
    // SmartDashboard.putNumber("Limelighty", poseEstimate.pose.getY());
    // SmartDashboard.putNumber("Limelightrotation", poseEstimate.pose.getRotation().getDegrees());
  }

  @Override
  public void disabledInit() {
    LimelightHelpers.SetIMUMode(Constants.kLimelightName, 1);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(); // MEE PHX6ex has
      // CommandScheduler.getInstance().schdeuld(m_autonomousCommand)
    }
    LimelightHelpers.SetIMUMode(Constants.kLimelightName, 4);
    //Adding here as well.  Thought is that mayeb robot init doesn't have driver station access.
    alliance = MatchState.getAlliance();
    // && m_autonomousCommand.getName() == "None"
/*      if (alliance.get() == Alliance.RED) {
      m_robotContainer.m_robotDrive.getPigeon2().setYaw(180.0);
      System.out.println("Setting 180 yaw");
    } else {
      m_robotContainer.m_robotDrive.getPigeon2().setYaw(0.0);
      System.out.println("Setting yaw 0");
    }  */
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand
          .cancel(); // mee PHX6ex  CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
    LimelightHelpers.SetIMUMode(Constants.kLimelightName, 4);
    perspectiveSet=false;

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}

  @Override
  public void simulationPeriodic() {}
}
