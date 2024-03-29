// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. Aman :)

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase {
  // state stuff
  public static enum DrivetrainState {
    AUTON_PATH, JOYSTICK_DRIVE, DISABLED
  }

  private DrivetrainState state;

  // kinematics stuff
  private SwerveDriveKinematics swerveKinematics;
  private SwerveModuleState[] desiredStates = new SwerveModuleState[4];
  private SwerveModule[] motors;
  private int rotationPoint = 0;
  public boolean fieldOriented = false;

  // Auton Stuff
  private Pose2d pose;
  private SwerveDriveOdometry simOdometry;
  private SwerveDriveOdometry odometry;
  private Field2d field;
  public PIDController xController;
  public PIDController yController;
  public ProfiledPIDController thetaController;
  private TrajectoryConfig trajectoryConfig;

  // sensors
  // private AHRS gyro;
  private AHRS gyro;
  private double gyroOffset = 0; // degrees
  
  // SIM
  private double time = 0;
  DriveSimulationData simulationData;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrivetrain() {
    gyro = new AHRS(SPI.Port.kMXP);

    // reset in new thread since gyro needs some time to boot up and we don't 
    // want to interfere with other code
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      }
      catch (Exception e)
      {
        System.out.println("Reset Gyro Failed");
      }
    }).start(); 

    swerveKinematics = new SwerveDriveKinematics(
      Constants.SwerveDrivetrain.m_frontLeftLocation, 
      Constants.SwerveDrivetrain.m_frontRightLocation, 
      Constants.SwerveDrivetrain.m_backLeftLocation, 
      Constants.SwerveDrivetrain.m_backRightLocation);

    motors = new SwerveModule[4];

    motors[0] = new SwerveModule(
      Constants.SwerveDrivetrain.m_frontLeftDriveID,
      Constants.SwerveDrivetrain.m_frontLeftTurnID,
      Constants.SwerveDrivetrain.m_frontLeftEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_frontLeftEncoderOffset);

    motors[1] = new SwerveModule(
      Constants.SwerveDrivetrain.m_frontRightDriveID,
      Constants.SwerveDrivetrain.m_frontRightTurnID,
      Constants.SwerveDrivetrain.m_frontRightEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_frontRightEncoderOffset);

    motors[2] = new SwerveModule(
      Constants.SwerveDrivetrain.m_backLeftDriveID,
      Constants.SwerveDrivetrain.m_backLeftTurnID,
      Constants.SwerveDrivetrain.m_backLeftEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_backLeftEncoderOffset);

    motors[3] = new SwerveModule(
      Constants.SwerveDrivetrain.m_backRightDriveID,
      Constants.SwerveDrivetrain.m_backRightTurnID,
      Constants.SwerveDrivetrain.m_backRightEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_backRightEncoderOffset);

    pose = new Pose2d();
    odometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d());
    simOdometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d());
    field = new Field2d();

    simulationData = new DriveSimulationData(simOdometry, field);

    desiredStates[0] = new SwerveModuleState();
    desiredStates[1] = new SwerveModuleState();
    desiredStates[2] = new SwerveModuleState();
    desiredStates[3] = new SwerveModuleState();

    xController = new PIDController(Constants.SwerveDrivetrain.m_x_control_P, Constants.SwerveDrivetrain.m_x_control_I, Constants.SwerveDrivetrain.m_x_control_D);
    yController = new PIDController(Constants.SwerveDrivetrain.m_y_control_P, Constants.SwerveDrivetrain.m_y_control_I, Constants.SwerveDrivetrain.m_y_control_D);
    thetaController = new ProfiledPIDController(
      Constants.SwerveDrivetrain.m_r_control_P, 
      Constants.SwerveDrivetrain.m_r_control_I, 
      Constants.SwerveDrivetrain.m_r_control_D, 
      Constants.SwerveDrivetrain.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    trajectoryConfig = new TrajectoryConfig(
      Constants.SwerveDrivetrain.kDriveMaxSpeedMPS, 
      Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    trajectoryConfig.setKinematics(swerveKinematics);
    state = DrivetrainState.JOYSTICK_DRIVE;
  }

  /**
   * Zero the physical gyro
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * get the heading of the physical gyro
   * @return heading angle in degrees
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw() - gyroOffset, 360);
    //.getAngle()
    /**
     * use IEEEremainder because it uses formula:
     * dividend - (divisor x Math.Round(dividend / divisor))
     * versus the remainder operator (%) which uses:
     * (Math.Abs(dividend) - (Math.Abs(divisor) x (Math.Floor(Math.Abs(dividend) / Math.Abs(divisor))))) x Math.Sign(dividend)
     */
  }

  /**
   * get the gyro angle as rotation2d
   * @return Rotation2d heading
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * get the simulated heading of the robot as rotation2d
   * @return Rotation2d simulated heading
   */
  public Rotation2d getSimRotation2d()  {
    return new Rotation2d(simulationData.getHeading());
  }

  /**
   * update simulation, drive and smart dashboard data
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putData("Field", field);
    for (SwerveModule m : motors) {
      SmartDashboard.putString(m.getName(), m.getStateSummary());
    }
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putString("Robot Sim Location", simOdometry.getPoseMeters().getTranslation().toString());

    odometry.update(getRotation2d(), getOutputModuleStates());
    field.setRobotPose(
      odometry.getPoseMeters().getX(),
      odometry.getPoseMeters().getY(),
      getRotation2d()
    );

    // use Euler integration to estimate current heading from velocities, dt, and past states
    double currTime = Timer.getFPGATimestamp();
    double dt = currTime - time;
    time = currTime;
    // Translation2d vel = getLinearVelocity(); don't use
    // double xPosSim = pose.getX() + vel.getX() * dt; don't use
    // double yPosSim = pose.getY() + vel.getY() * dt; don't use
    // double anglePosSim = simulationData.getHeading() + getDesiredRotationalVelocity() * dt;
    // simulationData.update(desiredStates, anglePosSim); // for sim
  }

  /**
   * stop the swerve modules
   */
  public void stopModules() {
    for (SwerveModule m : motors) {
      m.disableModule();
    }
  }

  /**
   * set the states of all four modules
   * @param states
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrivetrain.kMaxSpeedMPS);
    for (int i = 0; i < motors.length; i++)
    {
      motors[i].setDesiredState(states[i]);
      desiredStates[i] = states[i];
    }
  }

  /**
   * set the states of all four modules using speeds (horizontal, vertical and angular) as well as a rotation point
   * @param v_forwardMps
   * @param v_sideMps
   * @param v_rot (rad/sec)
   * @param rotatePoint
   */
  public void setSpeeds(double v_forwardMps, double v_sideMps, double v_rot, Translation2d rotatePoint) {
    ChassisSpeeds speeds = new ChassisSpeeds(v_forwardMps, v_sideMps, v_rot);
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(speeds, rotatePoint);
    for (int i = 0; i < 4; i++)
    {
      motors[i].setDesiredState(moduleStates[i]);
    }
  }

  /**
   * get the swerve kinematics of the robot
   * @return SwerveDriveKinematics kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  /**
   * get the linear velocity of the robot in mps
   * @return linear velocity
   */
  public Translation2d getLinearVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getState(),
        motors[1].getState(),
        motors[2].getState(),
        motors[3].getState()
    );
    return new Translation2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond
    );
  }

  /**
   * get the rotational velocity of the robot in rad/sec
   * @return rotational velocity
   */
  public double getRotationalVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getState(),
        motors[1].getState(),
        motors[2].getState(),
        motors[3].getState()
    );
    return speeds.omegaRadiansPerSecond;
  }

  /**
   * get the desired rotational velocity of the robot
   * @return desired rotational velocity rad/sec
   */
  public double getDesiredRotationalVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getDesiredState(),
        motors[1].getDesiredState(),
        motors[2].getDesiredState(),
        motors[3].getDesiredState()
    );
    return speeds.omegaRadiansPerSecond;
  }

  /**
   * get the actual states of all swerve modules
   * @return SwerveModuleState[] states
   */
  public SwerveModuleState[] getOutputModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++)
    {
      states[i] = motors[i].getState();
    }
    return states;
  }

  /**
   * get the position of the robot
   * @return Pose2d pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * get the simulated position of the robot
   * @return Pose2d pose
   */
  public Pose2d getSimPose() {
    return simOdometry.getPoseMeters();
  }

  /**
   * reset the real odometry of the robot
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }

  /**
   * reset the simulated odometry of the robot
   * @param pose
   */
  public void resetSimOdometry(Pose2d pose) {
    simulationData.setHeading(pose.getRotation().getRadians());
    simOdometry.resetPosition(pose, new Rotation2d(simulationData.getHeading()));
    stopModules();
  }

  /**
   * get the field
   * @return Field2d field
   */
  public Field2d getField() {
    return field;
  }

  /**
   * get the index of the Translation2d array of rotation points for evasive maneuvers
   * @return index
   */
  public int getRotationPointIdx() {
    return rotationPoint;
  }

  /**
   * set the index of the Translation2d array of rotation points for evasive maneuvers
   * @param idx
   */
  public void setRotationPointIdx(int idx) {
    rotationPoint = idx;
  }

  /**
   * get the trajectory config of the robot for autonomous
   * @return TrajactoryConfig trajectoryConfig
   */
  public TrajectoryConfig getTrajectoryConfig() {
    return trajectoryConfig;
  }

  /**
   * convert a swerve command and trajectory to SequentialCommandGroup
   */
  public SequentialCommandGroup getDriveSimCommand(SwerveControllerCommand swerveCommand, Trajectory trajectory) {
    Command swerveControllerCommand;
    return new SequentialCommandGroup(
        new InstantCommand(() -> resetSimOdometry(trajectory.getInitialPose())),
        swerveCommand,
        new InstantCommand(() -> stopModules())
    );
  }

  /**
   * set state as autonomous
   */
  public void setAutonomous() {
    state = DrivetrainState.AUTON_PATH;
  }

  /**
   * set state as joystick drive
   */
  public void setJoystick() {
    state = DrivetrainState.JOYSTICK_DRIVE;
  }

  /**
   * set state as disabled
   */
  public void setDisabled() {
    state = DrivetrainState.DISABLED;
  }

  public void resetModules() {
    // 10 times just because talons are weird 
    // and setSelectedSensorPosition does not always 
    // correctly work but repeating it many times 
    // ensures that the talon is set correctly 
    for (SwerveModule m:motors) {
      m.resetEncoders();
    }

    // for (SwerveModule m:motors) {
    //   m.zeroPosition();
    // }
  }
}