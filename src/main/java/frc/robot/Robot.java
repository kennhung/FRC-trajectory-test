// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  // create to use on a real robot.
  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);

  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  // Create our gyro object like we would on a real robot.
  private AnalogGyro m_gyro = new AnalogGyro(1);

  // Create the simulated gyro object, used for setting the gyro
  // angle. Like EncoderSim, this does not need to be commented out
  // when deploying code to the roboRIO.
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide,
      KitbotGearing.k10p71, // 10.71:1
      KitbotWheelSize.SixInch, // 6" diameter wheels.
      null // No measurement noise.
  );

  private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  private double kP = 7;
  private double kI = 0;
  private double kD = 0;
  private PIDController m_leftPID = new PIDController(kP, kI, kD);
  private PIDController m_rightPID = new PIDController(kP, kI, kD);

  private Field2d m_field = new Field2d();
  private Field2d m_field_traj = new Field2d();

  Pose2d origin;
  private DifferentialDriveOdometry m_odometry;

  private Trajectory trajectory;
  Timer timer = new Timer();
  RamseteController controller = new RamseteController();
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.8);

  @Override
  public void robotInit() {
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 1024);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 1024);

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Field traj", m_field_traj);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
          .resolve("D:\\Builders.GC\\2021RobotSim\\src\\main\\deploy\\output\\InfRecharge.wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
    }

    origin = trajectory.getInitialPose();

    m_odometry = new DifferentialDriveOdometry(origin.getRotation(), origin);

    SmartDashboard.putData("l_PID", m_leftPID);
    SmartDashboard.putData("r_PID", m_rightPID);

    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_field_traj.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void robotPeriodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putNumber("x", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y", m_odometry.getPoseMeters().getY());
  }

  @Override
  public void autonomousInit() {
    // var start = new Pose2d(0, 4, Rotation2d.fromDegrees(0));
    // var end = new Pose2d(12, 4, Rotation2d.fromDegrees(90));

    // var interiorWaypoints = new ArrayList<Translation2d>();

    // interiorWaypoints.add(new Translation2d(4, 0));
    // interiorWaypoints.add(new Translation2d(8, 8));

    // TrajectoryConfig config = new TrajectoryConfig(10, 3);

    // trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints,
    // end, config);

    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    Trajectory.State goal = trajectory.sample(timer.get());
    var chaSpeed = controller.calculate(m_odometry.getPoseMeters(), goal);

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chaSpeed);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;

    m_leftPID.setSetpoint(left);
    m_rightPID.setSetpoint(right);

    SmartDashboard.putNumber("left", left);
    SmartDashboard.putNumber("right", left);
    SmartDashboard.putNumber("left_error", m_leftPID.getPositionError());
    SmartDashboard.putNumber("right_error", m_rightPID.getPositionError());
    SmartDashboard.putNumber("velocity", goal.velocityMetersPerSecond);
    m_field_traj.setRobotPose(goal.poseMeters);
  }

  public void simulationPeriodic() {
    if (DriverStation.getInstance().isEnabled()) {
      // Set the inputs to the system. Note that we need to convert
      // the [-1, 1] PWM signal to voltage by multiplying it by the
      // robot controller voltage.

      if (DriverStation.getInstance().isAutonomous()) {
        m_driveSim.setInputs(m_leftPID.calculate(m_leftEncoder.getRate()),
            m_rightPID.calculate(m_rightEncoder.getRate()));
      } else {
        m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
            m_rightMotor.get() * RobotController.getInputVoltage());
      }
      //

      // Advance the model by 20 ms. Note that if you are running this
      // subsystem in a separate thread or have changed the nominal timestep
      // of TimedRobot, this value needs to match it.
      m_driveSim.update(0.02);

      // Update all of our sensors.
      m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
      m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
      m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
      m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
      m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
    }
  }

  @Override
  public void teleopInit() {
  }

  Joystick stick = new Joystick(0);
  
  @Override
  public void teleopPeriodic() {
    m_leftMotor.set(-stick.getRawAxis(1));
    m_rightMotor.set(-stick.getRawAxis(5));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
