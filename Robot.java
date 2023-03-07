// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import java.util.ArrayList;

public class Robot extends TimedRobot {

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("iSeeYou");

  // Create list of april tags on the field to know location
  private ArrayList<AprilTagFieldLayout> aprilTagList = new ArrayList<AprilTagFieldLayout>();
  aprilTagList.add(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(6);
  private final WPI_VictorSPX m_rightRearMotor = new WPI_VictorSPX(2);

  private final XboxController m_stick = new XboxController(0);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);


  @Override
  public void robotInit() {
    m_rightFrontMotor.setInverted(true);
    m_rightRearMotor.setInverted(true);

    //MAYBE REMOVE NEXT LINE
    camera.setDriverMode(true);
  }

  @Override
  public void teleopPeriodic() {

    if(Math.abs(m_stick.getLeftY()) > 0.2 || Math.abs(m_stick.getLeftX()) > 0.2)
    {
      diffDrive.arcadeDrive(-m_stick.getLeftY() * 0.5, -m_stick.getLeftX() * 0.5);
    }
    else
    {
      diffDrive.arcadeDrive(-m_stick.getRightY(), -m_stick.getRightX());
    }

    double forwardSpeed;
    double rotationSpeed;

    if (m_stick.getAButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        // First calculate range
        double range =
          PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            forwardSpeed = 0; 
            rotationSpeed = 0;
            }
          } else {
            // Manual Driver Mode
            forwardSpeed = -m_stick.getRightY();
            rotationSpeed = m_stick.getLeftX();
        }

    PortForwarder.add(5800, "photonvision.local", 5800);
    var result = camera.getLatestResult();

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

  }
}
