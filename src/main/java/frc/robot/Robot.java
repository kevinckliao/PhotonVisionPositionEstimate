// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.*;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.math.geometry.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  // Publishers created once
  private BooleanPublisher hasTargetPub;
  private DoublePublisher xPub, yPub, headingPub;
  // Motors
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  private final Joystick m_stick = new Joystick(0);
    // --- new PhotonVision fields ---
  private PhotonCamera camera;
  private Field2d field;
  private NetworkTable visionTable;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
  }

  /** Called once when the robot code starts running */
  @Override
  public void robotInit() {
      // initialize PhotonVision
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      inst.startClient4("SimRobot");
      inst.setServer("192.168.0.233");  // your PV IP
      inst.startDSClient();
      camera = new PhotonCamera("Razer_Kiyo_X");
      visionTable = inst.getTable("VisionPose");

      // Create publishers only once
      hasTargetPub = visionTable.getBooleanTopic("hasTarget").publish();
      xPub = visionTable.getDoubleTopic("xMeters").publish();
      yPub = visionTable.getDoubleTopic("yMeters").publish();
      headingPub = visionTable.getDoubleTopic("headingDeg").publish();

      // --- Field visualization setup ---
      field = new Field2d();
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Field", field);

      System.out.println("✅ PhotonVision + Field2d initialized.");
    }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-m_stick.getY(), -m_stick.getX());

    PhotonPipelineResult result = camera.getLatestResult();
      // ✅ Publish whether any targets exist
      hasTargetPub.set(result.hasTargets());

      if (result.hasTargets()) {
        int count = result.getTargets().size();
        //System.out.printf("📸 %d targets detected%n", count);

        // --- Print info for each detected tag ---
        for (PhotonTrackedTarget t : result.getTargets()) {
            Transform3d camToTarget = t.getBestCameraToTarget();
            Translation3d tr = camToTarget.getTranslation();

            // System.out.printf(
            //     "  ▶ ID: %d  X=%.2f  Y=%.2f  Z=%.2f  Amb=%.3f%n",
            //     t.getFiducialId(),
            //     tr.getX(), tr.getY(), tr.getZ(),
            //     t.getPoseAmbiguity()
            // );
        }

        // --- If PhotonVision produced a fused multi-tag pose ---
        if (result.getMultiTagResult().isPresent()) {
            Transform3d pvTransform = result.getMultiTagResult().get().estimatedPose.best;
            Pose3d pvPose = new Pose3d(
                pvTransform.getTranslation(),
                pvTransform.getRotation()
            );

            System.out.printf(
                "🤖 Fused Pose → X=%.2f  Y=%.2f  Z=%.2f  Heading=%.1f°%n",
                pvPose.getX(), pvPose.getY(), pvPose.getZ(),
                pvPose.getRotation().toRotation2d().getDegrees()
            );

            // Update NetworkTable publishers
            xPub.set(pvPose.getX());
            yPub.set(pvPose.getY());
            headingPub.set(pvPose.getRotation().toRotation2d().getDegrees());

            // Update field visualization
            field.setRobotPose(pvPose.toPose2d());
        } else {
            System.out.println("⚠️  No fused pose (single tag only).");
        }
      } else {
          System.out.println("No targets detected.");
          field.setRobotPose(new Pose2d());
      }
  } //teleopPeriodic()
}
