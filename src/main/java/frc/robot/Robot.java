// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.SPI; // required for ADIS IMUs
//import edu.wpi.first.wpilibj.ADIS16448_IMU; 
import edu.wpi.first.wpilibj.ADIS16470_IMU;  

import org.photonvision.PhotonCamera; //https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

public class Robot extends TimedRobot {
  private final Spark m_rightDrive = new Spark(0);
  private final Spark m_leftDrive = new Spark(1);
  private final Victor shooter = new Victor(3);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_stick = new XboxController(0);
  private final Timer m_timer = new Timer();
  private final Encoder right_encoder = new Encoder(6, 7, true, CounterBase.EncodingType.k4X);
  private final Encoder left_encoder = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);
  //public static final ADIS16448_IMU imu = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kY, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);
  public static final ADIS16470_IMU imu = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kY,ADIS16470_IMU.IMUAxis.kX,ADIS16470_IMU.IMUAxis.kZ, SPI.Port.kOnboardCS0, ADIS16470_IMU.CalibrationTime._1s);
  PhotonCamera camera = new PhotonCamera("Camera_Module_v1");    //wide angle on rPi3 is called "OV5647"
                                                                            //other camera on rPi4 is called "Camera_Module_v3"
  public double SpeedForward,rotationClockwise,cYaw;
  public static double pTargetYaw;
  public static boolean pHasTarget;
  public static double Sharpness = 0.3;
  public static double MaxTurnEffort = 0.25;
  public static double shooterSpeed = 0.3;   
  @Override
  public void robotInit() {
    imu.calibrate();
    m_rightDrive.setInverted(true);
    
    right_encoder.setSamplesToAverage(5);
    left_encoder.setSamplesToAverage(5);
    right_encoder.setDistancePerPulse(6.0 * 0.3048 /8093); // the pulses where averaged from l&r = 8093 of 6' (3 x 2' tiles)
    left_encoder.setDistancePerPulse( 6.0 * 0.3048 /8093); // the factor 0.3048 converts feet into metres
    right_encoder.setMinRate(1.0);
    left_encoder.setMinRate(1.0);
    right_encoder.reset();
    left_encoder.reset();
    SmartDashboard.putNumber("Sharpness", Sharpness);
    SmartDashboard.putNumber("MaxTurnEffort", MaxTurnEffort);
    SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
  }
  
  @Override
  public void robotPeriodic(){
    var result = camera.getLatestResult();
    pHasTarget = result.hasTargets();
      
    if (pHasTarget) {
      pTargetYaw = result.getBestTarget().getYaw();
      SmartDashboard.putBoolean("Has Target",pHasTarget);
      SmartDashboard.putNumber("TargetID", result.getBestTarget().getFiducialId());
      SmartDashboard.putNumber("Target Yaw", pTargetYaw);    
    }
    
    cYaw = -imu.getAngle(ADIS16470_IMU.IMUAxis.kY) % 360;
    
    
    SmartDashboard.putNumber("Angle", -imu.getAngle(ADIS16470_IMU.IMUAxis.kY));
    SmartDashboard.putNumber("cYaw", cYaw);
    SmartDashboard.putNumber("R_Encoder Distance", right_encoder.getDistance());
    SmartDashboard.putNumber("L_Encoder Distance", left_encoder.getDistance());
    SmartDashboard.putNumber("R_Encoder", right_encoder.get());
    SmartDashboard.putNumber("L_Encoder", left_encoder.get());
    SmartDashboard.putNumber("Rotation Clockwise", rotationClockwise);
   
    //Sharpness = SmartDashboard.getNumber("Sharpness",Sharpness);
    //MaxTurnEffort = SmartDashboard.getNumber("MaxTurnEffort", MaxTurnEffort);
    //shooterSpeed = SmartDashboard.getNumber("Shooter Speed",shooterSpeed);
  }
  @Override
  public void autonomousInit() {
//    m_timer.reset();
    right_encoder.reset();
    left_encoder.reset();
//    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  // if (right_encoder.getDistance() < (1.00)) {       //only  drive when the encoder reads less than 1.00 m -- NEEDS ENCODERS TO PREVENT WILD ROBOT --
    if (m_timer.get() < 2) {         //only  drive when the timer is less than 2 seconds
      m_robotDrive.arcadeDrive(0.5, 0.0,false); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  @Override
  public void teleopPeriodic() {
    shooter.set(shooterSpeed);
    if (m_stick.getLeftBumper()) {
        right_encoder.reset();
        left_encoder.reset();
    }
    if (m_stick.getRightBumper()) {
      imu.reset();
  }
  SpeedForward = m_stick.getRightTriggerAxis()-m_stick.getLeftTriggerAxis();
  rotationClockwise = m_stick.getLeftX()*0.6;

  rotationClockwise = m_stick.getYButton() ?(1-Math.exp(-Sharpness*Math.pow(cYaw,2)))*Math.signum(cYaw)*MaxTurnEffort: rotationClockwise;
//  rotationClockwise = m_stick.getXButton() ?(1-Math.exp(-Sharpness*Math.pow(pTargetYaw,2)))*Math.signum(pTargetYaw)*MaxTurnEffort: rotationClockwise;
  

  m_robotDrive.arcadeDrive(SpeedForward, rotationClockwise,false); 
  }
}
