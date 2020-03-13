/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.concurrent.TimeUnit;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import jdk.javadoc.internal.tool.Start;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // crete motor controller objects XD
  WPI_TalonSRX driveOne = new WPI_TalonSRX(1);
  WPI_TalonSRX driveTwo = new WPI_TalonSRX(2);
  WPI_TalonSRX driveThree = new WPI_TalonSRX(3);
  WPI_TalonSRX driveFour = new WPI_TalonSRX(4);
  WPI_TalonSRX intake = new WPI_TalonSRX(6);
  WPI_VictorSPX feed = new WPI_VictorSPX(7);
  WPI_TalonSRX shooter = new WPI_TalonSRX(5);
  WPI_VictorSPX hook = new WPI_VictorSPX(10);
  WPI_VictorSPX winch1 = new WPI_VictorSPX(8);
  WPI_VictorSPX winch2 = new WPI_VictorSPX(9);
  // create solenoid objects XD
  DoubleSolenoid ballShoot = new DoubleSolenoid(7, 6);

  SpeedControllerGroup m_winch = new SpeedControllerGroup(winch1, winch2);
  SpeedControllerGroup m_left = new SpeedControllerGroup(driveOne, driveThree);
  SpeedControllerGroup m_right = new SpeedControllerGroup(driveTwo, driveFour);
  // Create drive object
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  // Create joystick object
  private Joystick m_stick = new Joystick(0);
  private Joystick weeb = new Joystick(1);

  private Timer timer1 = new Timer();

  CameraServer server;

  boolean shooting = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Set Talon motor controllers to 0% output
    driveOne.set(ControlMode.PercentOutput, 0);
    driveTwo.set(ControlMode.PercentOutput, 0);
    driveThree.set(ControlMode.PercentOutput, 0);
    driveFour.set(ControlMode.PercentOutput, 0);
    feed.set(ControlMode.PercentOutput, 0);
    intake.set(ControlMode.PercentOutput, 0);
    intake.setNeutralMode(NeutralMode.Brake);
    feed.setNeutralMode(NeutralMode.Brake);
    // Set intake current limit to 35 amps and stay off for 1 ms
    intake.configPeakCurrentLimit(15, 1);
    intake.enableCurrentLimit(true);

    new Thread(() -> {
      UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
      camera1.setResolution(256, 144);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 256, 144);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start();
}



  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // lmao
    timer1.reset();
    timer1.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    if (timer1.get() < 5.0) {
      //just to get off the line
      m_drive.arcadeDrive(-0.5, 0);
    } else m_drive.arcadeDrive(0, 0);
    
    //code for optimal position, comment if not
    if (timer1.get() > 6.0) {
      shooter.set(ControlMode.PercentOutput, -0.65); //-0.65 (higher for alignment issues)
    } else {
      shooter.set(ControlMode.PercentOutput, 0.0);
    }

    if (timer1.get() > 9.0) {
      ballShoot.set(DoubleSolenoid.Value.kReverse);
      feed.set(ControlMode.PercentOutput, 0.6); //0.6
    } else {
      ballShoot.set(DoubleSolenoid.Value.kForward);
      feed.set(ControlMode.PercentOutput, 0.0);
    }
  }

  @Override
  public void teleopInit() {
    timer1.reset();
    timer1.stop();
    shooting = false;
  }

  @Override
  public void teleopPeriodic() {
    // Fetch joystick and button values
    m_drive.arcadeDrive(m_stick.getY(), m_stick.getX());
    boolean shootButton = m_stick.getRawButton(1);
    boolean feedButton = m_stick.getRawButton(3);
    boolean hookUpButton = m_stick.getRawButton(12);
    boolean hookDownButton = m_stick.getRawButton(11);
    boolean intakeButton = m_stick.getRawButton(2);
    boolean outtakeButton = m_stick.getRawButton(5);
    boolean winchUpButton = m_stick.getRawButton(8);
    boolean winchDownButton = m_stick.getRawButton(7);

    boolean feedButtonAux = weeb.getRawButton(1);
    boolean feedOutbuttonAux = weeb.getRawButton(2);

    if (hookUpButton) {
      hook.set(ControlMode.PercentOutput, 0.75);
    } else {
      if (hookDownButton) {
        hook.set(ControlMode.PercentOutput, -0.5);
      } else {
        hook.set(ControlMode.PercentOutput, 0);
      }
    }
    // Intake commands
    if (intakeButton){
      intake.set(ControlMode.PercentOutput, -0.4);
    } else{
      if (feedButton){
        feed.set(ControlMode.PercentOutput, 0.6);
        } else {
     if (outtakeButton){
        feed.set(ControlMode.PercentOutput, -0.4);
        intake.set(ControlMode.PercentOutput, 0.2);
    } else {
      intake.set(ControlMode.PercentOutput, 0);
        feed.set(ControlMode.PercentOutput, 0);
    }
   }
  }
    // shoot commands
      if (shootButton) {
        //wait an amount of seconds before opening the pneumatic
        timer1.reset();
        timer1.stop();
        timer1.start();
        shooting = true;
        //once the motor is spun up, open the pneumatic
      } 
      
      if (timer1.get() > 0 && timer1.get() < 10 && shooting == true) {
        //turn on shooter and spin up to prepare shooting
        shooter.set(ControlMode.PercentOutput, -0.625); //-0.625
      } else {
        shooter.set(ControlMode.PercentOutput, 0.0);
        ballShoot.set(DoubleSolenoid.Value.kForward);
        timer1.reset();
        timer1.stop();
        shooting = false;
      }
      
      if (timer1.get() > 1.5 && timer1.get() < 10 && shooting == true) {
        ballShoot.set(DoubleSolenoid.Value.kReverse);
      }

      if (winchUpButton){
     m_winch.set(0.8);
      } else {
        if (winchDownButton){
          m_winch.set(-0.3);
      } else {
        m_winch.set(0);
      }
      }
                // Intake Auxiliary commands
  if (feedButtonAux){
    feed.set(ControlMode.PercentOutput, 0.6);
             } else {
          if (feedOutbuttonAux){
             feed.set(ControlMode.PercentOutput, -0.4);
         }
       }
  }
}
