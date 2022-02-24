// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
//imports wpi and 3rd perty libaries
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
//create variables, constants, and names
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private DifferentialDrive m_myRobot; //what to drive with the differential drive
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private static final int leftDeviceID = 1; 
  private static final int rightDeviceID = 2;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private RelativeEncoder m_encoder;

  public double feetDriven;

  public double startTime; //creates a vairable to store the time
  //Pneumatics
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1); //names the pneumatic solenoid
  AnalogGyro gyro = new AnalogGyro(0); //creates a new analog gyroscope called gyro on analog in 0
  public double heading; //creates a heading varibale for gyro calculations
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //Instantiates hardware. Connects names and variables from above to specific instances of hardware
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor); //creates a differential drive to control my_robot

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    m_encoder = m_rightMotor.getEncoder(); //attaches the encoder value from right motor to the variable m_encoder

   
    Shuffleboard.getTab("Example tab").add(gyro); //creates a gyro tab on the shuffleboard
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {} //always running regardless of robbot mode

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    startTime = Timer.getFPGATimestamp(); //sets the auto start time
heading=gyro.getAngle();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
        
    }
    double error = gyro.getAngle();
        m_myRobot.tankDrive(.5 + (.005 * error), .5 - (.005 * error));
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
   m_rightMotor.getEncoder().setPosition(0); //resets encoder between teleops

 }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY()); //drive my_robot with a TankDrive style getting the values from sticks
    if (m_leftStick.getRawButtonPressed(1)){ //upon pressing button 1 on the left stick do this action
    solenoid.set(DoubleSolenoid.Value.kForward);
    }
    if (m_leftStick.getRawButtonReleased(1)){ //when button is released do this action
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    feetDriven = (2.0*3.14*3*(1/10.71)*m_encoder.getPosition()/12); //computes feet driven from encoder revolutions
    SmartDashboard.putNumber("Encoder Velocity1", m_encoder.getVelocity()); //output the encoder value in RPM to the smart dashboard
    SmartDashboard.putNumber("Drive Distance 1", feetDriven);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
