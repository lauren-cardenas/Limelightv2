package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //DRIVE TRAIN DEFINITIONS

  private final WPI_VictorSPX m_leftMotor1 = new WPI_VictorSPX(RobotMap.m_leftMotor1);
  private final WPI_VictorSPX m_leftMotor2 = new WPI_VictorSPX(RobotMap.m_leftMotor2);
  private final WPI_VictorSPX m_rightMotor1 = new WPI_VictorSPX(RobotMap.m_rightMotor1);
  private final WPI_VictorSPX m_rightMotor2 = new WPI_VictorSPX(RobotMap.m_rightMotor2);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftMotor1, m_leftMotor2);
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightMotor1, m_rightMotor2);

  private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final Encoder m_leftEncoder = new Encoder(RobotMap.m_leftEnc1,RobotMap.m_leftEnc2);
  private final Encoder m_rightEncoder = new Encoder(RobotMap.m_rightEnc1,RobotMap.m_rightEnc2);

  //CONTROLLER DEFINITIONS

  private final XboxController m_driverController = new XboxController(RobotMap.DRIVER_CONTROLLER);
  private final XboxController m_operatorController = new XboxController(RobotMap.OPERATOR_CONTROLLER);
  
  //GAME TIMER
  
  private final Timer m_timer = new Timer();

  //LIMELIGHT DEFINITIONS


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //Resets the timer at the beginning of autonomous.
    m_timer.reset();

    //Starts the timer at the beginning of autonomous.
    m_timer.start();

  }

 
  @Override
  public void autonomousPeriodic() {
    
    //Switch is used to switch the autonomous code to whatever was chosen
    //  on your dashboard.
    //Make sure to add the options under m_chooser in robotInit.
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        if (m_timer.get() < 2.0){
          m_driveTrain.arcadeDrive(0.5, 0); //drive forward at half-speed
        } else {
          m_driveTrain.stopMotor(); //stops motors once 2 seconds have elapsed
        }
        break;
    }
  }

  
  @Override
  public void teleopPeriodic() {
    double triggerVal = 
    (m_driverController.getTriggerAxis(Hand.kRight)
    - m_driverController.getTriggerAxis(Hand.kLeft))
    * RobotMap.DRIVING_SPEED;

    double stick = 
    (m_driverController.getX(Hand.kLeft))
    * RobotMap.TURNING_RATE;

    double left_command = (triggerVal + stick) * RobotMap.DRIVING_SPEED;
    double right_command = (triggerVal - stick) * RobotMap.DRIVING_SPEED;

    m_driveTrain.tankDrive(left_command, right_command);

    //TUNE THESE TO SMOOTH ROBOT MOVEMENT
    double kP = -0.1;
    double min_command = 0.03;

    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if (m_driverController.getXButton()){
      double heading_error = -tx;
      double steering_adjust = 0.0;
      if(tx>1.0){
        steering_adjust = kP * heading_error - min_command;
      }
      else if(tx<1.0){
        steering_adjust = kP * heading_error + min_command;
      }
      left_command += steering_adjust;
      right_command -= steering_adjust;
    }
  }

  @Override
  public void testPeriodic() {
  }
        
}
