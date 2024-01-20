

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Joystick m_stick;
  private RobotContainer m_robotContainer;

  private final XboxController gamePadController = new XboxController(1);
  Joystick joystick = new Joystick(0);

 
  private MotorControllerGroup leftMotor; 
  private MotorControllerGroup rightMotor;
  private DifferentialDrive Muzeyyen;
 
  
  private static final int left1DeviceID = 4; 
  private static final int left2DecvieID = 3;
  private static final int right1DeviceID =2;
  private static final int right2DeviceID =5;
  private static final int intakeDeviceID = 1;
  private static final int ArmMotorPort = 7;
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;


 
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_rightMotor2;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax intakeMotor;
  private VictorSP ArmMotor;

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private double offset = 0;
  private int count = 0;
  private boolean balanced = false;
  private double startTime; 

  
  boolean autoBalanceXMode;
  boolean autoBalanceYMode;
  
  
  
  @Override
  public void robotInit() {
  m_robotContainer = new RobotContainer();
  m_leftMotor1 = new CANSparkMax(left1DeviceID, MotorType.kBrushed);
   m_rightMotor2 = new CANSparkMax(right2DeviceID, MotorType.kBrushed);
   m_leftMotor2 = new CANSparkMax(left2DecvieID, MotorType.kBrushed);
   m_rightMotor1 = new CANSparkMax(right1DeviceID, MotorType.kBrushed);
   intakeMotor = new CANSparkMax(intakeDeviceID, MotorType.kBrushed);
   
    
    m_robotContainer = new RobotContainer();
    leftMotor = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    rightMotor = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
    Muzeyyen = new DifferentialDrive(leftMotor, rightMotor);
    rightMotor.setInverted(true);
  }

  
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("aci", ahrs.getPitch());
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    startTime = Timer.getFPGATimestamp();
    offset = ahrs.getPitch();
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
    // şarj istasyonuna bağlı kalma 
   if(time - startTime < 3){
    intakeMotor.set(-0.5);
  }
  else{
    intakeMotor.set(0);
  
  }  
   
   if( time - startTime > 3 && time - startTime <= 7.5 ){
    m_leftMotor1.set(0);
    m_leftMotor2.set(0);
    m_rightMotor1.set(0);
    m_rightMotor2.set(0);
   }
   else if(time - startTime > 7.3 && time - startTime <= 9){
    m_leftMotor1.set(-0.45);
    m_leftMotor2.set(-0.45);
    m_rightMotor1.set(0.45);
    m_rightMotor2.set(0.45);
   }
   else if(time - startTime > 9.7 && time - startTime <= 15 && !balanced){
    if(Math.abs(ahrs.getPitch()) - offset < 3.0)
    {
      count++;
    }

    if(count > 4)
    {
      balanced = true;
    }

    m_leftMotor1.set(-0.29);
    m_leftMotor2.set(-0.29);
    m_rightMotor1.set(0.29);
    m_rightMotor2.set(0.29);
   }
   else
   {
    m_leftMotor1.set(0);
    m_leftMotor2.set(0);
    m_rightMotor1.set(0);
    m_rightMotor2.set(0);
   }
  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
   Muzeyyen.arcadeDrive(-gamePadController.getRawAxis(1), -gamePadController.getRawAxis(4));
   Muzeyyen.arcadeDrive(-m_stick.getRawAxis(0), m_stick.getRawAxis(1));
    
    // Arm System 
    
    if(joystick.getRawButton(5))  //Arm yukarı
    {
      ArmMotor.set(0.6);
    }
    else if(joystick.getRawButton(3))   //Arm aşağı
    {
      ArmMotor.set(-0.6);
    }
    else
    {
      ArmMotor.set(0);
    }

    //Intake System 

    if(joystick.getRawButton(1))  // küp al
    {
      intakeMotor.set(0.5);
    }
    else if(joystick.getRawButton(2))   //alt, orta küp at
    {
      intakeMotor.set(-0.3);
    }
    else if(joystick.getRawButton (7))   //yukarı küp at 
    {
      intakeMotor.set(-0.55);
    }
    else
    {
      intakeMotor.set(0);
    }

    


  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
