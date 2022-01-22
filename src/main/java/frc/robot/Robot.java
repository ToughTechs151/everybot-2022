package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CANSparkMax driveLeftA = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax driveLeftB = new CANSparkMax(2, MotorType.kBrushed);

  CANSparkMax driveRightA = new CANSparkMax(3, MotorType.kBrushed);
  CANSparkMax driveRightB = new CANSparkMax(4, MotorType.kBrushed);

  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  VictorSPX intake = new VictorSPX(6);

  Joystick driverController = new Joystick(0);

  boolean armUp = true;
  boolean burstMode = false;
  double lastBurstTime = 0;

  double autoStart = 0;
  boolean goForAuto = false;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driveLeftA.setInverted(true);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(true);
    driveLeftB.burnFlash();
    driveRightA.setInverted(false);
    driveRightA.burnFlash();
    driveRightB.setInverted(false);
    driveRightB.burnFlash();
    
    arm.setInverted(false);
    arm.setIdleMode(IdleMode.kBrake);
    arm.burnFlash();

    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < 0.5){
        arm.set(0.5);
      }
      else{
        arm.set(0.08);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < 0.35){
        arm.set(-0.5);
      }
      else{
        arm.set(-0.13);
      }
    }

    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if(goForAuto){
      if(autoTimeElapsed < 3){
        intake.set(ControlMode.PercentOutput, -1);
      }else if(autoTimeElapsed < 6){
        intake.set(ControlMode.PercentOutput, 0);
        driveLeftA.set(-0.3);
        driveLeftB.set(-0.3);
        driveRightA.set(-0.3);
        driveRightB.set(-0.3);
      }else{
        intake.set(ControlMode.PercentOutput, 0);
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forward = -driverController.getRawAxis(1);
    double turn = -driverController.getRawAxis(2);
    
    double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;

    driveLeftA.set(driveLeftPower);
    driveLeftB.set(driveLeftPower);
    driveRightA.set(driveRightPower);
    driveRightB.set(driveRightPower);

    if(driverController.getRawButton(5)){
      intake.set(VictorSPXControlMode.PercentOutput, 1);;
    }
    else if(driverController.getRawButton(7)){
      intake.set(VictorSPXControlMode.PercentOutput, -1);
    }
    else{
      intake.set(VictorSPXControlMode.PercentOutput, 0);
    }

    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < 0.5){
        arm.set(0.5);
      }
      else{
        arm.set(0.08);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < 0.35){
        arm.set(-0.5);
      }
      else{
        arm.set(-0.13);
      }
    }
  
    if(driverController.getRawButtonPressed(6) && !armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
    }
    else if(driverController.getRawButtonPressed(8) && armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
    }  

  }

  @Override
  public void disabledInit() {
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    arm.set(0);
    intake.set(ControlMode.PercentOutput, 0);
  }
    
}
