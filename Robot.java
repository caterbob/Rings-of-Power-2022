// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //Constants
  int LEFT_MOTOR_ID = 4;
  int RIGHT_MOTOR_ID = 2;
  int CONTROLLER_ID = 0;
  
  //Drivetrain Class
  Drivetrain m_drivetrain = Drivetrain.getInstance();

  //Deadband
  
  //Joystick
  Joystick controller = new Joystick(CONTROLLER_ID);

  // Pneumatics
  Solenoid piston = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  Solenoid claw = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
  boolean piston_up = true;
  boolean claw_grip = false;

  // Motors
  TalonSRX elbow = new TalonSRX(0);
  CANSparkMax servo = new CANSparkMax(1, MotorType.kBrushless);
  RelativeEncoder encoder = servo.getEncoder();
  MotorType AC = MotorType.kBrushless;

  // Button Mapping
  Joystick m_controller = new Joystick(1);

  int k_piston = 2;   // b   
  int k_claw = 6;     // right bumper
  int k_servoAxis = 1;
  int k_elbowAxis = 4;


  @Override
  public void robotInit() {
    piston.set(true);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Piston Code
    if(m_controller.getRawButton(k_piston)){
      if(piston_up){
        piston.set(false);
        piston_up = false;
      }
    }
    else{
        piston.set(true);
        piston_up = true; 
    } 

    // Claw Code
    if(m_controller.getRawButton(k_claw)){
      if(claw_grip){
        claw.set(false);
        claw_grip = false;
      }
    }
    else{
      claw.set(true);
      claw_grip = true; 
    } 
  
    // Claw Servo Code
    if(m_controller.getRawAxis(k_servoAxis)> .5 && encoder.getPosition()!= 115.5)
      servo.set(0.1);
    else if(m_controller.getRawAxis(k_servoAxis)< -.5 && encoder.getPosition()!= 0)
      servo.set(-0.1);
    else
     servo.set(0);

    // Elbow Talon (may have to change since talons may not have negative speed allowed)
    if (m_controller.getRawAxis(k_elbowAxis) > .5)
      elbow.set(ControlMode.PercentOutput, 0.05);
    else if (m_controller.getRawAxis(k_elbowAxis) < -.5)
      elbow.set(ControlMode.PercentOutput, -0.05);
    else
      elbow.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
