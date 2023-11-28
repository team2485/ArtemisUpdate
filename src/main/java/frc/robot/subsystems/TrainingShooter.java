// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrainingShooter extends SubsystemBase {
 
  private CANSparkMax m_left;
  private CANSparkMax m_right;
  private CANSparkMax m_feeder;
  public TrainingShooter(int leftPort, int rightPort,int feederPort) {

    m_left=new CANSparkMax(leftPort,CANSparkMax.MotorType.kBrushless);
    m_right=new CANSparkMax(rightPort,CANSparkMax.MotorType.kBrushless); //plug in the missing values, motor type is brushless
    m_right.setInverted(true);
    m_feeder = new CANSparkMax(feederPort, MotorType.kBrushless);
    
    
    //m_feedeController.setTolerance(Constants.Feeder.);
  }



// public double getLeftEncoderVelocity() {
  
// }

// public double getRightEncoderVelocity() {
  
// }
public void runVelocityPID(double velocity,double feederVel) {
 //GOAL: run flywheels at velocity% of power
 m_left.set(velocity);
 m_right.set(velocity);
  m_feeder.set(feederVel);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
