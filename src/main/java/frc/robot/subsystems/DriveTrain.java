// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import libraries


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

//import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//UNUSED LIBRARIES 

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.math.controller.PIDController;
//import java.util.ResourceBundle.Control;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Encoder;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import edu.wpi.first.wpilibj.drive.*;
//import edu.wpi.first.wpilibj.controller.*;


// THIS IS A CHANGE I JUST MADE

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

//VARIABLES PARA EL GIROSCOPIO
private static final double kAngleSetpoint = 0.0; //Angulo al que debe apuntar.
private static final double kPTurning = 0.1; // propotional turning constant// Constante de proporcionalidad
//private static final double kiTurning = 0.0;
//private static final double kdTurning = 0.0;

// UNCOMMENT IF USING THE ADXR450
private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
public ADXRS450_Gyro imu = new ADXRS450_Gyro(kGyroPort);

//UNCOMMENT IF USING THE ADIS16470
//private  ADIS16470_IMU imu = new ADIS16470_IMU();



//VARIABLE PARA AJUSTAR LA VELOCIAD MÁXIMA DE LOS MOTORES.
private double dampenSpeed = Constants.dampenSpeed;
private double dampenTurn = Constants.dampenturn;


//MOTOR GROUPS

private WPI_VictorSPX rightMotorFront = new WPI_VictorSPX(Constants.MOTOR_RIGHT_0_ID);
private WPI_VictorSPX rightMotorRear = new WPI_VictorSPX(Constants.MOTOR_RIGHT_1_ID);
private WPI_VictorSPX leftMotorFront = new WPI_VictorSPX(Constants.MOTOR_LEFT_2_ID);
private WPI_VictorSPX leftMotorRear = new WPI_VictorSPX(Constants.MOTOR_LEFT_3_ID);

private DifferentialDrive differentialDrive;

//CREAR CONTROLADORES PARA EL PID DE LOS ENCODERS
private static double kDt = 0.02;

private final TrapezoidProfile.Constraints m_constraintsright =
new TrapezoidProfile.Constraints(4, 3 );
private final TrapezoidProfile.Constraints m_constraintsleft =
new TrapezoidProfile.Constraints(4, 3 );

private final ProfiledPIDController m_controllerRight =
new ProfiledPIDController(0.15, 0.05, 0.0, m_constraintsright, kDt);
private final ProfiledPIDController m_controllerLeft =
new ProfiledPIDController(0.15, 0.05, 0.0, m_constraintsleft, kDt);

//CREAR CONTROLADORES PARA EL PID DEL GYRO

private static double kDtg = 0.005;

private final TrapezoidProfile.Constraints gyro_Constraints =
new TrapezoidProfile.Constraints(10, 2 );
public final ProfiledPIDController gyro_controller =
new ProfiledPIDController(0.5, 0.0, 0.0, gyro_Constraints, kDtg);

double rotspeed = Constants.DT_ROTATION_SPEED;


//DriveTrain ENCODERS

private Encoder rightEncoder = new Encoder(0,1);
private Encoder leftEncoder = new Encoder(2,3);
private Encoder extraEncoder = new Encoder(4,5);




  /** Creates a new DriveTrain. *////////////////////////////////////////////////////////////////////
  public DriveTrain() 
  {
    super(); //CONSTRUYE EL OBJETO DEL PADRE.
    
    //Set motors
    
    rightMotorRear.follow(rightMotorFront);
    leftMotorRear.setInverted(true);
    leftMotorFront.setInverted(true);
    leftMotorRear.follow(leftMotorFront);
    differentialDrive = new DifferentialDrive(rightMotorFront,leftMotorFront);



    //Configuracion de los encoders

    rightEncoder.setDistancePerPulse(1./2048);
    leftEncoder.setDistancePerPulse(1./2048);
    leftEncoder.setReverseDirection(true); //cambiar signo para que ambos tengan el mismo sentido
    extraEncoder.setDistancePerPulse(1./1000);
    
  }

  //HELPER METHODS FOR THE DRIVETRAIN
  public void drive(Joystick joy) //metodo para el manejo con control libre.
  {
    differentialDrive.arcadeDrive(joy.getY()*dampenSpeed,joy.getX()*dampenTurn );
  }

  public void driveStraight(Joystick joy)//Manejar en linea recta utilizando el giroscopio
  {

    
    double turningValue = (kAngleSetpoint  -  getAngle() ) * kPTurning;
    // Invert the direction of the turn if we are going backwards
		//turningValue = Math.copySign(turningValue, joy.getY());
    differentialDrive.arcadeDrive(joy.getY()*dampenSpeed, turningValue);
    differentialDrive.setDeadband(0.05);
  
    
  }

  public void driveStraight(double speed) //manejar en linea recta sin MANDO o Joystick
  {

    
  double turningValue = (kAngleSetpoint  -  getAngle() ) * kPTurning;
  differentialDrive.arcadeDrive(-speed, turningValue);

  }

  public void driveToDistance(double distance)//manejar a una distancia con encoders
 {

  //double distancetoRotations = distance*(1/0.45);
  double setpoint = distance;

  m_controllerRight.setGoal(setpoint);
  m_controllerLeft.setGoal(setpoint);

  double averageEncoders = (rightEncoder.getDistance() + leftEncoder.getDistance()) / 2;


  driveStraight(m_controllerLeft.calculate(averageEncoders));

  //differentialDrive.tankDrive(-m_controllerRight.calculate(rightEncoder.getDistance()),
  //                            -m_controllerLeft.calculate(leftEncoder.getDistance()));

  

}
  
  public void stopDriveTrain(){

  differentialDrive.stopMotor();
}

  public void rotate(double angleSp){
      double setpoint = angleSp-3;
      gyro_controller.setGoal(setpoint);
   

      if(getAngle()>angleSp){

        differentialDrive.tankDrive(-rotspeed, rotspeed);
        //differentialDrive.tankDrive(-gyro_controller.calculate(m_gyro.getAngle()),0);

      }

      if(getAngle()<angleSp)
      {
       
        differentialDrive.tankDrive(rotspeed, -rotspeed);
       // differentialDrive.tankDrive(0,-gyro_controller.calculate(m_gyro.getAngle()));

      }



}


// SETTER AND GETTER METHODS FOR THE GYRO
public double getAngle(){

return -imu.getAngle();

}
  
  public void resetGyro(){


    imu.reset();

  }

  public void calibrateGyro(){

    imu.calibrate();

  } 
 
 
// SETTER AND GETTER METHODS FOR THE ENCODERS  
  public void resetLeftEncoder(){

    leftEncoder.reset();
  }

  public void resetRightEncoder(){

  rightEncoder.reset();

  }
  
  public double getLeftEncoder(){

  return leftEncoder.getDistance();

  }

  public double getRightEncoder(){

    return rightEncoder.getDistance();
  
    }


 //SETTER AND GETTER METHODS FOR THE CONTROLLERS   
  public void resetControllers(){

m_controllerLeft.reset(0.0);
m_controllerRight.reset(0.0);
gyro_controller.reset(0.0);



    }
  

////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {

    SmartDashboard.putNumber("rightEncoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("leftEncoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("rawEncoderRight", rightEncoder.getRaw());
    SmartDashboard.putNumber("Gyro 16470: ", ( getAngle() ));
    
    SmartDashboard.putNumber("Gyro Controller", gyro_controller.calculate(getAngle()));

   /* SmartDashboard.putNumber("GyroInstantX", imu.getGyroInstantX());
    SmartDashboard.putNumber("GyroInstantY", imu.getGyroInstantY());
    SmartDashboard.putNumber("GyroInstantZ", imu.getGyroInstantZ());
*/

    SmartDashboard.putNumber("ExtraEncoder", extraEncoder.getDistance());



    // This method will be called once per scheduler run
  }
}
