// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// IDENTIFICACIÓN PARA LOS CONTROLADORES DE MOTOR EN EL DRIVERTRAIN

public static final int MOTOR_RIGHT_0_ID = 2;
public static final int MOTOR_RIGHT_1_ID = 3;
public static final int MOTOR_LEFT_2_ID = 0;
public static final int MOTOR_LEFT_3_ID = 1;


// hola
// IDENTIFICACION PARA EL JOYSTICK

public static final int GAME_PAD = 0;
public static final int ButtonSquare = 1;
public static final int ButtonX = 2;
public static final int ButtonCircle = 3;
public static final int ButtonTriangle = 4;
public static final int ButtonL1 = 5;
public static final int ButtonR1 = 6;
public static final int ButtonL2 = 7;
public static final int ButtonR2 = 8;
public static final int ButtonShare = 9;
public static final int ButtonOptions = 10;
public static final int ButtonL3 = 11;
public static final int ButtonR3 = 12;
public static final int ButtonPs = 13;
public static final int ButtonTouchPad = 14;




//CONSTANTES PARA AJUSTES DE SET POINT




// CONSTANTES DE VELOCIDAD MAXIMA DE MOTORES DEL DRIVETRAIN

public static final double dampenSpeed = 0.8;
public static final double dampenturn = 0.6;
public static final double DT_ROTATION_SPEED = 0.35;

// CONSTANTES PARA MOTION PROFILING DEL DRIVETRAIN
public static final double kMaxSpeedMetersPerSecond= 1.5;
public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;
public static double dampenSpeedIntake = 1;

// CONSTANTES PARA MOTORES DEL INTAKE

//public static int triggerID = 10;
public static int intakeID = 6;

// CONSTANTES PARA MOTOR DEL SHOOTER
public static int shooterID = 4;
public static double dampenSpeedShooter=1;


// CONSTANTE PARA MOTOR INDEXER
public static int indexID = 7;
public static double dampenSpeedindexer=0.4;


//CONSTANTE PARA MOTOR ELEVADOR LIFT
public static int liftID= 5;










}
