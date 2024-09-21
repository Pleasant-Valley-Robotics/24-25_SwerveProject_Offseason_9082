// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class MotorPorts{
    public static final int INTAKE_TOP_MOTOR_PORT = 16;
    public static final int INTAKE_BOTTOM_MOTOR_PORT = 17;

    //Same as above. 
    public static final int SHOOTER_TOP_MOTOR_PORT = 18;
    public static final int SHOOTER_BOTTOM_MOTOR_PORT = 19;

    public static final int ARM_LEFT_MOTOR_PORT = 20;
    public static final int ARM_RIGHT_MOTOR_PORT = 21;

    //Same as above. 
    public static final int LEFT_HANGING_MOTOR_PORT = 22;
    public static final int RIGHT_HANGING_MOTOR_PORT = 23;
  }

  public static class PhysicalConstants {
    public static final double REVS_TO_RADIANS = Math.PI * 2;
    public static final double REV_PER_MIN_TO_RADIAN_PER_SEC = Math.PI / 60;
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class IntakeConstants {
    // PID controller constants
    public static final double K_P = 0, K_I = 0, K_D = 0;
    // Feedforward controller constants
    public static final double K_S = 0, K_V = 0;

    public static final PIDController INTAKE_PID = new PIDController(K_P, K_I, K_D);
    public static final SimpleMotorFeedforward INTAKE_FEEDFORWARD = new SimpleMotorFeedforward(K_S, K_V);

    public static final double INTAKE_RPM_CONVERSION = 26.0 / 18;
  }

  public static class ShooterConstants {
    // PID controller constants
    public static final double K_P = 0, K_I = 0, K_D = 0;

    // Feedforward controller constants
    public static final double K_S = 0, K_V = 0;

    public static final PIDController SHOOTER_PID = new PIDController(K_P, K_I, K_D);
    public static final SimpleMotorFeedforward SHOOTER_FEEDFORWARD = new SimpleMotorFeedforward(K_S, K_V);

    public static final double SHOOTER_RPM_CONVERSION = 26.0 / 18;
  }
}
