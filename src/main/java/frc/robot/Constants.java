// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SimpleCurrentLimit;
import frc.robot.util.field.AllianceTransform;

public class Constants {

  public static class DriveConstants{

    public static double kTrackwidth = Units.inchesToMeters(19);
    public static double kDistancePerPulse = (1.0/2048d) * (Units.inchesToMeters(6) * Math.PI) * (1/10.71);

    public static PIDConstants mTrajConstants = new PIDConstants(0.5, 0, 0);
    
    public static final SupplyCurrentLimitConfiguration kDriveCurrentLimit = SimpleCurrentLimit.getSimpleCurrentLimit(30);

    public static final PIDController kTurnPID = new PIDController(1d / 10, 0, 0);
    public static final PIDController kAutoDrivePID = new PIDController(2, 0, 0);
    public static final PIDController kTeleDrivePID = new PIDController(0.5, 0, 0);
    public static final PIDController kChargePID = new PIDController(1.5d/11d, 0, 0);
    public static final PIDController kSnapPID = new PIDController(1d/180, 0, 0);
    
    public static final double kDriveKS = 0.25;
    public static final double kTurnKS = 0.65;

    public static double kTrajectoryMaxSpeed = 1;
	  public static double kTrajectoryMaxAccel = 1;
    
    public static double kMaxSpeed = 6380.0 * (1 / 10.71) * (Units.inchesToMeters(6) * Math.PI) * (1 / 60d);
    public static double kMaxTurnSpeed = (kMaxSpeed * (1 / (kTrackwidth * Math.PI))) * (2 * Math.PI); 
    
    public static double kTurboTurningSpeed = 0.4;
    public static double kNormalTurningSpeed = 0.2;
    public static double kTurboForwardSpeed = 1;
    public static double kNormalForwardSpeed = 0.8;

    public static double kSlowForwardSpeed = 0.6;
    public static double kSlowTurningSpeed = 0.05;

    public static enum FrontState {
     
      FORWARD(1),
      REVERSE(-1);

      public final double direction;

      /**
       * @param direction Motor Percentage
       */

      FrontState(double direction) {
        this.direction = direction;
      }
    }
   

   public static enum ModState {
    
    TURBO(DriveConstants.kTurboForwardSpeed, DriveConstants.kTurboTurningSpeed),
    NORMAL(DriveConstants.kNormalForwardSpeed, DriveConstants.kNormalTurningSpeed),
    SLOW(DriveConstants.kSlowForwardSpeed, DriveConstants.kSlowTurningSpeed);

    public final double xMod;
    public final double rotMod;

    /**
     * @param modifier Turbo or Slow
     */

    ModState(double xMod, double rotMod) {
      this.xMod = xMod;
      this.rotMod = rotMod;
      }
      
    }
  }

    public static class RobotConstants{
        public static double maxVoltage = 15;
        
        public static class CAN {
          public static int kFrontLeft = 1;
          public static int kBackLeft = 2;
          public static int kFrontRight = 3;
          public static int kBackRight = 4;
          public static int kPigeon = 5;
          public static int kIntake = 6;
          public static int kFrontPivot = 7;
          public static int kBackPivot = 8;

        }

        public enum StartingPose {
     
          BLUE_SUB(new Pose2d(new Translation2d(2.6, 4.45), Rotation2d.fromDegrees(-90))),
          BLUE_MID(new Pose2d(new Translation2d(2.6, 2.75), Rotation2d.fromDegrees(-90))),
          BLUE_BUMP(new Pose2d(new Translation2d(2.6, 1), Rotation2d.fromDegrees(-90))),

          RED_SUB(AllianceTransform.flipAlliance(BLUE_SUB.pose)),
          RED_MID(AllianceTransform.flipAlliance(BLUE_MID.pose)),
          RED_BUMP(AllianceTransform.flipAlliance(BLUE_BUMP.pose));

          
          public final Pose2d pose;
    
          /**
           * @param direction Motor Percentage
           */
    
          StartingPose(Pose2d pose) {
            this.pose = pose;
          }
        }
    }

    public static class IntakeConstants {
    
        public static final double kInSpeed = 0.25;
        public static final double kOutSpeed = 0.40;
    
        public static final double kS = 1;
    
        public static enum State {
          
          GRAB(-kInSpeed),
          RELEASE(kOutSpeed),
          L1RELEASE(0.1),
          L2RELEASE(0.15),
          L3RELEASE(0.275),
          L3RELEASETELE(0.3),
          IDLE(-kS/12),
          STOP(0),
          STARTING(0);
    
          public final double speed;
    
          /**
           * @param speed Motor Percentage
           */
          State(double speed) {
            this.speed = speed;
          }
    
        }
    
      }

      public static class PivotConstants {

        public static double kGearing = ((1.0 / 20) * (16.0 / 60));
        public static double kThroughboreOffset = 0.501;

        public static enum State {
          SUBSTATION(5),
          L1(45),
          L2(-20),
          L3(-25),
          L3Tele(-27),
          STOP(0),
          STARTING(-100),
          FLOOR(110),
          CARRY(-90);
    
          public final double angle;
    
          /**
           * @param angle Pivot Angle
           */
          State(double angle) {
            this.angle = angle;
          }
    
        }
    
      }
}