// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.util.Deadbander;

public class RobotContainer {

  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Pivot mPivot = new Pivot();
  private final CommandFactory mCommandFactory = new CommandFactory(mDrivetrain, mIntake, mPivot);

  private AutoCommands mAutoCommands = new AutoCommands(mDrivetrain, mCommandFactory);
  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);
  private CommandGenericHID mDebug = new CommandGenericHID(2);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    //#region Debug Controls
    mDebug.button(1).onTrue(
      mCommandFactory.getStartingConfig()
    );

    mDebug.button(2).onTrue(
      mCommandFactory.getManIntake()
    ).onFalse(
      mCommandFactory.stopManIntake()
    );

    mDebug.button(3).onTrue(
      mCommandFactory.getManOuttake()
    ).onFalse(
      mCommandFactory.stopManIntake()
    );

    //#endregion

    //#region Driver Controls
    mDrivetrain.setDefaultCommand(
      new RunCommand(()-> mDrivetrain.drive(
        -Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(),0.1) * DriveConstants.kMaxSpeed,
        -Deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.1) * DriveConstants.kMaxTurnSpeed
      ),
      mDrivetrain
    )
    );

    mDriver.a().toggleOnTrue(
      new RunCommand(
        () -> mDrivetrain.driveWGridMode(
          Deadbander.applyLinearScaledDeadband(mDriver.getLeftX(),0.1) * DriveConstants.kMaxSpeed,
          Deadbander.applyLinearScaledDeadband(-mDriver.getRightY(), 0.1) * 10
        ),
        mDrivetrain
      )
    );

    mDriver.povLeft().onTrue(
      new InstantCommand(
        () -> mDrivetrain.resetPoseAndGyro(new Pose2d(0, 0, Rotation2d.fromDegrees(180))))
    );
    
    mDriver.povUp().onTrue(
      mDrivetrain.changeState(DriveConstants.FrontState.FORWARD)
    );

    mDriver.povDown().onTrue(
      mDrivetrain.changeState(DriveConstants.FrontState.REVERSE)
    );

    mDriver.leftTrigger().onTrue(
      mDrivetrain.changeState(DriveConstants.ModState.TURBO)
    ).onFalse(
      mDrivetrain.changeState(DriveConstants.ModState.NORMAL)
    );

    mDriver.rightTrigger().onTrue(
      mDrivetrain.changeState(DriveConstants.ModState.SLOW)
    ).onFalse(
      mDrivetrain.changeState(DriveConstants.ModState.NORMAL)
    );

    //#endregion
    //#region Operator Controls

    mOperator.povRight().onTrue(
      new InstantCommand(
        mPivot::zeroEncoder,
        mPivot   )
    );

    mOperator.povLeft().onTrue(
      mCommandFactory.getStartingConfig()
    );
    
    mOperator.x().onTrue(
      mCommandFactory.getSubPickup()
    ).onFalse(
      mCommandFactory.getCarry()
    );

    mOperator.rightBumper().onTrue(
      mCommandFactory.getFloorPickup()
    ).onFalse(
      mCommandFactory.getCarry()
    );
    
    mOperator.y().onTrue(
      mCommandFactory.getFireL1()
    ).onFalse(
      mCommandFactory.getCarry()
    );

    mOperator.b().onTrue(
      mCommandFactory.getFireL2()
    ).onFalse(
      mCommandFactory.getCarry()
    );

    mOperator.a().onTrue(
      mCommandFactory.getFireL3Tele()
    ).onFalse(
      mCommandFactory.getCarry()
    );

    mOperator.rightTrigger().onTrue(
      mCommandFactory.getManIntake()
    ).onFalse(
      mCommandFactory.stopManIntake()
    );
   
    mOperator.leftTrigger().onTrue(
      mCommandFactory.getManOuttake()
    ).onFalse(
      mCommandFactory.stopManIntake()
    );

    mOperator.povUp().onTrue(
      mCommandFactory.getManL2()
    );

    mOperator.povDown().onTrue(
      mCommandFactory.getManL1()
    );

    //#endregion

  }

  public void stopAll(){
    CommandScheduler.getInstance().schedule(
      new ParallelCommandGroup(
        new InstantCommand(() -> mDrivetrain.stop()),
        new InstantCommand(() -> mIntake.changeState(Constants.IntakeConstants.State.STOP)),
        new InstantCommand(() -> mPivot.stop())
      ).ignoringDisable(true)
    );
  }

  public void enabledBrake(){
    CommandScheduler.getInstance().schedule(
        new InstantCommand(() -> mPivot.brake()).ignoringDisable(true)
    );
  }

  public Command getAutonomousCommand() {

    return mAutoCommands.getAuto();

  }
}