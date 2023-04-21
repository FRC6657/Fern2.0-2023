// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants.StartingPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.field.AllianceTransform;

/** Add your docs here. */
public class AutoCommands {

    private final SendableChooser<Command> autoChooser;

    public AutoCommands(Drivetrain drivetrian, CommandFactory commandFactory) {

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Nothing", new PrintCommand("No auto selected"));

        autoChooser.addOption("Rotate",
        new SequentialCommandGroup(
            commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_SUB, StartingPose.RED_SUB),
            commandFactory.getRotateAbsolute(0),
            commandFactory.getDriveMeters(0.1, 1)
        )
        );
        
        autoChooser.addOption("Sub-L3-Taxi-1.5",
        new SequentialCommandGroup(
            commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_SUB, StartingPose.RED_SUB),
            commandFactory.getFireL3(),
            commandFactory.getCarry(),
            commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(-0.352, 0.352), 2),
            commandFactory.getRotateAbsolute(10),
            commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(3.85, -3.85), 2),
            commandFactory.getRotateAbsolute(AllianceTransform.allianceBasedDouble(35,-35)),
            commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(1.46, -1.46), 2),
            commandFactory.getFloorPickup(),
            commandFactory.getCarry()
        )
    );

        autoChooser.addOption("Mid-L3-Charge", 
            new SequentialCommandGroup(
                commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_MID, StartingPose.RED_MID),
                commandFactory.getFireL3(),
                commandFactory.getCarry(),
                commandFactory.getRotateAbsolute(AllianceTransform.allianceBasedDouble(-180,180)),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(-4,4), 2),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(2,-2),1),
                commandFactory.getCharge()
            )
        );

        autoChooser.addOption("Bump-L3-Taxi",
            new SequentialCommandGroup(
                commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_SUB, StartingPose.RED_SUB),
                commandFactory.getFireL3(),
                commandFactory.getCarry(),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(0.3, -0.3), 2),
                commandFactory.getRotateAbsolute(0),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(3.75,-3.75)),
                commandFactory.getRotateAbsolute(AllianceTransform.allianceBasedDouble(-45,45)),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(1.25, -1.25)),
                commandFactory.getFloorPickup(),
                commandFactory.getCarry()
            )
        );
    
        autoChooser.addOption("Mid-L3-Charge-1.5", 
        new SequentialCommandGroup(
            commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_MID, StartingPose.RED_MID),
            commandFactory.getFireL3(),
            commandFactory.getCarry(),
            commandFactory.getRotateAbsolute(AllianceTransform.allianceBasedDouble(-180,180)),
            commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(-4,4), 2),  
            commandFactory.getFloorPickup(),
            commandFactory.getCarry(),
            commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(2.5,-2.5),1),
            commandFactory.getCharge()
        )
    );

    autoChooser.addOption("Charge Test", 
    new SequentialCommandGroup(
        commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_MID, StartingPose.RED_MID),
        commandFactory.getCharge()
    )
);

    autoChooser.addOption("L3-Rotate",
        new SequentialCommandGroup(
            commandFactory.getFireL3(),
            commandFactory.getCarry(),
            commandFactory.getRotateAbsolute(0)
        )
    );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAuto(){
        return autoChooser.getSelected();
    }

}