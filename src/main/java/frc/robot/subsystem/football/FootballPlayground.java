// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.football;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.PortMan;

public class FootballPlayground extends SubsystemBase 
{
    private static Logger logger = Logger.getLogger(FootballPlayground.class.getName());


public void init(PortMan portMan) throws Exception{
    logger.entering(FootballPlayground.class.getName(), "init()");

    logger.exiting(FootballPlayground.class.getName(), "init()");
}

}