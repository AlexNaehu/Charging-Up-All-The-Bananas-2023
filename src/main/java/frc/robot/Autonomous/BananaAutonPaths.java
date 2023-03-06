package frc.robot.Autonomous;

import frc.robot.Mechanisms.BananaClaw;
import frc.robot.Robot.BananaPreSets;
import frc.robot.Robot.Robot;

public class BananaAutonPaths {
    
    private final double leftHalfTurn = 90.0; //"left turn" means the front right corner whips around CCW
    private final double leftFullTurn = 180;
    private final double rightHalfTurn = -90.0; //"right turn" means the front left corner whips around CW
    private final double rightFullTurn = -180;

    

    private final int straightForwardSide = 0;
    private final int straightForwardMid = 1;
    private final int tipToeForward = 2; //Used for inching onto the charging station and for approaching Scoring

    private final int straightBackwardSide = 3;
    private final int straightBackwardMid = 4;
    private final int tipToeBackward = 5; //Using for inching into the charging station and for approaching Scoring



    /*IMPORTANT: FIGURE OUT HOW TO PRELOAD OBJECTS INTO THE CLAW*/



    public void rightScoreMob() //turns left, away from the wall on the right side of the robot
    {
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardSide);
        BananaPreSets.cargoPickUp();
        BananaTurn.turnPID(leftFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl1RocketBall();
        BananaTurn.turnPID(leftFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);

    }

    public void leftScoreMob() //turns right, away from the wall on the left side of the robot
    {
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardSide);
        BananaPreSets.cargoPickUp();
        BananaTurn.turnPID(rightFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl1RocketBall();
        BananaTurn.turnPID(rightFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);

    }

    //turns left, doesn't actually matter which way; also inches onto the charging station
    public void midScorePark() 
    {
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardMid);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.neutralPivotAngle();
        BananaTurn.turnPID(leftFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardMid);
        BananaDriveStraight.driveStraight(tipToeForward);

    }
}
