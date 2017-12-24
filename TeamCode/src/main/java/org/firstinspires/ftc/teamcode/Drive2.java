package org.firstinspires.ftc.teamcode;

/**
 * Created by howard on 12/21/17.
 */

public class Drive2 {
    public static final float MAX_TURN_TIME = (float)4000;
    public static final float BUMP_TIME     = (float)2000;

    public botMotors fwd6(float pwr, long goal, long stageTime){
        botMotors mPower = new botMotors();
        if (goal < stageTime){
            mPower.leftFront = pwr;
            mPower.rightFront = pwr;
            mPower.leftRear = pwr;
            mPower.rightRear = pwr;
            mPower.status = -1;
        }
        else
        {
            mPower.leftFront = 0;
            mPower.rightFront = 0;
            mPower.leftRear = 0;
            mPower.rightRear = 0;
            mPower.status = 0;
        }
        return mPower;
    }
    /***********************************************************
     * gyroturn5
     *  startHeading  - input. heading when this "state" started
     *  currHeading   - input. what is the heading when gt5 invoked
     *  newHeading    - input. Destination heading
     *  turnPwr       - input. -100 to 100, percent power. negative means counterclockwise
     *  turnTime      - input. how long this "state" has been in play
     * @return mPower - multiply the pwrSet by -1 for the starboard motor in the calling routine.
     *   Note: The gyroturn5 will quit after MAX_TURN_TIME, and will give a "bump" to increase turn
     *   power after BUMP_TIME.  The idea is to juice the power
     */
    public botMotors gyroTurn5(int startH, int currH, int newH, float pwrSet, float turnTime)
    {
        botMotors mPower = new botMotors();
        int accumTurn = Math.abs(startH - currH);
        accumTurn = (accumTurn > 360)? (360-accumTurn):accumTurn;
        int cw = (pwrSet < 0)? -1: 1;
        int transit = (((currH > newH) && (cw > 0)) ||
                ((currH < newH) && (cw < 0))) ?  360 : 0;
        int desiredRotation = Math.abs(transit + (cw*newH) + ((-1*cw)*currH));
        desiredRotation = (desiredRotation > 360) ? desiredRotation - 360 : desiredRotation;
        if((accumTurn < desiredRotation) && (turnTime < MAX_TURN_TIME)){
            pwrSet = (turnTime > BUMP_TIME)? pwrSet: (float)1.0;
        }
        else
        {
            pwrSet = 0;
        }
        mPower.leftFront = pwrSet;
        mPower.rightFront = pwrSet;
        mPower.leftRear = pwrSet;
        mPower.rightRear = pwrSet;
        return mPower;
    }

    /*****************************************************
     * Fwd5 - a simplified forward Drive for autonomous
     * double traveled
     * double goal
     * float pwr
     * double duration
     * @return - commanded power.  Do this once for each motor
     */




}
