//0x3c - left
//0x6c - right
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

import java.util.Arrays;

/**
 * Created by sam on 11/30/2017.
 */
@Autonomous(name = "Autonomous", group = "Competition")
public class Autonomous_v1 extends StateMachine_v5 {

    StateMachine_v5 dt = new StateMachine_v5();
    StateMachine_v5 glyph = new StateMachine_v5();
    StateMachine_v5 arm = new StateMachine_v5();

    VuforiaLocalizer vl;
    VuforiaTrackable relicTemplate;

    Orientation axes;

    double adjustment = 0;

    final byte BLUE = 1;
    final byte RED = 2;
    final double CLAW_OPEN = 1;
    final double CLAW_HALF = .4;
    final double CLAW_CLOSE = .1;


    byte Alliance = 0;
    int StartPos = 0;


    int question_number = 1;
    boolean btnOS = false;
    boolean readyL = false, readyR = false;

    @Override
    public void init() {
        super.init();
        mtrLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void init_loop() {
//        addIMUSensor("imu");
        axes = IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("Setup", Arrays.toString(new double[]{axes.firstAngle, axes.secondAngle, axes.thirdAngle}));
        telemetry.addData("SetupOK", (int) Math.abs(axes.firstAngle) <= 1 && (int) Math.abs(axes.secondAngle) <= 1 && (int) Math.abs(axes.thirdAngle) <= 1);
        srvGr2.setPosition(GR2OPEN);
        switch (question_number) {
            case 1:
                telemetry.addData("Alliance: a=blue b=red", "");
                if ((gamepad1.a || gamepad1.b) && btnOS == false) {
                    Alliance = (gamepad1.a) ? BLUE : RED;
                    btnOS = true;
                } else if (!gamepad1.a && !gamepad1.b && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 2:
                telemetry.addData("StartPos: a=1 b=2", "");
                if ((gamepad1.a || gamepad1.b) && btnOS == false) {
                    StartPos = (gamepad1.a) ? 1 : 2;
                    btnOS = true;
                } else if (!gamepad1.a && !gamepad1.b && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 3:
                telemetry.addData("Ready to Begin", "");
                break;
        }

        if (StartPos == 0 || Alliance == 0)
            telemetry.addData("WARNING YOU DON'T HAVE ALL INITIALIZATION PARAMETERS IN", "");

    }

    @Override
    public void loop() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        initializeMachine(dt);
        initializeMachine(glyph);
        initializeMachine(arm);

        double pos = Math.abs(((get_encoder_count(mtrArmFlip) * 360 / 1700.)) / 360.);
        pos += adjustment;
        pos = pos >= 1 ? 1 : pos;
        pos = pos <= 0 ? 0 : pos;
        set_position(srvLevel, pos);

        //TODO:Scan pictograph and ball colors
        if (Alliance == BLUE) {
            if (StartPos == 1) {

                srvGr2.setPosition(GR2CLOSED);
                Pause(glyph, 150);
                MotorMove(glyph, mtrLift, 1200, 0.3);

                Pause(dt, 350);
                Drive(dt, 27, 0.2);
                SetFlag(dt, arm, "off platform");
                SetFlag(dt, glyph, "off platform");
                //ScanPattern(dt);
                //ScanJewels(dt);
                Turn(dt, 174.6, 0.2);
                Drive(dt, 3.7, 0.2);
                FlipArm(dt, -1600, 0.21);
                SetFlag(dt, arm, "extended");
                Drive(dt, 1.5, 0.2);

                WaitForFlag(arm, "off platform");
                MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * .4105), 0.5);
                ServoMove(arm, srvExtend, .95);
                Pause(arm, 4010);
                SetFlag(arm, glyph, "extended");
                WaitForFlag(arm, "extended");
                if (ballArray[0].equals(BallColor.BLUE)) {
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * .05), 0.5);
                    Pause(arm, 500);
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * -.04), 0.5);
                } else {
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * -.05), 0.5);
                    Pause(arm, 500);
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * .04), 0.5);
                }
                Pause(arm, 1555);
                //SetFlag(arm,glyph,"start moving");
                ServoMove(arm, srvExtend, 0);

                WaitForFlag(glyph, "off platform");
                Pause(glyph, 600);
                ServoMove(glyph, srvExtend, 0.25);
                WaitForFlag(glyph, "extended");
                if (next_state_to_execute(glyph)) {
                    adjustment += .5;
                    incrementState(glyph);
                }
                //WaitForFlag(glyph,"start moving");
                ServoMove(glyph, srvClaw, CLAW_HALF);

                //TODO:Add hitting logic
                //TODO:Remove this (just testing)

                //Pause(arm, 5000);
                if (next_state_to_execute(arm)) {
                    SetFlag(new StateMachine_v5(), dt, "hit");
                    SetFlag(new StateMachine_v5(), arm, "hit");
                    SetFlag(new StateMachine_v5(), glyph, "hit");
                    incrementState(arm);
                }

                WaitForFlag(arm, "hit");
                WaitForFlag(dt, "hit");
                WaitForFlag(glyph, "hit");


                SetFlag(arm, dt, "continue");

                WaitForFlag(dt, "continue");
                Drive(dt, -3, -0.2);
                //FlipArm(dt, 0, -.2);

                if (next_state_to_execute(arm)) {
                    adjustment = 0;
                    incrementState(arm);
                }

                ServoMove(arm, srvClaw, CLAW_CLOSE);
                ServoMove(arm, srvExtend, -1);
                Pause(arm, 2100);
                SetFlag(arm, dt, "retracting");
                Pause(arm, 3500);
                ServoMove(arm, srvExtend, 0);
                MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * -0.4), 0.4);

                WaitForFlag(dt, "retracting");
                Turn(dt, 90, .2);
                SetFlag(dt, glyph, "lower lift");
                WaitForFlag(glyph, "lower lift");
                MotorMove(glyph, mtrLift, -900, 0.3);
                OWTurn(dt, 34, -.2);
                Drive(dt, -32, -.4);
                Turn(dt, 61, .2);
                if (next_state_to_execute(dt)) {

                    if (snsColorLeft.blue() < 10) {
                        mtrLeftDrive.setPower(0.15);
                    } else {
                        mtrLeftDrive.setPower(0.0);
                        readyL = true;
                    }
                    if (snsColorRight.blue() < 10) {
                        mtrRightDrive.setPower(0.15);
                    } else {
                        mtrRightDrive.setPower(0.0);
                        readyR = true;
                    }
                    if (readyR && readyL) {
                        incrementState(dt);
                    }
                }

                if (vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                    OWTurn(dt, 40, 0.3);
                } else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                    OWTurn(dt, -40, 0.3);
                } else Drive(dt, 4, 0.3);
                ServoMove(dt, srvGr2, GR2OPEN);
                Drive(dt, -3, 0.3);
            }
        }
        //MotorMove(glyph,mtrLift,-300,0.3);
        if (Alliance == RED) {
            if (StartPos == 1) {

                srvGr2.setPosition(GR2CLOSED);
                Pause(glyph, 150);
                MotorMove(glyph, mtrLift, 1200, 0.3);

                Pause(dt, 350);
                Drive(dt, 27, 0.2);
                SetFlag(dt, arm, "off platform");
                SetFlag(dt, glyph, "off platform");
                //ScanPattern(dt);
                //ScanJewels(dt);
                Turn(dt, -174.6, 0.2);
                Drive(dt, 3.7, 0.2);
                FlipArm(dt, -1600, 0.21);
                SetFlag(dt, arm, "extended");
                Drive(dt, 1.5, 0.2);

                WaitForFlag(arm, "off platform");
                MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * .4105), 0.5);
                ServoMove(arm, srvExtend, .95);
                Pause(arm, 4010);
                SetFlag(arm, glyph, "extended");
                WaitForFlag(arm, "extended");
                if (ballArray[0].equals(BallColor.RED)) {
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * .05), 0.5);
                    Pause(arm, 500);
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * -.04), 0.5);
                } else {
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * -.05), 0.5);
                    Pause(arm, 500);
                    MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * .04), 0.5);
                }
                Pause(arm, 1555);
                //SetFlag(arm,glyph,"start moving");
                ServoMove(arm, srvExtend, 0);

                WaitForFlag(glyph, "off platform");
                Pause(glyph, 600);
                ServoMove(glyph, srvExtend, 0.25);
                WaitForFlag(glyph, "extended");
                if (next_state_to_execute(glyph)) {
                    adjustment += .5;
                    incrementState(glyph);
                }
                //WaitForFlag(glyph,"start moving");
                ServoMove(glyph, srvClaw, CLAW_HALF);

                //TODO:Add hitting logic
                //TODO:Remove this (just testing)

                //Pause(arm, 5000);
                if (next_state_to_execute(arm)) {
                    SetFlag(new StateMachine_v5(), dt, "hit");
                    SetFlag(new StateMachine_v5(), arm, "hit");
                    SetFlag(new StateMachine_v5(), glyph, "hit");
                    incrementState(arm);
                }

                WaitForFlag(arm, "hit");
                WaitForFlag(dt, "hit");
                WaitForFlag(glyph, "hit");


                SetFlag(arm, dt, "continue");

                WaitForFlag(dt, "continue");
                Drive(dt, -3, -0.2);
                //FlipArm(dt, 0, -.2);

                if (next_state_to_execute(arm)) {
                    adjustment = 0;
                    incrementState(arm);
                }

                ServoMove(arm, srvClaw, CLAW_CLOSE);
                ServoMove(arm, srvExtend, -1);
                Pause(arm, 2100);
                SetFlag(arm, dt, "retracting");
                Pause(arm, 3500);
                ServoMove(arm, srvExtend, 0);
                MotorMove(arm, mtrArmSpin, (int) (1680 * 4.75 * -0.4), 0.4);

                WaitForFlag(dt, "retracting");
                Turn(dt, -90, .2);
                SetFlag(dt, glyph, "lower lift");
                WaitForFlag(glyph, "lower lift");
                MotorMove(glyph, mtrLift, -900, 0.3);
                OWTurn(dt, -34, -.2);
                Drive(dt, -32, -.4);
                Turn(dt, -61, .2);
                if (next_state_to_execute(dt)) {

                    if (snsColorLeft.blue() < 10) {
                        mtrLeftDrive.setPower(0.15);
                    } else {
                        mtrLeftDrive.setPower(0.0);
                        readyL = true;
                    }
                    if (snsColorRight.blue() < 10) {
                        mtrRightDrive.setPower(0.15);
                    } else {
                        mtrRightDrive.setPower(0.0);
                        readyR = true;
                    }
                    if (readyR && readyL) {
                        incrementState(dt);
                    }
                }

                if (vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                    OWTurn(dt, 40, 0.3);
                } else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                    OWTurn(dt, -40, 0.3);
                } else Drive(dt, 4, 0.3);
                ServoMove(dt, srvGr2, GR2OPEN);
                Drive(dt, -3, 0.3);
            }
        }

        telemetry.addData("rightblue", snsColorRight.blue());
        telemetry.addData("rightred", snsColorRight.red());
        telemetry.addData("leftblue", snsColorLeft.blue());
        telemetry.addData("leftred", snsColorLeft.red());
//        telemetry.addData("armFlipEnc", get_encoder_count(mtrArmFlip));
//        telemetry.addData("claw", srvClaw.getPosition());
//        telemetry.addData("dt", dt.toString());
//        telemetry.addData("arm", arm.toString());
//        telemetry.addData("glyph", glyph.toString());
    }
}
