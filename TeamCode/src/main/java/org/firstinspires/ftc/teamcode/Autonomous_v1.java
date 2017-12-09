package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;

/**
 * Created by sam on 11/30/2017.
 */
@Autonomous(name = "AutonV1", group = "Testing")
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
    final double CLAW_HALF = .75;
    final double CLAW_CLOSE = .5;
    byte Alliance = BLUE;
    int StartPos = 1;
    int ballPos = 1;

    int question_number = 1;
    boolean btnOS = false;

    @Override
    public void init() {
        super.init();
        int vuforiaID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(vuforiaID);
        params.vuforiaLicenseKey = "AXlr9dj/////AAAAGRujCnGOL0aIpjtd4y5IK2sFwI4jstOeOlytTrPr3jzeQQ9tGEHgLmuGdxzuxGNY5641pyXeeyJHccL+I4QCZq8Sodm5DAUBsAQQ9ox1EY3+KNfZISN06k1IqDf7YaRXhE02j+7aE4Apnm3Hvn9V5CDKSTgOq73eJId9uzHkuNaIx+UDV4fRS1HK5L6dSGmIw3+RW7uaFdBF0E2bvWzzpZv51KFzw5oy/9qFB9r6ke5B5Gd2zw9JjafwudFSpLVCiNzIreoTaIFYVMmGMuIFIT4C6oC13EcvbNl5CFpl+irLqhSI//nlL2L2DKxKtW5UTQqNBlOSBdTxWR/kSN12edlwOu0kFgzhKBFapn3KHC0V";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vl = ClassFactory.createVuforiaLocalizer(params);
        VuforiaTrackables relicTrackables = this.vl.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();

        mtrLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void init_loop() {
//        addIMUSensor("imu");
        axes = IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("Setup", Arrays.toString(new double[] {axes.firstAngle, axes.secondAngle, axes.thirdAngle}));
        telemetry.addData("SetupOK", (int)Math.abs(axes.firstAngle) <= 1 && (int)Math.abs(axes.secondAngle) <= 1 && (int)Math.abs(axes.thirdAngle) <= 1);

        switch(question_number) {
            case 1:
                telemetry.addData("Alliance: a=blue b=red","");
                if((gamepad1.a || gamepad1.b) && btnOS == false) {
                    Alliance = (gamepad1.a) ? BLUE: RED;
                    btnOS = true;
                }else if(!gamepad1.a && !gamepad1.b && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 2:
                telemetry.addData("StartPos: a=1 b=2","");
                if((gamepad1.a || gamepad1.b) && btnOS == false) {
                    StartPos = (gamepad1.a) ? 1 : 2;
                    btnOS = true;
                }else if(!gamepad1.a && !gamepad1.b && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 3:
                telemetry.addData("Ballcolor: a=blue b=red","");
                if((gamepad1.a || gamepad1.b) && btnOS == false) {
                    ballPos = (gamepad1.a) ? 1 : 2;
                    btnOS = true;
                }else if(!gamepad1.a && !gamepad1.b && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 4:
                telemetry.addData("Ready to Begin" ,"");
                break;
        }


    }

    @Override
    public void loop() {
        //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        initializeMachine(dt);
        initializeMachine(glyph);
        initializeMachine(arm);

        double pos = Math.abs(((get_encoder_count(mtrArmFlip) * 360 / 1700.) % 360) / 360.);
        pos += adjustment;
        pos = pos >= 1 ? 1 : pos;
        set_position(srvLevel, pos);

        //TODO:Scan pictograph and ball colors

        Drive(dt,27,0.2);
        SetFlag(dt,arm,"off platform");
        Turn(dt, 177.5,0.2);
        Drive(dt, 4.25, 0.2);

        WaitForFlag(arm,"off platform");
        MotorMove(arm,mtrArmSpin,(int)(1680*4.75 * .4),0.5);

        ServoMove(arm,srvExtend,1);
        Pause(arm, 5000);
        ServoMove(arm, srvExtend, 0);

        FlipArm(dt, -1385, 0.2);
        SetFlag(dt, arm, "extended");

        WaitForFlag(arm, "extended");
        //TODO:Add hitting logic
        //TODO:Remove this (just testing)
        if(next_state_to_execute(new StateMachine_v5())){
            if(gamepad1.x){
                SetFlag(new StateMachine_v5(), dt, "hit");
                SetFlag(new StateMachine_v5(), arm, "hit");
            }
        }
        WaitForFlag(arm, "hit");
        WaitForFlag(dt, "hit");
        FlipArm(dt, 0, .2);
        ServoMove(arm, srvExtend, -1);
        Pause(arm, 1500);
        SetFlag(arm, dt, "retracting");
        Pause(arm, 3500);
        ServoMove(arm, srvExtend, 0);

        WaitForFlag(dt, "retracting");


// if (Alliance == BLUE) {
//            if (StartPos == 1) {
//                //TODO:Add in sensing.
//                Drive(dt,-22,0.8);
//                Turn(dt, 180, .5);
//                /*ServoMove(arm,srvClaw,0.5);
//                MotorMove(arm, mtrArmSpin, 100, 0.3);
//                MotorMove(arm, mtrArmFlip, 100, 0.3);*/
//                if(ballPos == 1) {
//
//                }
//                if(vuMark == RelicRecoveryVuMark.LEFT){
//                    Drive(dt, 28, .5);
//                } else if(vuMark == RelicRecoveryVuMark.CENTER){
//                    Drive(dt,  36, .5);
//                } else if(vuMark == RelicRecoveryVuMark.RIGHT) {
//                    Drive(dt, 44, .5);
//                }
//            } else if (StartPos == 2) {
//
//            }
//        }else if (Alliance == RED) {
//            if (StartPos == 1) {
//                Turn(dt, -90, .5);
//                if (vuMark == RelicRecoveryVuMark.LEFT) {
//                    Drive(dt, 44, .5);
//                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
//                    Drive(dt, 36, .5);
//                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
//                    Drive(dt, 28, .5);
//                }
//                Turn(dt, 90, .5);
//                set_position(srvClaw, .5);
//                MotorMove(dt, mtrArmSpin, 50, 0.3);
//                MotorMove(dt, mtrArmFlip, 100, 0.3);
//            } else if (StartPos == 2) {
//
//            }
//        }
        telemetry.addData("leftenc", get_encoder_count(mtrLeftDrive));
        telemetry.addData("rightenc", get_encoder_count(mtrRightDrive));
        telemetry.addData("armFlipEnc", get_encoder_count(mtrArmFlip));
        telemetry.addData("pos", pos);
        telemetry.addData("dt", dt.toString());
    }
}
