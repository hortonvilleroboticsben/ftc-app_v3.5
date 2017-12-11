package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static android.content.ContentValues.TAG;
import static org.opencv.imgproc.Imgproc.minEnclosingCircle;


class StateMachine_v5 extends Subroutines_v12 {

    public String flag = "";
    int current_number = 0;                           //The order of a specific state//
    int state_in_progress = 1;                        //Which state in the list is to be run//
    boolean stateComplete = false;                    //DRIVE and TURN: Whether state_type:DRIVE or state_type:TURN is completed with assigned motion//
    private boolean oneShot = false;
    private final double gearRatio = 1.0;
    private final double countsPerRev = 560;
    private final double wheelDiameter = 4.166666666666667;                      //double wheelDiameter = 4.19;
    private final double turnDiameter = 15.45;        // private final double turnDiameter = 13.95;

    VuforiaLocalizer vl;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    BallDetector.BallColor[] ballArray = {null, null};


    @Override
    public String toString() {
        return "cn:" + current_number + "\tSIP:" + state_in_progress + "\tFlag:" + flag;
    }

    boolean waitHasFinished(long milliseconds) {
        boolean returnVal = false;

        if (initOS) {
            systemTime = System.nanoTime() / 1000000;
            initOS = false;
        } else if ((System.nanoTime() / 1000000) - systemTime >= milliseconds) {
            initOS = true;
            returnVal = true;
        }

        return returnVal;
    }

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
    }

    void initializeMachine(StateMachine_v5 object) {
        object.current_number = 0;
        object.stateComplete = false;
    }

    void incrementState(StateMachine_v5 obj) {
        obj.state_in_progress++;
        obj.stateComplete = true;
    }

    void SetFlag(StateMachine_v5 object, StateMachine_v5 receiver, String key) {
        if (next_state_to_execute(object)) {
            receiver.flag = key;
            incrementState(object);
        }
    }

    void WaitForFlag(StateMachine_v5 object, String key) {
        if (next_state_to_execute(object)) {
            if (object.flag.equals(key)) {
                incrementState(object);
            }
        }
    }

    void ClearFlag(StateMachine_v5 object) {
        if (next_state_to_execute(object)) {
            object.flag = "";
            incrementState(object);
        }
    }

    //manages current state number and compares it to state in progress
    boolean next_state_to_execute(StateMachine_v5 object) {
        object.current_number++;
        return (object.state_in_progress == object.current_number && !object.stateComplete);
    }

    boolean previous_state_running(StateMachine_v5 object) {
        return object.state_in_progress == object.current_number;
    }

    boolean next_state_to_execute(StateMachine_v5 object, boolean filler) {
        object.current_number++;
        return (object.state_in_progress == object.current_number);
    }

    void Drive(StateMachine_v5 object, double distance, double speed) {
        if (next_state_to_execute(object)) {
            double wheelCircumference = wheelDiameter * Math.PI;
            double revs = distance / wheelCircumference;
            double targetDegrees = gearRatio * revs * countsPerRev;

            if (speed < 0) {
                targetDegrees = Math.abs(targetDegrees) * -1;
            }

            if (!driveFinished) {
                run_drive_to_position();
                set_drive_target((int) targetDegrees, (int) targetDegrees);
                set_drive_power(speed, speed);
            }

            if (have_drive_encoders_reached(targetDegrees, targetDegrees) || driveFinished) { //if move is finished
                if (!driveFinished) reset_drive_encoders(); //if encoders have not been reset,
                driveFinished = true;                       //reset encoders
                set_drive_power(-0.0f, -0.0f);//stop robot
                if (have_drive_encoders_reset()) {//if encoders have actually reset,
                    driveFinished = false;  //move on to next method
                    incrementState(object);
                }
            }
        }
    }

    void Turn(StateMachine_v5 object, double degrees, double speed) {
        if (next_state_to_execute(object)) {
            double wheelCircumference = wheelDiameter * Math.PI;//
            double turnCircumference = turnDiameter * Math.PI;
            double turnDistance = turnCircumference / wheelCircumference;
            double degreeValue = turnDistance / 360;
            double revs = degreeValue * Math.abs(degrees);
            double targetDegrees = gearRatio * revs * countsPerRev;
            //double rampSpeed = rampTurn(targetDegrees, speed);

            if (degrees < 0) {
                if (!driveFinished) {
                    run_drive_to_position();
                    set_drive_target((int) -targetDegrees, (int) targetDegrees);
                    set_drive_power(-speed, speed);
                }

                if (have_drive_encoders_reached(-targetDegrees, targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            } else if (degrees > 0) {
                if (!driveFinished) {
                    run_drive_to_position();
                    set_drive_target((int) targetDegrees, (int) -targetDegrees);
                    set_drive_power(speed, -speed);
                }

                if (have_drive_encoders_reached(targetDegrees, -targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            }
        }
    }

    void OWTurn(StateMachine_v5 object, double degrees, double speed) {
        if (next_state_to_execute(object)) {
            double wheelCircumference = wheelDiameter * Math.PI;//
            double turnCircumference = turnDiameter * 2 * Math.PI;
            double turnDistance = turnCircumference / wheelCircumference;
            double degreeValue = turnDistance / 360;
            double revs = degreeValue * Math.abs(degrees);
            double targetDegrees = gearRatio * revs * countsPerRev;

            if (speed < 0) targetDegrees *= -1;

            if (degrees < 0) {
                if (!driveFinished) {
                    run_using_encoder(mtrRightDrive);
                    run_to_position(mtrLeftDrive);
                    set_encoder_target(mtrLeftDrive, (int) targetDegrees);
                    set_power(mtrLeftDrive, speed);
                    set_power(mtrRightDrive, 0);
                }

                if (has_encoder_reached(mtrLeftDrive, (int) targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            } else if (degrees > 0) {
                if (!driveFinished) {
                    run_using_encoder(mtrLeftDrive);
                    run_to_position(mtrRightDrive);
                    set_encoder_target(mtrRightDrive, (int) targetDegrees);
                    set_power(mtrRightDrive, speed);
                    set_power(mtrLeftDrive, 0);
                }

                if (has_encoder_reached(mtrRightDrive, (int) targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            }
        }
    }

    public void GyroTurn(StateMachine_v5 obj, double degrees, double speed) {
        if (next_state_to_execute(obj)) {
            DcMotor.RunMode r = mtrLeftDrive.getMode();
            run_using_drive_encoders();
            Orientation o = IMUnav.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double z = o.thirdAngle;
            telemetry.addData("rot", z);
            if (Math.abs(z) + 1 <= Math.abs(degrees)) {
                set_drive_power(-speed * Math.signum(degrees), speed * Math.signum(degrees));
            } else {
                set_drive_power(0, 0);
                reset_drive_encoders();
                mtrLeftDrive.setMode(r);
                mtrRightDrive.setMode(r);
                incrementState(obj);
            }
        }
    }

    void Pause(StateMachine_v5 object, long milliseconds) {
        if (next_state_to_execute(object)) {
            if (waitHasFinished(milliseconds)) {
                incrementState(object);
            }
        }
    }

    void ResetDrive(StateMachine_v5 object) {
        if (next_state_to_execute(object)) {
            reset_drive_encoders();
            if (get_encoder_count(mtrLeftDrive) == 0 && get_encoder_count(mtrRightDrive) == 0) {
                run_using_drive_encoders();
                incrementState(object);
            }
        }
    }

    public void ResetEncoder(StateMachine_v5 object, DcMotor motor) {
        if (next_state_to_execute(object)) {
            reset_encoders(motor);
            if (get_encoder_count(motor) == 0) {
                run_using_encoder(motor);
                incrementState(object);
            }
        }
    }

    public void Stop(StateMachine_v5 object) {
        if (next_state_to_execute(object)) {
            set_drive_power(0.0f, 0.0f);
            incrementState(object);
        }
    }

    public void ServoMove(StateMachine_v5 object, Servo servo, double position) {
        if (next_state_to_execute(object)) {
            set_position(servo, position);
            incrementState(object);
        }
    }

    public void ServoMove(StateMachine_v5 object, CRServo CRservo, double position) {
        if (next_state_to_execute(object)) {
            set_position(CRservo, position);
            incrementState(object);
        }
    }

    public void WriteI2C(StateMachine_v5 object, I2cDevice device, I2cAddr address, int register, int value) {
        if (next_state_to_execute(object)) {

            device.enableI2cWriteMode(address, register, value);
            if (device.isI2cPortInWriteMode()) {

            }

        }

    }

    public void WriteColorValues(StateMachine_v5 object, ColorSensor colorSensor) {
        if (next_state_to_execute(object)) {
            String colorVal = getColorVal(colorSensor, "red") + ", " + getColorVal(colorSensor, "blue")
                    + ", " + getColorVal(colorSensor, "green");
            writeToFile(colorVal);
        }
    }

    public void MotorMove(StateMachine_v5 s, DcMotor motor, long encCount, double power) {
        if (next_state_to_execute(s)) {
            DcMotor.RunMode r = motor.getMode();
            run_to_position(motor);
            set_encoder_target(motor, (int) encCount);
            set_power(motor, power);
            if (has_encoder_reached(motor, encCount)) {
                set_power(motor, 0);
                reset_encoders(motor);
                motor.setMode(r);
                incrementState(s);
            }
        }
    }

    public void ScanPattern(StateMachine_v5 obj){
        if(next_state_to_execute(obj)){
            ForLoop for1 = new ForLoop();
            for1.loop(1, for1.counter <= 3, 1, new ForLoop.Body() {
                @Override
                public void run() {
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                }
            });
            if(for1.hasFinished) incrementState(obj);
        }
    }

    public void ScanJewels(StateMachine_v5 obj){
        if(next_state_to_execute(obj)){
            BallDetector b1 = new BallDetector();
            b1.canRun = true;
            ForLoop f1 = new ForLoop();
            f1.loop(1, f1.counter <= 3, 1, new ForLoop.Body() {
                @Override
                public void run() {

                }
            });
            if(f1.hasFinished) {
                b1.canRun = false;
                incrementState(obj);
            }
        }
    }

    public void FlipArm(StateMachine_v5 s, long encCount, double power) {
        if (next_state_to_execute(s)) {
            run_to_position(mtrArmFlip);
            set_encoder_target(mtrArmFlip, (int) encCount);
            set_power(mtrArmFlip, power);
            if (!mtrArmFlip.isBusy()) {
                set_power(mtrArmFlip, 0);
                incrementState(s);
            }
        }
    }

    public int x = 0;
    public int y = 0;
    public int X = 0;
    public int Y = 0;
    public int x_tol = 0;
    public int y_tol = 0;
    public int counter = 0;
    public boolean OS = false;
    public boolean broken = false;
    public int wave = 0;

    void resetVariables(StateMachine_v5 object) {
        object.x = 0;
        object.y = 0;
        object.X = 0;
        object.Y = 0;
        object.counter = 0;
        object.OS = false;
        object.x_tol = 0;
        object.y_tol = 0;
        object.broken = false;
    }

    void resetMachine(StateMachine_v5 object) {
        resetVariables(object);
        object.wave = 0;
        object.current_number = 0;
        object.state_in_progress = 1;
    }

    int stringVal(String string, int val) {
        char stringVal = string.charAt(val);
        String brokenString = "" + stringVal;
        return Integer.parseInt(brokenString);
    }


}

class BallDetector extends StateMachine_v5 implements CameraBridgeViewBase.CvCameraViewListener2 {
    boolean canRun = false;

    private Mat mRgba;
    private Mat mROI;
    private ColorBlobDetector mDetectorRed;
    private ColorBlobDetector mDetectorBlue;
    //private Mat mSpectrumRed;
    //private Mat mSpectrumBlue;
    private Rect rROI;
    private Point centerROI;
    private int colorSelectCount = 0;

    private Mat mSpectrumRed;
    private Mat mSpectrumBlue;
    private Size SPECTRUM_SIZE;

    private Scalar CONTOUR_COLOR_RED;
    private Scalar CONTOUR_COLOR_BLUE;

    private Scalar ROI_COLOR;

    private Scalar mBlobColorRgba;
    //private Scalar mBlobColorHsv;

    public BallDetector() {
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);

        rROI = new Rect((int) width / 3, (int) height / 3, (int) width / 3, (int) height / 3);
        centerROI = new Point(rROI.width / 2, rROI.height / 2);
        mROI = new Mat(mRgba, rROI);

        mDetectorRed = new ColorBlobDetector();
        mDetectorBlue = new ColorBlobDetector();

        mSpectrumRed = new Mat();
        mSpectrumBlue = new Mat();

        mBlobColorRgba = new Scalar(255);

        SPECTRUM_SIZE = new Size(200, 64);

        CONTOUR_COLOR_RED = new Scalar(255, 0, 0, 255);
        CONTOUR_COLOR_BLUE = new Scalar(0, 0, 255, 255);

        ROI_COLOR = new Scalar(255, 255, 255, 255);

        mDetectorRed.setColorRange(new Scalar(237, 120, 130, 0), new Scalar(20, 255, 255, 255));
        mDetectorBlue.setColorRange(new Scalar(125, 120, 130, 0), new Scalar(187, 255, 255, 255));
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        if (canRun) {
            mRgba = inputFrame.rgba();
            mROI = new Mat(mRgba, rROI);

            double radiusRedBest = 0.0;
            double radiusBlueBest = 0.0;

            Imgproc.blur(mROI, mROI, new Size(20, 20));

            mDetectorRed.process(mROI);
            mDetectorBlue.process(mROI);

            List<MatOfPoint> contoursRed = mDetectorRed.getContours();
            Log.e(TAG, "Red Contours count: " + contoursRed.size());

            List<MatOfPoint> contoursBlue = mDetectorBlue.getContours();
            Log.e(TAG, "Blue Contours count: " + contoursBlue.size());

            List<Moments> muRed = new ArrayList<Moments>(contoursRed.size());

            Point centerRedBest = null;

            List<MatOfPoint2f> contours2fRed = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint2f> polyMOP2fRed = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint> polyMOPRed = new ArrayList<MatOfPoint>();

            float[] radiusRed = new float[contoursRed.size()];

            for (int i = 0; i < contoursRed.size(); i++) {
                Point centerRed = new Point();
                muRed.add(i, Imgproc.moments(contoursRed.get(i), false));

                contours2fRed.add(new MatOfPoint2f());
                polyMOP2fRed.add(new MatOfPoint2f());
                polyMOPRed.add(new MatOfPoint());

                contoursRed.get(i).convertTo(contours2fRed.get(i), CvType.CV_32FC2);
                Imgproc.approxPolyDP(contours2fRed.get(i), polyMOP2fRed.get(i), 3, true);
                polyMOP2fRed.get(i).convertTo(polyMOPRed.get(i), CvType.CV_32S);

                minEnclosingCircle(polyMOP2fRed.get(i), centerRed, radiusRed);
                Imgproc.circle(mRgba, new Point(centerRed.x + rROI.x, centerRed.y + rROI.y), 16, CONTOUR_COLOR_RED, 16);

                if (centerRedBest == null) {
                    centerRedBest = centerRed;
                    radiusRedBest = radiusRed[0];
                } else {
                    if (distance(centerROI, centerRed) < distance(centerROI, centerRedBest)) {
                        centerRedBest = centerRed;
                        radiusRedBest = radiusRed[0];
                    }
                }
            }

            List<Moments> muBlue = new ArrayList<Moments>(contoursBlue.size());
            Point centerBlueBest = null;

            List<MatOfPoint2f> contours2fBlue = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint2f> polyMOP2fBlue = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint> polyMOPBlue = new ArrayList<MatOfPoint>();

            float[] radiusBlue = new float[contoursBlue.size()];

            for (int i = 0; i < contoursBlue.size(); i++) {
                Point centerBlue = new Point();
                muBlue.add(i, Imgproc.moments(contoursBlue.get(i), false));

                contours2fBlue.add(new MatOfPoint2f());
                polyMOP2fBlue.add(new MatOfPoint2f());
                polyMOPBlue.add(new MatOfPoint());

                contoursBlue.get(i).convertTo(contours2fBlue.get(i), CvType.CV_32FC2);
                Imgproc.approxPolyDP(contours2fBlue.get(i), polyMOP2fBlue.get(i), 3, true);
                polyMOP2fBlue.get(i).convertTo(polyMOPBlue.get(i), CvType.CV_32S);

                minEnclosingCircle(polyMOP2fBlue.get(i), centerBlue, radiusBlue);
                Imgproc.circle(mRgba, new Point(centerBlue.x + rROI.x, centerBlue.y + rROI.y), 16, CONTOUR_COLOR_BLUE, 16);

                if (centerBlueBest == null) {
                    centerBlueBest = centerBlue;
                    radiusBlueBest = radiusBlue[0];
                } else {
                    if (distance(centerROI, centerBlue) < distance(centerROI, centerBlueBest)) {
                        centerBlueBest = centerBlue;
                        radiusBlueBest = radiusBlue[0];
                    }
                }
            }

            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
            colorLabel.setTo(mBlobColorRgba);

            if (centerRedBest != null && centerBlueBest != null) {
                String message = "";
                if (centerBlueBest.x < centerRedBest.x) {
                    ballArray[0] = BallColor.BLUE;
                    ballArray[1] = BallColor.RED;
                } else {
                    ballArray[0] = BallColor.RED;
                    ballArray[1] = BallColor.BLUE;
                }
            }
        }
        return mRgba;
    }

    private double distance(Point center, Point check) {
        return Math.hypot((center.x - check.x), (center.y - check.y));
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }

    enum BallColor {
        RED, BLUE;
    }
}

class ColorBlobDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);

    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;

    // Color radius for range checking in HSV color space with touch color
    private Scalar mColorRadius = new Scalar(25, 50, 50, 0);

    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();


    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public void setColorRange(Scalar minHSV, Scalar maxHSV) {
        mLowerBound = minHSV;
        mUpperBound = maxHSV;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0] - mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0] + mColorRadius.val[0] <= 255) ? hsvColor.val[0] + mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        //mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        //mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];
        mLowerBound.val[1] = 0;
        mUpperBound.val[1] = 255;

        //mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        //mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];
        mLowerBound.val[2] = 0;
        mUpperBound.val[2] = 255;

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;

        Log.i(TAG, "Set HSV Color Min: (" +
                mLowerBound.val[0] + "," +
                mLowerBound.val[1] + "," +
                mLowerBound.val[2] + ")");

        Log.i(TAG, "Set HSV Color Max: (" +
                mUpperBound.val[0] + "," +
                mUpperBound.val[1] + "," +
                mUpperBound.val[2] + ")");

        Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);

        for (int j = 0; j < maxH - minH; j++) {
            byte[] tmp = {(byte) (minH + j), (byte) 255, (byte) 255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void setMinContourArea(double area) {
        mMinContourArea = area;
    }

    public void process(Mat rgbaImage) {
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        if (mUpperBound.val[0] < mLowerBound.val[0]) {

            Mat mMask1 = new Mat();
            Mat mMask2 = new Mat();


            Scalar tLowerBound = mLowerBound.clone();
            Scalar tUpperBound = mUpperBound.clone();

            tLowerBound.val[0] = 0.0;
            tUpperBound.val[0] = mUpperBound.val[0];

            Core.inRange(mHsvMat, tLowerBound, tUpperBound, mMask1);

            tLowerBound = mLowerBound.clone();
            tUpperBound.val[0] = 255.0;

            Core.inRange(mHsvMat, tLowerBound, tUpperBound, mMask2);

            Core.add(mMask1, mMask2, mMask);

        } else {
            Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        }

        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }

        // Filter contours by area and resize to fit the original image size
        mContours.clear();
        each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
                Core.multiply(contour, new Scalar(4, 4), contour);
                mContours.add(contour);
            }
        }
    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }
}

class ForLoop{
    int counter = 0;
    boolean hasFinished = false;
    private boolean os = false;

    public ForLoop(){
        reset();
    }

    interface Body{
        abstract void run();
    }

    public void loop(int startVal, boolean check, int change, Body body){
        if(!os) counter = startVal;
        os = true;

        if(check){
            body.run();
            startVal+=change;
        }else hasFinished = true;
    }

    public void reset(){
        counter = 0;
        hasFinished = false;
    }
}