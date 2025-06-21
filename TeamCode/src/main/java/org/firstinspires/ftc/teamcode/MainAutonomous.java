package org.firstinspires.ftc.teamcode.mainopmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class MainAutonomous extends LinearOpMode {
    private final int[][] AprilTagCoords =
            {
                    {-72, 48},
                    {0, 72},
                    {72, 48},
                    {72, -48},
                    {0, -72},
                    {-72, -48}

            };
    int[]  servoPos = {-30,30}; //open, close
    int[] armPos = {480,5}; //out, in
    int[] armRotatePos = {-1200, -800, -5}; //up, down
    private static final int CountsPerInch = 1; //NEED TO CALCULATE
    private static final int CountsPerDegree = 1; //NEED To CALCULATE
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor arm_rotate;
    private DcMotor arm_extension;
    private Servo claw_servo;
    private IMU imu;
    private final ElapsedTime timer = new ElapsedTime();
    String State = "";

    @Override
    public void runOpMode() throws InterruptedException {

        left_drive = hardwareMap.get(DcMotor.class, "left");
        right_drive = hardwareMap.get(DcMotor.class, "right");
        claw_servo = hardwareMap.get(Servo.class, "claw");
        arm_extension = hardwareMap.get(DcMotor.class, "arm");
        arm_rotate = hardwareMap.get(DcMotor.class, "armRotate");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        initAprilTag();

        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()) {
            String initPos = DetermineInitialPosition();
            switch(initPos)
            {
                case "Clip":
                    State = "ClipToSub";
                    break;
                case "Score":
                    State = "ScoreBaskets";
                    break;
            }
            telemetry.addData("state: ",State);
            telemetry.addData("left, right", "%7d :%7d", (left_drive.getCurrentPosition()), (right_drive.getCurrentPosition()));
            telemetry.update();
        }

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        waitForStart();

        while(opModeIsActive()) //TURN = positive = left
        {
            telemetry.addData("State", State);
            telemetry.addData("Heading: ", "%7f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("left, right", "%7d :%7d", (left_drive.getCurrentPosition()), (right_drive.getCurrentPosition()));
            telemetry.update();
            State = "ScoreBaskets";
            ScoringBaskets();
            requestOpModeStop();
        }
        visionPortal.close();
    }

    public void ClippingToSub() //somewhat reliable clips when started going forwards
    {
        if(opModeIsActive())
        {
            MoveClaw(true);
            MoveForward(280, 300); //265
            RotateArm(0);
            ExtendArm(false);
            RotateArm(1);
            MoveClaw(false);
            ExtendArm(true);
            RotateArm(2, 0.3);
            TurnWithEncoders(35, 200); //33
            MoveForward(-550, 300);
        }
    }

    public void ReturnToOZ() //observation zone
    {
        if(opModeIsActive())
        {
            TurnWithEncoders(-135, 0.05);
            MoveForward(100, 0.3);
            TurnWithEncoders(180, 0.05);
        }
    }

    public void ScoringBaskets()
    {
        if(opModeIsActive())
        {
            /*TurnWithEncoders(-20, 300);
            MoveForward(300, 300);
            TurnWithEncoders(35, 300);
            MoveForward(280, 400);*/

            MoveClaw(true);
            RotateArm(0);
            ExtendArm(false);
            MoveForward(60, 300);
            //RotateArm(2, 0.3);
            MoveClaw(false);
            ExtendArm(true);
            RotateArm(2);

        }
    }

    public void GoToAZ() //Ascent Zone
    {
        if(opModeIsActive())
        {
            TurnWithEncoders(-100, 0.05); //100 degrees right
            MoveForward(500, 0.3); //2m
            TurnWithEncoders(-80, 0.05); //80 degrees right
            MoveForward(100, 0.05); //0.5m
            TurnWithEncoders(180, 0.05); //180 degrees
        }
    }

    public void MoveForward(double distance, double maxSpeed) //distance in inches
    {
        if(opModeIsActive()) {
            int moveCounts = (int) (distance * CountsPerInch);
            int leftTarget = left_drive.getCurrentPosition() + moveCounts;
            int rightTarget = right_drive.getCurrentPosition() + moveCounts; //new target positions

            left_drive.setTargetPosition(leftTarget);
            right_drive.setTargetPosition(rightTarget);
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx)left_drive).setVelocity(300);
            ((DcMotorEx)right_drive).setVelocity(300);
            CheckBusy();
            if(!(leftTarget-5 < left_drive.getCurrentPosition() && left_drive.getCurrentPosition() < leftTarget + 5) || !(rightTarget-5 < right_drive.getCurrentPosition() && right_drive.getCurrentPosition() < rightTarget + 5))
            {
                MoveForward(leftTarget-left_drive.getCurrentPosition(), 0.1);
            }
        }

    }

    public void MoveCurve(double distance, double maxSpeed, int adjust, double adjustSpeed) //distance in inches
    {
        if(opModeIsActive()) {
            int moveCounts = (int) (distance * CountsPerInch);
            int leftTarget = left_drive.getCurrentPosition() + moveCounts + adjust;
            int rightTarget = right_drive.getCurrentPosition() + moveCounts - adjust; //new target positions

            left_drive.setTargetPosition(leftTarget);
            right_drive.setTargetPosition(rightTarget);
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Move(maxSpeed-adjustSpeed, adjustSpeed);
            CheckBusy();
        }

    }


    public void TurnWithEncoders(double heading, double speed) //heading between -180 nad 180 relative to robot
    {
        if(opModeIsActive())
        {
            int leftTarget, rightTarget;

            rightTarget = (int)(CountsPerDegree * heading) + right_drive.getCurrentPosition();
            leftTarget = left_drive.getCurrentPosition()-(int)(CountsPerDegree * heading);

            left_drive.setTargetPosition(leftTarget);
            right_drive.setTargetPosition(rightTarget);
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx)left_drive).setVelocity(200);
            ((DcMotorEx)right_drive).setVelocity(200);
            CheckBusy();
            //use GetHeading

        }

    }

    public void RotateArm(int p)
    {
        RotateArm(p, 0.5);
    }
    public void RotateArm(int p, double speed)
    {
        if(opModeIsActive())
        {
            arm_rotate.setTargetPosition(armRotatePos[p]);
            arm_rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(p == 2)
            {
                arm_rotate.setPower(speed);
            }
            else
            {
                arm_rotate.setPower(-speed);
            }
            while(arm_rotate.isBusy()){}
        }
    }

    public void ExtendArm(boolean in)
    {
        if(opModeIsActive())
        {
            arm_extension.setTargetPosition(armPos[(in ? 1 : 0)]);
            arm_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(in)
            {
                arm_extension.setPower(-0.5);
            }
            else
            {
                arm_extension.setPower(0.5);
            }
            while(arm_extension.isBusy()){}
        }
    }

    public void MoveClaw(boolean close)
    {
        claw_servo.setPosition(DegreesToPos(servoPos[(close ? 1 : 0)]));
        timer.reset();
        while(timer.seconds() < 0.5){}
    }
    public void Move(double drive, double turn)
    {
        if(opModeIsActive())
        {
            double leftSpeed = drive + turn;
            double rightSpeed = drive - turn;

            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            left_drive.setPower(leftSpeed);
            right_drive.setPower(rightSpeed);
        }

    }

    public double GetHeading(){ //-180 to 180
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void CheckBusy()
    {
        if(opModeIsActive())
        {
            while(left_drive.isBusy() || right_drive.isBusy())
            {
                telemetry.addData("State", State);
                telemetry.addData("Heading: ", "%7f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("left, right", "%7d :%7d", (left_drive.getCurrentPosition()), (right_drive.getCurrentPosition()));
                telemetry.update();
            }
            while(arm_rotate.isBusy() || arm_extension.isBusy())
            {
                telemetry.addData("State", State);
                telemetry.addData("Heading: ", "%7f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("rotate, extension", "%7d :%7d", (arm_rotate.getCurrentPosition()), (arm_extension.getCurrentPosition()));
                telemetry.update();
            }
            timer.reset();
            while(timer.seconds() < 1){}
            Move(0,0);
            timer.reset();
            while(timer.seconds() < 0.5){}
        }

    }

    public String DetermineInitialPosition()
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection d:currentDetections)
        {
            if(d.id == 13 || d.id == 16)
            {
                return "Score";
            }
        }
        return "Clip";
    }


    private void initAprilTag() //inches + radians
    {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        builder.addProcessor(aprilTag);

        visionPortal = builder.build();


    }


    private double DegreesToPos(double degrees) //0 = -135, 1 = 135
    {
        return (degrees + 135)/270;
    }

    public double NormaliseAngle(double angle) //returns angle between 180 and -180
    {
        if(angle > 180)
        {
            angle -= 360;
        }
        else if(angle < -180)
        {
            angle += 360;
        }
        return angle;

    }




}
