package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// TODO tune max velocity - increase it?
// Add limits

//test actions + basic Ts
//tune complex Ts + build whole path

@Config
@Autonomous
public class RRTest extends LinearOpMode
{
    public static class Params {
        public int distance = -41;
    }

    public static RRTest.Params PARAMS = new RRTest.Params();

    public class Mechanisms
    {
        private Arm arm;
        private Claw claw;
        public Mechanisms(HardwareMap hardwareMap)
        {
            arm = new Arm(hardwareMap);
            claw = new Claw(hardwareMap);
        }

        public void clip()
        {
            arm.armUpClip();
            arm.armExtendClip();
            arm.clip();
            claw.open();
            arm.armRetract();
            arm.armDown();
        }

        public void basket()
        {
            arm.armUpClip();
            arm.armExtendClip();
            claw.open();
            arm.armRetract();
            arm.armDown();
        }
    }

    public class Arm
    {
        private final int[] Extensions = {1400,5, 745}; // out, in, clipping,
        private final int[] Rotations = {1960,10, 1650, 1000}; // up, down, clipping, wall
        private DcMotorEx extendMotor;
        private  DcMotorEx rotateMotor;
        public Arm(HardwareMap hardwareMap)
        {
            extendMotor = hardwareMap.get(DcMotorEx.class, "arm");
            rotateMotor = hardwareMap.get(DcMotorEx.class, "armRotate");

            //extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public class ArmUpClip implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(!init)
                {
                    rotateMotor.setPower(0.5);
                    init = true;
                }
                double pos = rotateMotor.getCurrentPosition();
                packet.put("Rotation", pos);

                if(pos < Rotations[0])
                {
                    return true;
                }
                else
                {
                    rotateMotor.setPower(0);
                    return false;
                }
            }
        }

        public class ArmUpWall implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(!init)
                {
                    rotateMotor.setPower(0.5);
                    init = true;
                }
                double pos = rotateMotor.getCurrentPosition();
                packet.put("Rotation", pos);

                if(pos < Rotations[3])
                {
                    return true;
                }
                else
                {
                    rotateMotor.setPower(0);
                    return false;
                }
            }

        }

        public class ArmDown implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(!init)
                {
                    rotateMotor.setPower(-0.5);
                    init = true;
                }
                double pos = rotateMotor.getCurrentPosition();
                packet.put("Rotation", pos);

                if(pos > Rotations[1])
                {
                    return true;
                }
                else
                {
                    rotateMotor.setPower(0);
                    return false;
                }
            }
        }
        public class Clip implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(!init)
                {
                    rotateMotor.setPower(-0.5);
                    init = true;
                }
                double pos = rotateMotor.getCurrentPosition();
                packet.put("Rotation", pos);

                if(pos > 1400)
                {
                    return true;
                }
                else
                {
                    rotateMotor.setPower(0);
                    return false;
                }
            }
        }

        public class ArmExtendClip implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    extendMotor.setPower(0.7); // TODO
                    init = true;
                }
                double pos = extendMotor.getCurrentPosition();
                packet.put("Extension", pos);

                if (pos < Extensions[2]) {
                    return true;
                } else {
                    packet.put("finished", pos);
                    extendMotor.setPower(0);
                    return false;
                }
            }
        }
        public class ArmRetract implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    extendMotor.setPower(-0.7); // TODO
                    init = true;
                }
                double pos = extendMotor.getCurrentPosition();
                packet.put("Extension", pos);

                if (pos > Extensions[1]) {
                    return true;
                } else {
                    extendMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action armUpClip()
        {
            return new ArmUpClip();
        }
        public Action armDown()
        {
            return new ArmDown();
        }
        public Action armExtendClip()
        {
            return new ArmExtendClip();
        }
        public Action armRetract()
        {
            return new ArmRetract();
        }
        public Action clip()
        {
            return new Clip();
        }
    }

    public class Claw
    {
        private Servo claw;
        private final int[] Positions = {60,0}; //open, closed

        public Claw(HardwareMap hardwareMap)
        {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class OpenClaw implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                claw.setPosition(Positions[(0)]);
                return false;
            }
        }

        public class CloseClaw implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                claw.setPosition(Positions[(1)]);
                return false;
            }
        }

        public Action open()
        {
            return new OpenClaw();
        }
        public Action close()
        {
            return new CloseClaw();
        }


    }


    @Override
    public void runOpMode()
    {
        Pose2d initialPos = new Pose2d(9, -63, Math.toRadians(90));
        TankDrive drive = new TankDrive(hardwareMap, initialPos);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        //init trajectory stuffs
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPos)
                .lineToY(-45)
                .waitSeconds(3)
                .turn(Math.toRadians(-90))
                .lineToX(20)

                .splineTo(new Vector2d(42,-12), Math.toRadians(-90))
                .splineTo(new Vector2d(42,-47), Math.toRadians(-90))

                .waitSeconds(2)
                .splineTo(new Vector2d(9,-49),Math.toRadians(90));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh();

        Action moveToSub = tab1.build();
        Action moveToObs = tab2.build();

        telemetry.addData("Starting Position", initialPos);
        telemetry.update();

        //Actions.runBlocking(claw.close());

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        moveToSub,
                        /*arm.armUpClip(),
                        arm.armExtendClip(),
                        arm.clip(),
                        arm.armRetract(),
                        arm.armDown(),*/
                        moveToObs
                )
        );

    }



}
