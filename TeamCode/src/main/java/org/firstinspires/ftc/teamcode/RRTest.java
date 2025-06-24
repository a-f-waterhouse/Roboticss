package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TankDrive;

// TODO tune max velocity - increase it?
// Add limits
// Finish functionality

@Autonomous
public class RRTest extends LinearOpMode
{
    public class Arm
    {
        private final int[] Extensions = {0,0}; // out, in
        private final int[] Rotations = {0,0}; // up, down
        private DcMotorEx extendMotor;
        private  DcMotorEx rotateMotor;
        public Arm(HardwareMap hardwareMap)
        {
            extendMotor = hardwareMap.get(DcMotorEx.class, "arm");
            rotateMotor = hardwareMap.get(DcMotorEx.class, "rotate");

            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public class ArmUp implements Action
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

        public class ArmDown implements Action
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

                if(pos < Rotations[1])
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

        public class ArmExtend implements Action
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

                if (pos < Extensions[0]) {
                    return true;
                } else {
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

                if (pos < Extensions[1]) {
                    return true;
                } else {
                    extendMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action armUp()
        {
            return new ArmUp();
        }
        public Action armDown()
        {
            return new ArmDown();
        }
        public Action armExtend()
        {
            return new ArmExtend();
        }
        public Action armRetract()
        {
            return new ArmRetract();
        }
    }

    public class Claw
    {
        private Servo claw;
        private final int[] Positions = {0,0}; //open, closed

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
        TrajectoryActionBuilder tabTest = drive.actionBuilder(initialPos)
                .lineToY(-39)
                .turn(Math.toRadians(180));
        Action trajectory = tabTest.build();

        telemetry.addData("Starting Position", initialPos);
        telemetry.update();

        //Actions.runBlocking(claw.closeClaw());

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(initialPos)
                        .lineToY(-49)
                        /*.waitSeconds(2)
                        .splineTo(new Vector2d(36,-24), Math.toRadians(70))
                        .splineTo(new Vector2d(43,-12), Math.toRadians(90))
                        .lineToY(-47)
                        .turn(180)
                        .splineTo(new Vector2d(9,-39),Math.toRadians(-90))*/
                        .build());



        /*
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );*/
    }



}
