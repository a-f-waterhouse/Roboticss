package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp

public class NO_LIMITS extends LinearOpMode {

    private DcMotor left_motor;
    private DcMotor right_motor;
    private DcMotor arm_motor;
    private DcMotor arm_rotate_motor;
    private Servo claw_servo;
    private final ElapsedTime     bPressed = new ElapsedTime();
    private final ElapsedTime     aPressed = new ElapsedTime();
    private final ElapsedTime     yPressed = new ElapsedTime();
    private double DegreesToPos(double degrees) //0 = -135, 1 = 135
    {
        return (degrees + 135)/270;
    }

    @Override
    public void runOpMode() {
        left_motor = hardwareMap.get(DcMotor.class, "left");
        right_motor = hardwareMap.get(DcMotor.class, "right");
        claw_servo = hardwareMap.get(Servo.class, "claw");
        arm_motor = hardwareMap.get(DcMotor.class, "arm");
        arm_rotate_motor = hardwareMap.get(DcMotor.class, "armRotate");

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "initialised");
        telemetry.update();

        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        arm_rotate_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        arm_rotate_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done


        waitForStart();

        int[]  servoPos = {30,-30};
        int[] armPos = {480,5};
        int[] armRotatePos = {-1350, -5}; //out in
        int s = 0;
        double arm_rotate_power = 0;

        while(opModeIsActive())
        {
            if(gamepad1.b)
            {
                if(bPressed.seconds() >= 0.5)
                {
                    s++;
                    s%=2;
                    bPressed.reset();
                }
            }
            if(gamepad1.dpad_up)
            {
                yPressed.reset();
                arm_rotate_power = 0.8;
            }
            else if (gamepad1.dpad_down)
            {
                aPressed.reset();
                arm_rotate_power = -0.45;
            }
            else if(aPressed.seconds() >= 0.5 || yPressed.seconds() >= 0.5)
            {
                arm_rotate_power = 0;
            }

            double x = (gamepad1.left_stick_x) ;
            double y = (-gamepad1.left_stick_y);
            double armPower = -gamepad1.right_stick_y;

            right_motor.setPower(y-x);
            left_motor.setPower(y+x);
            claw_servo.setPosition(DegreesToPos(servoPos[s]));
            arm_motor.setPower(armPower/5);
            arm_rotate_motor.setPower(arm_rotate_power);

            telemetry.addData("Status", "running");
            telemetry.addData("Drive motors [L/R]","%d | %d", left_motor.getCurrentPosition(), right_motor.getCurrentPosition());
            telemetry.addData("Arm [Extend/Rotate]", "%d | %d", arm_motor.getCurrentPosition(), arm_rotate_motor.getCurrentPosition());
            telemetry.update();
        }
    }

}
