package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.Math;

@TeleOp(name = "TeleOp (no arm)")
public class test_thingy extends OpMode {
    //name motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor ArmMotor;
    //initialize
    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        telemetry.addData("Hardware", "much gud");
    }

        //loop start
        @Override
        public void loop () {

            float y = gamepad1.left_stick_y;
            float x = gamepad1.left_stick_x;
            float t = gamepad1.right_stick_x;
            ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //up and down, left right, and turning
            motorFL.setPower(y - x - t);
            motorFR.setPower(-y - x - t);
            motorBL.setPower(y + x - t);
            motorBR.setPower(-y + x - t);

            //Arm up and down (WIP)

            boolean raise_arm = false;

            // If triangle button is pressed, increase arm motor position
            if(gamepad2.triangle) {
                encoder(2);
            }

            // If arm motor has reached target position, stop movement
            if (ArmMotor.getCurrentPosition() >= ArmMotor. getTargetPosition()) {
                ArmMotor.setPower(0);
            }
            telemetry.addData("Motor ticks:", ArmMotor.getCurrentPosition());

        }

        public void encoder(int turnage)
        {
            // Number of ticks associated with the motor
            double ticks = 537.7;
            // Target position is current position + (ticks/rotation * number of rotations)
            double newTarget = ArmMotor.getCurrentPosition() + (ticks * turnage);

            // Set arm motor target position and power
            ArmMotor.setTargetPosition((int) newTarget);
            ArmMotor.setPower(0.5);
        }
}
