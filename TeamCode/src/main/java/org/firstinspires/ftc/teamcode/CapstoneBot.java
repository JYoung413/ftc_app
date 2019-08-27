package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Capstone Opmode", group = "Capstone")
public class CapstoneBot extends OpMode {
    
    private DcMotor wheelA;
    private DcMotor wheelB;
    private DcMotor wheelC;

    private CapstoneGyro gyro;

    private double desiredHeading = 0.0;
    private double gyroHeading = 0.0;
    private double lastGyroError = 0.0;

    private final double dt = 0.02;
    private final double gP = 0.015;
    private final double gD = 0.0005;

    double aPow = 0.;
    double bPow = 0.;
    double cPow = 0.;

    @Override
    public void init() {
        wheelA = hardwareMap.get(DcMotor.class, "a");
        wheelB = hardwareMap.get(DcMotor.class, "b");
        wheelC = hardwareMap.get(DcMotor.class, "c");
        gyro = new CapstoneGyro(hardwareMap.get(BNO055IMU.class, "gyro"));
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        gyroHeading = getGyroAngle();
        desiredHeading = getGyroAngle();
        lastGyroError = 0.0;
    }

    public double getGyroAngle() {
        return -gyro.getAngle();
    }
    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;
        telemetry.addData("gyro", gyro.getAngle());
        if(Math.abs(r) > 0.1) {
            aPow = (-1/2 * x) - (Math.sqrt(3)/ 2 * y) + r;
            bPow = (-1/2 * x) + (Math.sqrt(3) / 2 * y) + r;
            cPow = -x + r;
            desiredHeading = getGyroAngle();
        } else if (Math.abs(r) <= 0.1) {
            gyroHeading = getGyroAngle();
            double angleDifference = desiredHeading - gyroHeading;
            if(Math.abs(angleDifference) > 5.0) { // tolerance check
                double turn = gP * angleDifference + (gD *
                        ((angleDifference - lastGyroError) / dt));
                aPow = (-1 / 2 * x) - (Math.sqrt(3) / 2 * y) + turn;
                bPow = (-1 / 2 * x) + (Math.sqrt(3) / 2 * y) + turn;
                cPow = -x + turn;
                lastGyroError = angleDifference;
            } else {
                aPow = (-1 / 2 * x) - (Math.sqrt(3) / 2 * y);
                bPow = (-1 / 2 * x) + (Math.sqrt(3) / 2 * y);
                cPow = -x;
            }
        }

        wheelA.setPower(aPow);
        wheelB.setPower(bPow);
        wheelC.setPower(cPow);
    }

    @Override
    public void stop() {
    }
}
