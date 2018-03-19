/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
        } else if (Math.abs(r) <= 0.1){
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
