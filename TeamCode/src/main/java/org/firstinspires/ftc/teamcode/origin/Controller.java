package org.firstinspires.ftc.teamcode.origin;

import static org.firstinspires.ftc.teamcode.origin.Utilize.SigNum;

import com.qualcomm.robotcore.util.Range;

public class Controller {
    public double Kp, Ki, Kd, Kf, baseSpeed, Setpoint, Error, LastError, ErrorTolerance, i_min, i_max;
    public double Integral, Derivative, Dt, LastTime, BaseSpeed;
    public Double Main_i_min = -1.0, Main_i_max = 1.0;


    public Controller(double Kp, double Ki, double Kd, double Kf, double baseSpeed, double errorTolerance, Double i_max, Double i_min) {
        if (i_max == null && i_min == null){
            i_max = Main_i_max;
            i_min = Main_i_min;
        }
        setPIDF(Kp, Ki, Kd, Kf, i_max, i_min);
        this.baseSpeed = Range.clip(Math.abs(baseSpeed), 0, 0.5);
        this.Setpoint = this.Error = this.LastError = this.Integral = this.Derivative = this.Dt = this.LastTime = 0;
        this.ErrorTolerance = Math.abs(errorTolerance);
    }

    public void setPIDF(double Kp, double Ki, double Kd, double Kf, double i_max, double i_min) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.i_max = i_max;
        this.i_min = i_min;
    }

    public double Calculate(double setpoint, double current) {
        Setpoint = setpoint;
        return Calculate(setpoint - current);
    }

    public double Calculate(double error) {
        double CurrentTime = System.nanoTime() * 1E-9;
        if (LastTime == 0) LastTime = CurrentTime;
        this.Dt         = CurrentTime - LastTime;
        this.LastTime    = CurrentTime;
        this.Error       = error;  // Error = Setpoint - Current
        this.Integral    = this.Integral + (Error * Dt);
        if (!((i_max == -1) && (i_min == -1))) {
            this.Integral = Range.clip(this.Integral, i_min, i_max);
            boolean Is_Error_In_Tolerance = atSetpoint();
            if (Is_Error_In_Tolerance) {
                this.Integral = 0;
                return 0;
            }
        }
        this.Derivative  = Math.abs(Dt) > 1E-6 ? (Error - LastError) / Dt : 0;
        this.LastError   = Error;
        this.BaseSpeed   = baseSpeed * SigNum(error);
        return (this.Error * this.Kp) + (this.Integral * this.Ki) + (this.Derivative * this.Kd) + (this.Setpoint * this.Kf);
    }

    public boolean atSetpoint() { return Math.abs(this.Error) < this.ErrorTolerance; }
}
