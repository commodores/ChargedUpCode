package frc.lib.util;

import com.revrobotics.SparkMaxPIDController;

public class RevPIDGains {
    public final double p;
    public final double i;
    public final double d;

    public RevPIDGains(double _p, double _i, double _d) {
        p = _p;
        i = _i;
        d = _d;
    }

    public static void setSparkMaxGains(SparkMaxPIDController _controller, RevPIDGains _gains) {
        _controller.setP(_gains.p);
        _controller.setI(_gains.i);
        _controller.setD(_gains.d);
    }
}