package frc.lib.romiBase.auto;

import frc.lib.muchspeedAuto.paths.PathConfig;

public class DefaultPathConfig extends PathConfig {
    public DefaultPathConfig(){
        TRACK_W_METERS      = 0.142072613;
        KS                  = 0.929;
        KV                  = 6.33;
        KA                  = 0.0389;
        KP                  = 0.085;
        kI                  = 0;
        kD                  = 0;
        MAX_V               = 10;
        MAX_ACCEL           = 0.2;
        MAX_VEL             = 0.5;
        RAMSETE_B           = 2;
        RAMSETE_ZETA        = 0.7;
    }
}
