package segway.controller;

import segway.Constants;

/**
 * Standard Lego EV3 31313 wheels
 * @author Max Morozov
 */
public class EV3Wheels {
    protected static final float K_F1 = -0.770037357557574F * Constants.DEG2RAD;
    protected static final float K_F2 = -84.1899879423200F * Constants.DEG2RAD;
    protected static final float K_F3 = -1.22027762814827F * Constants.DEG2RAD;
    protected static final float K_F4 = -9.28721276122188F * Constants.DEG2RAD;

    protected static final float K_I = -0.416689176972853F * Constants.DEG2RAD; // servo control integral gain

    protected static final float K_THETADOT = 9.30232558139535F / Constants.DEG2RAD;   // forward target speed gain 0.2 m/s
}
