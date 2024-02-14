package frc.robot.ultrashot;

public class AngleStates {

    private double  theta, // Radians robot
                    omega, // Radians per second robot
                    phi, // Radians shooter
                    psi; // Radians per second shooter

    public AngleStates() {
        this.theta = 0.0;
        this.omega = 0.0;
        this.phi = 0.0;
        this.psi = 0.0;
    }

    public AngleStates(double theta, double omega, double phi, double psi) {
        this.theta = theta;
        this.omega = omega;
        this.phi = phi;
        this.psi = psi;
    }

    public double getTheta() {
        return this.theta;
    }

    public double getOmega() {
        return this.omega;
    }

    public double getPhi() {
        return this.phi;
    }

    public double getPsi() {
        return this.psi;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public void setOmega(double omega) {
        this.omega = omega;
    }

    public void setPhi(double phi) {
        this.phi = phi;
    }

    public void setPsi(double psi) {
        this.psi = psi;
    }

    public void setAll(double theta, double omega, double phi, double psi) {
        this.theta = theta;
        this.omega = omega;
        this.phi = phi;
        this.psi = psi;
    }
    
}
